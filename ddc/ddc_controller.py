# -*- coding: utf-8 -*-
"""
ddc/ddc_controller.py —— DDC 主控制器（策略组合根）
====================================================

【DDC 是什么】
Direct Digital Control：控制器按固定扫描周期(本仿真为 1 分钟)循环执行
"读输入 → 运算 → 写输出"。真实 DDC(如 Honeywell、江森、西门子产品)
用图形化编程工具配置，本项目用 Python 等价实现同样的扫描逻辑。

【职责划分】（每个关注点一个模块，本文件只做策略组合与时间表决策）
  ddc/pid.py        PID 连续调节算法
  ddc/alarms.py     报警队列管理(去重/复位/导出)
  ddc/interlock.py  风机链路联锁顺序状态机
  本文件            时间表(setback/预冷/手动SP)、温度回路死区逻辑、
                    水箱位式控制、报警触发条件判定、输出写总线

【与真实项目的对应关系】
  PointBus.read("AI1")   ≈ DDC 从端子排采样温度传感器 4-20mA/0-10V 信号
  PointBus.write("AO1")  ≈ DDC 向电动水阀输出 0-10V 开度指令
  PointBus.write("DO2")  ≈ DDC 继电器输出驱动风机接触器
"""

import math

from ddc.alarms import AlarmManager
from ddc.interlock import FanInterlock
from ddc.pid import PIDController
from ddc.simtime import SimTime
from points.point_bus import PointBus
from points.point_defs import ROOM_NAMES


# ======================================================================
# DDC 控制器主体
# ======================================================================

class DDCController:
    """
    DDC 控制器：封装全部控制策略，按 scan() 周期运行。

    配置参数（构造时可覆盖默认值）：
      work_start/comfort_start/comfort_end/sys_stop  时间表时刻(分钟)
      setback_sp            夜间节能设定温度 ℃
      deadband              温控死区半宽 ℃（±1℃）
      tank_low/tank_high    水箱位式控制启停液位 m（回差 = high − low）
    """

    # ---------- 联锁状态常量(委托给 FanInterlock，保持旧引用兼容) ----------
    ST_STOPPED = FanInterlock.ST_STOPPED
    ST_DAMPER_OPENING = FanInterlock.ST_DAMPER_OPENING
    ST_FAN_STARTING = FanInterlock.ST_FAN_STARTING
    ST_RUNNING = FanInterlock.ST_RUNNING
    ST_STOPPING = FanInterlock.ST_STOPPING
    ST_DAMPER_CLOSING = FanInterlock.ST_DAMPER_CLOSING

    ROOM_AI = ["AI1", "AI2", "AI3"]
    ROOM_AO = ["AO1", "AO2", "AO3"]

    def __init__(self, bus: PointBus, *,
                 energy_saving: bool = True,
                 work_start: int = 6 * 60 + 50,     # 06:50 系统启动并预冷(优化启动)
                 comfort_start: int = 8 * 60,       # 08:00 工作时间开始
                 comfort_end: int = 18 * 60,        # 18:00 工作时间结束
                 sys_stop: int = 18 * 60 + 30,      # 18:30 系统停机
                 work_sp: float = 24.0,
                 setback_sp: float = 28.0,
                 deadband: float = 1.0,
                 tank_low: float = 1.00,
                 tank_high: float = 1.80,
                 damper_delay: int = 2,
                 fan_delay: int = 1,
                 stop_delay: int = 1,
                 high_temp_limit: float = 28.0,     # 高温报警阈值 ℃
                 low_temp_limit: float = 20.0,      # 低温报警阈值 ℃
                 night_cool_high: float = 29.5,     # 夜间节能模式下触发保护制冷的室温 ℃
                 alarm_hyst: float = 1.0):          # 报警恢复回差 ℃
        self.bus = bus
        # ---------------- 时间表 / 模式参数 ----------------
        self.work_start = work_start
        self.comfort_start = comfort_start
        self.comfort_end = comfort_end
        self.sys_stop = sys_stop
        self.work_sp = work_sp
        self.setback_sp = setback_sp
        self.deadband = deadband
        self.energy_saving_override = energy_saving   # 构造时的节能开关(测试用)
        # ---------------- 水箱位式控制参数 ----------------
        self.tank_low = tank_low
        self.tank_high = tank_high
        self.tank_valve_state = False                  # 进水阀当前指令(False=关)
        # ---------------- 组合子模块 ----------------
        self._alarms = AlarmManager()                          # 报警队列管理
        self._ilock = FanInterlock(bus, self._alarms,          # 风机链路联锁
                                   damper_delay=damper_delay,
                                   fan_delay=fan_delay,
                                   stop_delay=stop_delay)
        # ---------------- 温度回路 ----------------
        # PID 参数按热工对象特性整定：开度 1% 稳态温升约 0.4℃(过程增益)，
        # 房间时间常数约 4 小时；积分时间 Ti=Kp/Ki≈20 分钟，保证负荷爬坡时
        # 能在十几分钟内补足稳态开度，又不产生明显振荡（详见系统设计说明书）。
        self.pids = [
            PIDController(kp=22.0, ki=1.0, kd=15.0),   # 办公室
            PIDController(kp=22.0, ki=1.0, kd=15.0),   # 会议室
            PIDController(kp=18.0, ki=0.8, kd=18.0),   # 大堂(热容大，稍缓)
        ]
        self.current_sp = [work_sp, work_sp, work_sp]  # 当前生效的设定值(供看板)
        self.sensor_hold = [False, False, False]       # 传感器故障期间保持输出标志
        self._high_timer = [0, 0, 0]                   # 各房间高温持续时间计数
        # ---------------- 报警阈值 ----------------
        self.high_temp_limit = high_temp_limit
        self.low_temp_limit = low_temp_limit
        self.night_cool_high = night_cool_high
        self.alarm_hyst = alarm_hyst

    # ==================================================
    # 对外只读访问器（组合子模块的状态与统计）
    # ==================================================
    @property
    def state(self) -> str:
        """联锁状态机当前状态字符串。"""
        return self._ilock.state

    @property
    def interlock_count(self) -> int:
        """联锁动作次数。"""
        return self._ilock.interlock_count

    @property
    def startup_count(self) -> int:
        """机组完整启动次数。"""
        return self._ilock.startup_count

    @property
    def alarm_count(self) -> int:
        """报警发生次数。"""
        return self._alarms.count

    def recent_alarms(self, limit: int | None = None) -> list[dict]:
        """导出报警记录（新在前），供看板/日报等外部展示使用。"""
        return self._alarms.recent(limit)

    # ==================================================
    # ② 时间表运算
    # ==================================================
    def _schedule(self, bus_energy_saving: bool, mod: int):
        """
        根据时间表和模式决定：系统是否应运行(system_enable)、各房间 SP。
        返回 (system_enable, sp_list)
        """
        saving = self.energy_saving_override and bus_energy_saving
        in_work_window = self.work_start <= mod < self.sys_stop      # 系统运行窗口
        in_comfort = self.comfort_start <= mod < self.comfort_end    # 工作时间
        in_precool = (self.work_start <= mod < self.comfort_start)   # 早晨预冷时段
        if in_comfort or not saving:
            sp = self.work_sp          # 工作时间 24℃；节能关→全天 24℃
        elif in_precool:
            # 预冷设定比工作设定低 1℃，提前把房间拉入舒适区间下沿，
            # 消除"上班后负荷爬坡快于调节"造成的早晨超温(优化启动策略)
            sp = self.work_sp - 1.0
        else:
            sp = self.setback_sp       # 夜间 setback 28℃
        sp_list = [sp, sp, sp]

        # ---- 系统使能判断 ----
        enable = False
        if in_work_window:
            enable = True
        elif saving:
            # 夜间停机节能；但若某房间过热(PV>night_cool_high)则投入保护制冷
            pvs = [self.bus.read(a) for a in self.ROOM_AI]
            over = any((not math.isnan(pv)) and pv > self.night_cool_high for pv in pvs)
            if over:
                enable = True
                sp_list = [self.setback_sp] * 3
        else:
            enable = True              # 节能关：系统 24 小时连续运行

        # ---- 上位机手动 SP 覆盖（经 Modbus HR 区下发）----
        for i in range(3):
            if self.bus.is_sp_manual(i):
                sp_list[i] = self.bus.get_manual_sp(i)

        # ---- DI2 手/自动：手动模式下由 HR21 的远程请求决定系统启停 ----
        if self.bus.auto_mode:
            return enable, sp_list
        return self.bus.sys_enable_req, sp_list

    # ==================================================
    # ③a 空调温度回路（PID + 死区）
    # ==================================================
    def _control_rooms(self, sp_list: list[float], cooling_allowed: bool,
                       now: SimTime) -> list[float]:
        """
        三个房间独立 PID 控温。
        死区逻辑(±deadband)：PV < SP−db → 阀全关并复位积分；
                              PV > SP+db → PID 调节；
                              两者之间   → 保持原开度（消除噪声引起的频繁动作）。
        传感器故障：该回路输出保持故障前值(safe hold)，并产生报警。
        :return: 各房间最终 AO 输出(%)
        """
        outputs = []
        for i in range(3):
            pv = self.bus.read(self.ROOM_AI[i])
            sp = sp_list[i]
            self.current_sp[i] = sp
            pid = self.pids[i]

            # ---- 传感器故障检测（NaN 或物理超限）----
            if math.isnan(pv) or pv < -20.0 or pv > 60.0:
                self.sensor_hold[i] = True
                self._alarms.trigger(now, f"sensor_fault_AI{i+1}",
                                     "重要", self.ROOM_AI[i],
                                     f"{ROOM_NAMES[i]}温度传感器故障，"
                                     f"回路输出保持")
                outputs.append(pid.output)          # 安全保持
                continue
            if self.sensor_hold[i]:                 # 故障恢复
                self.sensor_hold[i] = False
                self._alarms.clear(f"sensor_fault_AI{i+1}")
                pid.reset()

            # ---- 高温/低温报警（带持续时间去抖与恢复回差）----
            # 报警监控是连续的，不受系统启停影响：夜间 setback 停机漂移、
            # 防火阀联锁停机期间的室温越限同样必须进入报警队列
            # （规格要求"高温/低温……全部进报警队列"），故此段必须在
            # cooling_allowed 判定之前执行。
            key_hi = f"high_temp_room{i}"
            if pv > self.high_temp_limit:
                self._high_timer[i] += 1
                if self._high_timer[i] >= 5:         # 连续 5 分钟超限才报警
                    self._alarms.trigger(now, key_hi, "警告",
                                         self.ROOM_AI[i],
                                         f"{ROOM_NAMES[i]}高温报警:"
                                         f"PV={pv:.1f}℃>{self.high_temp_limit}℃")
            elif pv < self.high_temp_limit - self.alarm_hyst:
                self._high_timer[i] = 0
                self._alarms.clear(key_hi)

            key_lo = f"low_temp_room{i}"
            if pv < self.low_temp_limit:
                self._alarms.trigger(now, key_lo, "警告",
                                     self.ROOM_AI[i],
                                     f"{ROOM_NAMES[i]}低温报警:"
                                     f"PV={pv:.1f}℃<{self.low_temp_limit}℃")
            elif pv > self.low_temp_limit + self.alarm_hyst:
                self._alarms.clear(key_lo)

            if not cooling_allowed:
                # 系统未投入(停机/联锁中)：阀全关，复位回路。
                # 只跳过调节，不跳过上面的报警监控。
                pid.reset()
                pid.output = 0.0
                outputs.append(0.0)
                continue

            err = pv - sp                            # 制冷偏差
            if pv < sp - self.deadband:
                # 温度已低于下死区 → 制冷关闭
                pid.reset()
                pid.output = 0.0
                out = 0.0
            elif pv > sp + self.deadband:
                out = pid.update(err, dt_min=1.0)    # 上死区外 → PID 调节
            else:
                out = pid.output                     # 死区内 → 保持原开度

            outputs.append(max(0.0, min(100.0, out)))
        return outputs

    # ==================================================
    # ③b 水箱液位回路（位式控制 + 大回差）
    # ==================================================
    def _control_tank(self, now: SimTime) -> None:
        """
        位式(ON/OFF)控制：
          液位 ≤ tank_low(1.0m)  → 开进水阀；
          液位 ≥ tank_high(1.8m) → 关进水阀；
          两者之间               → 保持原状态（回差 0.8m，防阀频繁动作）。
          DI4 高位浮球=0(硬限位) → 强制关阀。
        另附溢流/低液位报警。
        """
        level = self.bus.read("AI5")
        di4_float_ok = self.bus.read_bool("DI4")       # 1=未到高位

        if not di4_float_ok:
            self.tank_valve_state = False              # 浮球硬限位优先
        elif level <= self.tank_low:
            self.tank_valve_state = True               # 低液位开阀充水
        elif level >= self.tank_high:
            self.tank_valve_state = False              # 高液位关阀
        # 中间区间：保持（回差滞环）

        # ---- 水箱相关报警 ----
        if level >= 1.90:
            self._alarms.trigger(now, "tank_overflow", "重要", "AI5",
                                 f"水箱液位过高({level:.2f}m)，存在溢流风险")
        elif level <= 1.70:
            self._alarms.clear("tank_overflow")
        if level <= 0.20:
            self._alarms.trigger(now, "tank_lowlevel", "警告", "AI5",
                                 f"水箱低液位({level:.2f}m)，请检查供水")
        elif level >= 0.50:
            self._alarms.clear("tank_lowlevel")

        self.bus.write("DO4", 1.0 if self.tank_valve_state else 0.0)

    # ==================================================
    # 主扫描函数（DDC 每个周期调用一次）
    # ==================================================
    def scan(self, now: SimTime, dt_min: float = 1.0) -> None:
        """
        执行一个完整扫描周期：读输入 → 时间表 → 回路 → 联锁 → 报警 → 写输出。
        :param now:   当前仿真时刻(SimTime 值对象)
        :param dt_min: 扫描周期对应的仿真分钟数(保留扩展用)
        """
        # ① 读手/自动状态(DI2)与节能模式(HR 模式字)
        # ② 时间表 → 使能与 SP
        run_request, sp_list = self._schedule(self.bus.energy_saving, now.minute)

        # ③b 水箱位式控制（独立于空调系统，24h 运行）
        self._control_tank(now)

        # ④ 联锁状态机 → 是否允许供冷
        cooling_allowed = self._ilock.step(now, run_request)

        # ③a 三房间 PID 控温（仅 RUNNING 时有冷量输出）
        outputs = self._control_rooms(sp_list, cooling_allowed, now)
        for i, out in enumerate(outputs):
            self.bus.write(self.ROOM_AO[i], out)

        # ⑤ 声光报警器联动：有任何活动报警则 DO5=1
        any_active = self._alarms.any_active()
        self.bus.write("DO5", 1.0 if any_active else 0.0)


if __name__ == "__main__":
    # 最小自测：构造总线，手动模拟一天的关键时刻行为
    from plant.thermal import BuildingPlant
    from points.point_defs import print_point_table

    print_point_table()
    bus = PointBus()
    bus.set_mode_bits(0x03)          # 节能开 + 自动
    ddc = DDCController(bus, energy_saving=True)
    plant_mod = BuildingPlant(bus)
    for m in range(1440):
        plant_mod.step(m)
        ddc.scan(SimTime(1, m))
        if m % 120 == 0:
            print(f"{m//60:02d}:{m%60:02d} 状态={ddc.state:<14} "
                  f"PV={[round(bus.read(a),2) for a in ('AI1','AI2','AI3')]} "
                  f"SP={[round(s,1) for s in ddc.current_sp]} "
                  f"AO={[round(bus.read(a),1) for a in ('AO1','AO2','AO3')]} "
                  f"液位={bus.read('AI5'):.2f}")
    print("\n报警队列:")
    for a in ddc.recent_alarms(10)[::-1]:
        print(f"  [{a['time']}] {a['level']} {a['source']} {a['message']}")
    print(f"报警总数={ddc.alarm_count} 联锁次数={ddc.interlock_count} 启动次数={ddc.startup_count}")
