# -*- coding: utf-8 -*-
"""
ddc/ddc_controller.py —— DDC 直接数字控制器程序（项目核心）
============================================================

【DDC 是什么】
Direct Digital Control：控制器按固定扫描周期(本仿真为 1 分钟)循环执行
"读输入 → 运算 → 写输出"，所有回路/联锁/时间表/报警都在这个周期里完成。
真实 DDC(如 Honeywell、江森、西门子产品)用图形化编程工具配置，
本项目用 Python 等价实现同样的扫描逻辑。

【每个扫描周期完成的任务】
  ① 读取 AI/DI（经 PointBus，等价于 DDC 采样现场信号）；
  ② 时间表运算：工作时间 SP=24℃，夜间 setback 至 28℃（节能模式可配）；
     手动 SP（上位机经 Modbus 下发）优先于时间表；
  ③ 回路控制：
     - 空调温度回路 ×3：增量式位置 PID + ±1℃ 死区（带保持区防频繁动作）；
     - 水箱液位回路 ×1：位式(ON/OFF)控制 + 大回差，防止阀门频繁启停；
  ④ 联锁顺序控制：风机启动顺序 = 新风阀开 → 延时 → 送风机启动 → (延时)
     → 冷冻水泵投入；停止顺序相反。防火阀关闭(DI1=0)联锁立即停机并报警；
  ⑤ 报警处理：高温/低温/传感器故障/联锁动作/水箱溢流/液位过低/风机故障，
     全部进入报警队列(去重、可恢复)，并联动声光报警器 DO5；
  ⑥ 写 AO/DO 输出。

【与真实项目的对应关系】
  PointBus.read("AI1")   ≈ DDC 从端子排采样温度传感器 4-20mA/0-10V 信号
  PointBus.write("AO1")  ≈ DDC 向电动水阀输出 0-10V 开度指令
  PointBus.write("DO2")  ≈ DDC 继电器输出驱动风机接触器
"""

import math

from points.point_table import PointBus


# ======================================================================
# 一、PID 控制器（位置式算法 + 抗积分饱和）
# ======================================================================

class PIDController:
    """
    制冷工况 PID：
      偏差 e = PV − SP（PV 越高于设定值，需要越大的阀开度）
      u(k) = Kp·e + Ki·Σ(e·Δt) + Kd·(e−e_{k−1})/Δt ，输出限幅 [0, 100]%
    抗积分饱和：输出到达限幅且偏差继续同向时，冻结积分（防止退饱和超调）。
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = 0.0, out_max: float = 100.0) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0      # 积分累计量 Σ(e·Δt)
        self.prev_err: float | None = None   # 上一拍偏差(None 表示刚复位)
        self.output = 0.0        # 当前输出 %

    def reset(self) -> None:
        """复位（系统重新启动/死区关闭后调用，避免旧积分引起突跳）。"""
        self.integral = 0.0
        self.prev_err = None
        # 注意：output 不清零——死区内要求"保持原开度"

    def update(self, err: float, dt_min: float) -> float:
        """按当前偏差计算新输出（%）。"""
        # ---- 比例项 ----
        p_term = self.kp * err
        # ---- 微分项（对偏差微分，首拍不微分）----
        if self.prev_err is None:
            d_term = 0.0
        else:
            d_term = self.kd * (err - self.prev_err) / dt_min
        self.prev_err = err
        # ---- 试探性加入积分，检查是否饱和 ----
        trial_integral = self.integral + self.ki * err * dt_min
        raw = p_term + trial_integral + d_term
        if (raw > self.out_max and trial_integral > self.integral) or \
           (raw < self.out_min and trial_integral < self.integral):
            pass                      # 饱和且积分还在恶化 → 冻结积分(抗饱和)
        else:
            self.integral = trial_integral
        self.output = max(self.out_min, min(self.out_max,
                                            p_term + self.integral + d_term))
        return self.output


# ======================================================================
# 二、报警记录与队列
# ======================================================================

class AlarmRecord:
    """一条报警记录（进入报警队列）。"""

    __slots__ = ("abs_minute", "time_str", "level", "source", "message")

    def __init__(self, abs_minute: int, time_str: str,
                 level: str, source: str, message: str) -> None:
        self.abs_minute = abs_minute   # 绝对仿真分钟
        self.time_str = time_str       # "第X天 HH:MM"
        self.level = level             # "提示" / "警告" / "重要"
        self.source = source           # 报警来源点位，如 "AI1"
        self.message = message         # 中文报警描述


# ======================================================================
# 三、DDC 控制器主体
# ======================================================================

class DDCController:
    """
    DDC 控制器：封装全部控制策略，按 scan() 周期运行。

    配置参数（构造时可覆盖默认值）：
      work_start/work_end   工作时间（分钟），SP=work_sp
      setback_sp            夜间节能设定温度 ℃
      deadband              温控死区半宽 ℃（±1℃）
      tank_low/tank_high    水箱位式控制启停液位 m（回差 = high − low）
      damper_delay/fan_delay/pump_delay 联锁各步延时（分钟）
    """

    # ---------- 联锁状态机状态 ----------
    ST_STOPPED = "STOPPED"                # 系统停止(全部输出断开)
    ST_DAMPER_OPENING = "DAMPER_OPENING"  # 新风阀已开、延时中(等风道建立通路)
    ST_FAN_STARTING = "FAN_STARTING"      # 风机已启动、延时中(确认运行正常)
    ST_RUNNING = "RUNNING"                # 系统正常运行(水泵投入、允许阀输出)
    ST_STOPPING = "STOPPING"              # 反序停机第1步：退出冷冻水，风机吹扫
    ST_DAMPER_CLOSING = "DAMPER_CLOSING"  # 反序停机第2步：风机已停、延时后关新风阀

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
        # ---------------- 联锁参数与状态机 ----------------
        self.damper_delay = damper_delay
        self.fan_delay = fan_delay
        self.stop_delay = stop_delay
        self.state = DDCController.ST_STOPPED
        self._state_timer = 0                          # 当前状态已持续分钟数
        self._fire_lockout = False                     # 防火阀联锁锁定(需恢复正常才解锁)
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
        self.sp_manual_flag = [False, False, False]
        self.sensor_hold = [False, False, False]       # 传感器故障期间保持输出标志
        # ---------------- 报警 ----------------
        self.high_temp_limit = high_temp_limit
        self.low_temp_limit = low_temp_limit
        self.night_cool_high = night_cool_high
        self.alarm_hyst = alarm_hyst
        self.alarm_queue: list[AlarmRecord] = []       # 报警队列(只增不减，供看板/日报)
        self._active_alarms: dict[str, bool] = {}      # 活动报警去重表
        self._high_timer = [0, 0, 0]                   # 各房间高温持续时间计数
        # ---------------- 统计 ----------------
        self.alarm_count = 0                           # 报警发生次数
        self.interlock_count = 0                       # 联锁动作次数
        self.startup_count = 0                         # 机组启动次数(完整联锁序列)

    # ==================================================
    # 工具函数
    # ==================================================
    @staticmethod
    def fmt_time(day: int, minute_of_day: int) -> str:
        """把仿真时间格式化为 '第X天 HH:MM'。"""
        return f"第{day}天 {minute_of_day // 60:02d}:{minute_of_day % 60:02d}"

    def _raise_alarm(self, abs_minute: int, day: int, mod: int,
                     key: str, level: str, source: str, message: str) -> bool:
        """产生一条报警（同键活动期间去重）。返回是否真正产生了新报警。"""
        if self._active_alarms.get(key):
            return False
        self._active_alarms[key] = True
        self.alarm_queue.append(
            AlarmRecord(abs_minute, self.fmt_time(day, mod), level, source, message))
        self.alarm_count += 1
        return True

    def _clear_alarm(self, key: str) -> None:
        """报警条件消失后清除活动标记（允许下次再报）。"""
        self._active_alarms[key] = False

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
                       abs_minute: int, day: int, mod: int) -> list[float]:
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
                self._raise_alarm(abs_minute, day, mod, f"sensor_fault_AI{i+1}",
                                  "重要", self.ROOM_AI[i],
                                  f"{['办公室','会议室','大堂'][i]}温度传感器故障，"
                                  f"回路输出保持")
                outputs.append(pid.output)          # 安全保持
                continue
            if self.sensor_hold[i]:                 # 故障恢复
                self.sensor_hold[i] = False
                self._clear_alarm(f"sensor_fault_AI{i+1}")
                pid.reset()

            if not cooling_allowed:
                # 系统未投入(停机/联锁中)：阀全关，复位回路
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

            # ---- 高温/低温报警（带持续时间去抖与恢复回差）----
            key_hi = f"high_temp_room{i}"
            if pv > self.high_temp_limit:
                self._high_timer[i] += 1
                if self._high_timer[i] >= 5:         # 连续 5 分钟超限才报警
                    self._raise_alarm(abs_minute, day, mod, key_hi, "警告",
                                      self.ROOM_AI[i],
                                      f"{['办公室','会议室','大堂'][i]}高温报警:"
                                      f"PV={pv:.1f}℃>{self.high_temp_limit}℃")
            elif pv < self.high_temp_limit - self.alarm_hyst:
                self._high_timer[i] = 0
                self._clear_alarm(key_hi)

            key_lo = f"low_temp_room{i}"
            if pv < self.low_temp_limit:
                self._raise_alarm(abs_minute, day, mod, key_lo, "警告",
                                  self.ROOM_AI[i],
                                  f"{['办公室','会议室','大堂'][i]}低温报警:"
                                  f"PV={pv:.1f}℃<{self.low_temp_limit}℃")
            elif pv > self.low_temp_limit + self.alarm_hyst:
                self._clear_alarm(key_lo)

            outputs.append(max(0.0, min(100.0, out)))
        return outputs

    # ==================================================
    # ③b 水箱液位回路（位式控制 + 大回差）
    # ==================================================
    def _control_tank(self, abs_minute: int, day: int, mod: int) -> None:
        """
        位式(ON/OFF)控制：
          液位 ≤ tank_low(1.0m)  → 开进水阀；
          液位 ≥ tank_high(1.8m) → 关进水阀；
          两者之间               → 保持原状态（回差 0.8m，防阀频繁动作）。
          DI4 高位浮球=0(硬限位) → 强制关阀。
        另附溢流/低液位报警。
        """
        level = self.bus.read("AI5")
        di4_float_ok = self.bus.read("DI4") >= 0.5     # 1=未到高位

        if not di4_float_ok:
            self.tank_valve_state = False              # 浮球硬限位优先
        elif level <= self.tank_low:
            self.tank_valve_state = True               # 低液位开阀充水
        elif level >= self.tank_high:
            self.tank_valve_state = False              # 高液位关阀
        # 中间区间：保持（回差滞环）

        # ---- 水箱相关报警 ----
        if level >= 1.90:
            self._raise_alarm(abs_minute, day, mod, "tank_overflow", "重要", "AI5",
                              f"水箱液位过高({level:.2f}m)，存在溢流风险")
        elif level <= 1.70:
            self._clear_alarm("tank_overflow")
        if level <= 0.20:
            self._raise_alarm(abs_minute, day, mod, "tank_lowlevel", "警告", "AI5",
                              f"水箱低液位({level:.2f}m)，请检查供水")
        elif level >= 0.50:
            self._clear_alarm("tank_lowlevel")

        self.bus.write("DO4", 1.0 if self.tank_valve_state else 0.0)

    # ==================================================
    # ④ 联锁顺序控制（风机启动顺序状态机）
    # ==================================================
    def _interlock_sequence(self, run_request: bool,
                            abs_minute: int, day: int, mod: int) -> bool:
        """
        风机启动联锁状态机。
        启动顺序：新风阀开 → 延时(damper_delay) → 送/排风机启动 → 延时(fan_delay)
                   → 冷冻水泵投入(系统进入 RUNNING，允许阀开度输出)
        停止顺序：关水泵/水阀 → 延时 → 停风机 → 延时 → 关新风阀
        防火阀联锁(DI1=0)：任何状态下立即停风机/水泵/关阀，并产生重要报警；
                           恢复前禁止再次启动(锁定，防止反复重启损坏设备)。
        :return: cooling_allowed —— 是否允许冷冻水阀输出冷量
        """
        fire_closed = self.bus.read("DI1") < 0.5       # 0 = 防火阀已关闭

        # ---------------- 防火阀联锁（最高优先级）----------------
        if fire_closed:
            if self.state != DDCController.ST_STOPPED or not self._fire_lockout:
                # 立即停一切设备（不经反序停机流程——安全联锁必须瞬时执行）
                self.state = DDCController.ST_STOPPED
                self._state_timer = 0
                self._write_fan_outputs(False, False, False, 0.0)
                if not self._fire_lockout:
                    self.interlock_count += 1
                    self._raise_alarm(abs_minute, day, mod, "fire_interlock",
                                      "重要", "DI1",
                                      "防火阀关闭联锁动作：立即停风机/水泵并关闭新风阀")
                self._fire_lockout = True
            return False

        # 防火阀恢复正常 → 解除锁定，允许时间表重新启动系统
        if self._fire_lockout:
            self._fire_lockout = False
            self._clear_alarm("fire_interlock")

        # ---------------- 正常启动/停止序列 ----------------
        if self.state == DDCController.ST_STOPPED:
            self._write_fan_outputs(False, False, False, 0.0)
            if run_request:
                self.state = DDCController.ST_DAMPER_OPENING   # 第一步：开新风阀
                self._state_timer = 0
        elif self.state == DDCController.ST_DAMPER_OPENING:
            # 新风阀已开，风道建立压差需要时间 → 延时后再启动风机(防止风阀未开带载启动)
            self._write_fan_outputs(True, False, False, 0.0)
            self._state_timer += 1
            if self._state_timer >= self.damper_delay:
                self.state = DDCController.ST_FAN_STARTING
                self._state_timer = 0
        elif self.state == DDCController.ST_FAN_STARTING:
            # 风机已启动，延时确认运行电流正常后再投入冷冻水(防带故障载冷)
            fan_fault = self.bus.read("DI3") < 0.5
            self._write_fan_outputs(True, True, False, 0.0)
            if fan_fault:
                self._raise_alarm(abs_minute, day, mod, "fan_fault",
                                  "重要", "DI3", "送风机故障反馈，禁止投入冷冻水泵")
                return False
            self._state_timer += 1
            if self._state_timer >= self.fan_delay:
                self.state = DDCController.ST_RUNNING
                self._state_timer = 0
                self.startup_count += 1
        elif self.state == DDCController.ST_RUNNING:
            self._write_fan_outputs(True, True, True, 50.0)
            if not run_request:
                self.state = DDCController.ST_STOPPING   # 进入反序停机
                self._state_timer = 0
        elif self.state == DDCController.ST_STOPPING:
            # 反序停机第1步：先退冷冻水泵与水阀(防止盘管凝水/存水)，风机继续吹扫
            self._write_fan_outputs(True, True, False, 0.0)
            self._state_timer += 1
            if self._state_timer >= self.stop_delay:
                self.state = DDCController.ST_DAMPER_CLOSING
                self._state_timer = 0
        elif self.state == DDCController.ST_DAMPER_CLOSING:
            # 反序停机第2步：停风机(排风/送风)，新风阀延时关闭以利用余压吹干风道
            self._write_fan_outputs(True, False, False, 0.0)
            self._state_timer += 1
            if self._state_timer >= self.stop_delay:
                self._write_fan_outputs(False, False, False, 0.0)
                self.state = DDCController.ST_STOPPED
                self._state_timer = 0
        return self.state == DDCController.ST_RUNNING

    def _write_fan_outputs(self, damper: bool, fan: bool, pump: bool, freq: float) -> None:
        """一次性写风机链路的全部输出点。"""
        self.bus.write("DO1", 1.0 if damper else 0.0)   # 新风阀
        self.bus.write("DO2", 1.0 if fan else 0.0)      # 送风机
        self.bus.write("DO6", 1.0 if fan else 0.0)      # 排风机(与送风机联动)
        self.bus.write("DO3", 1.0 if pump else 0.0)     # 冷冻水泵
        self.bus.write("AO4", freq)                     # 风机频率

    # ==================================================
    # 主扫描函数（DDC 每个周期调用一次）
    # ==================================================
    def scan(self, day: int, minute_of_day: int, dt_min: float = 1.0) -> None:
        """
        执行一个完整扫描周期：读输入 → 时间表 → 回路 → 联锁 → 报警 → 写输出。
        :param day:           仿真第几天(从 1 开始)
        :param minute_of_day: 当天第几分钟(0~1439)
        """
        abs_minute = (day - 1) * 1440 + minute_of_day
        mod = minute_of_day

        # ① 读手/自动状态(DI2)与节能模式(HR 模式字)
        # ② 时间表 → 使能与 SP
        run_request, sp_list = self._schedule(self.bus.energy_saving, mod)

        # ③b 水箱位式控制（独立于空调系统，24h 运行）
        self._control_tank(abs_minute, day, mod)

        # ④ 联锁状态机 → 是否允许供冷
        cooling_allowed = self._interlock_sequence(run_request, abs_minute, day, mod)

        # ③a 三房间 PID 控温（仅 RUNNING 时有冷量输出）
        outputs = self._control_rooms(sp_list, cooling_allowed,
                                      abs_minute, day, mod)
        for i, out in enumerate(outputs):
            self.bus.write(self.ROOM_AO[i], out)

        # ⑤ 声光报警器联动：有任何活动报警则 DO5=1
        any_active = any(self._active_alarms.values())
        self.bus.write("DO5", 1.0 if any_active else 0.0)


if __name__ == "__main__":
    # 最小自测：构造总线，手动模拟一天的关键时刻行为
    from points.point_table import print_point_table

    print_point_table()
    bus = PointBus()
    bus.set_mode_bits(0x03)          # 节能开 + 自动
    ddc = DDCController(bus, energy_saving=True)
    plant_mod = __import__("plant.thermal", fromlist=["BuildingPlant"]).BuildingPlant(bus)
    for m in range(1440):
        plant_mod.step(m)
        ddc.scan(1, m)
        if m % 120 == 0:
            print(f"{m//60:02d}:{m%60:02d} 状态={ddc.state:<14} "
                  f"PV={[round(bus.read(a),2) for a in ('AI1','AI2','AI3')]} "
                  f"SP={[round(s,1) for s in ddc.current_sp]} "
                  f"AO={[round(bus.read(a),1) for a in ('AO1','AO2','AO3')]} "
                  f"液位={bus.read('AI5'):.2f}")
    print("\n报警队列:")
    for a in ddc.alarm_queue[:10]:
        print(f"  [{a.time_str}] {a.level} {a.source} {a.message}")
    print(f"报警总数={ddc.alarm_count} 联锁次数={ddc.interlock_count} 启动次数={ddc.startup_count}")
