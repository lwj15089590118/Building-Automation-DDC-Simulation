# -*- coding: utf-8 -*-
"""
ddc/interlock.py —— 风机链路联锁状态机（独立模块）
====================================================

把"设备能不能启动、按什么顺序启动"从控制算法中独立出来，
对应真实工程图纸上的联锁时序图：

  启动顺序：新风阀开 → 延时(damper_delay) → 送/排风机启动
            → 延时(fan_delay, 确认无故障) → 冷冻水泵投入(RUNNING)
  停止顺序：退水泵/关水阀 → 延时 → 停风机 → 延时 → 关新风阀
  安全联锁：防火阀关闭(DI1=0)任何状态下立即停一切设备并锁定，
            恢复前禁止重启（火灾工况不允许反复开机送风）。

本模块只回答一个问题：step() 之后是否允许冷冻水阀输出冷量(cooling_allowed)。
"""

from ddc.alarms import AlarmManager
from ddc.simtime import SimTime
from points.point_bus import PointBus


class FanInterlock:
    """风机链路联锁顺序状态机。"""

    # ---------- 状态机状态 ----------
    ST_STOPPED = "STOPPED"                # 系统停止(全部输出断开)
    ST_DAMPER_OPENING = "DAMPER_OPENING"  # 新风阀已开、延时中(等风道建立通路)
    ST_FAN_STARTING = "FAN_STARTING"      # 风机已启动、延时中(确认运行正常)
    ST_RUNNING = "RUNNING"                # 系统正常运行(水泵投入、允许阀输出)
    ST_STOPPING = "STOPPING"              # 反序停机第1步：退出冷冻水，风机吹扫
    ST_DAMPER_CLOSING = "DAMPER_CLOSING"  # 反序停机第2步：风机已停、延时后关新风阀

    def __init__(self, bus: PointBus, alarms: AlarmManager, *,
                 damper_delay: int = 2,
                 fan_delay: int = 1,
                 stop_delay: int = 1) -> None:
        self.bus = bus
        self._alarms = alarms
        self.damper_delay = damper_delay
        self.fan_delay = fan_delay
        self.stop_delay = stop_delay
        self.state = FanInterlock.ST_STOPPED
        self._state_timer = 0            # 当前状态已持续分钟数
        self._fire_lockout = False       # 防火阀联锁锁定(需恢复正常才解锁)
        self.interlock_count = 0         # 联锁动作次数
        self.startup_count = 0           # 完整启动次数

    def step(self, now: SimTime, run_request: bool) -> bool:
        """
        推进一个扫描周期。
        :param now:          当前仿真时刻（用于报警时间戳）
        :param run_request:  上层(时间表/手动)的运行请求
        :return: cooling_allowed —— 是否允许冷冻水阀输出冷量
        """
        fire_closed = not self.bus.read_bool("DI1")    # 0 = 防火阀已关闭

        # ---------------- 防火阀联锁（最高优先级）----------------
        if fire_closed:
            if self.state != FanInterlock.ST_STOPPED or not self._fire_lockout:
                # 立即停一切设备（不经反序停机流程——安全联锁必须瞬时执行）
                self.state = FanInterlock.ST_STOPPED
                self._state_timer = 0
                self._write_fan_outputs(False, False, False, 0.0)
                if not self._fire_lockout:
                    self.interlock_count += 1
                    self._alarms.trigger(now, "fire_interlock",
                                         "重要", "DI1",
                                         "防火阀关闭联锁动作：立即停风机/水泵并关闭新风阀")
                self._fire_lockout = True
            return False

        # 防火阀恢复正常 → 解除锁定，允许时间表重新启动系统
        if self._fire_lockout:
            self._fire_lockout = False
            self._alarms.clear("fire_interlock")

        # ---------------- 正常启动/停止序列 ----------------
        if self.state == FanInterlock.ST_STOPPED:
            self._write_fan_outputs(False, False, False, 0.0)
            if run_request:
                self.state = FanInterlock.ST_DAMPER_OPENING   # 第一步：开新风阀
                self._state_timer = 0
        elif self.state == FanInterlock.ST_DAMPER_OPENING:
            # 新风阀已开，风道建立压差需要时间 → 延时后再启动风机(防止风阀未开带载启动)
            self._write_fan_outputs(True, False, False, 0.0)
            self._state_timer += 1
            if self._state_timer >= self.damper_delay:
                self.state = FanInterlock.ST_FAN_STARTING
                self._state_timer = 0
        elif self.state == FanInterlock.ST_FAN_STARTING:
            # 风机已启动，延时确认运行电流正常后再投入冷冻水(防带故障载冷)
            fan_fault = not self.bus.read_bool("DI3")
            self._write_fan_outputs(True, True, False, 0.0)
            if fan_fault:
                self._alarms.trigger(now, "fan_fault",
                                     "重要", "DI3", "送风机故障反馈，禁止投入冷冻水泵")
                return False
            self._state_timer += 1
            if self._state_timer >= self.fan_delay:
                self.state = FanInterlock.ST_RUNNING
                self._state_timer = 0
                self.startup_count += 1
        elif self.state == FanInterlock.ST_RUNNING:
            self._write_fan_outputs(True, True, True, 50.0)
            if not run_request:
                self.state = FanInterlock.ST_STOPPING   # 进入反序停机
                self._state_timer = 0
        elif self.state == FanInterlock.ST_STOPPING:
            # 反序停机第1步：先退冷冻水泵与水阀(防止盘管凝水/存水)，风机继续吹扫
            self._write_fan_outputs(True, True, False, 0.0)
            self._state_timer += 1
            if self._state_timer >= self.stop_delay:
                self.state = FanInterlock.ST_DAMPER_CLOSING
                self._state_timer = 0
        elif self.state == FanInterlock.ST_DAMPER_CLOSING:
            # 反序停机第2步：停风机(排风/送风)，新风阀延时关闭以利用余压吹干风道
            self._write_fan_outputs(True, False, False, 0.0)
            self._state_timer += 1
            if self._state_timer >= self.stop_delay:
                self._write_fan_outputs(False, False, False, 0.0)
                self.state = FanInterlock.ST_STOPPED
                self._state_timer = 0
        return self.state == FanInterlock.ST_RUNNING

    def _write_fan_outputs(self, damper: bool, fan: bool, pump: bool, freq: float) -> None:
        """一次性写风机链路的全部输出点。"""
        self.bus.write("DO1", 1.0 if damper else 0.0)   # 新风阀
        self.bus.write("DO2", 1.0 if fan else 0.0)      # 送风机
        self.bus.write("DO6", 1.0 if fan else 0.0)      # 排风机(与送风机联动)
        self.bus.write("DO3", 1.0 if pump else 0.0)     # 冷冻水泵
        self.bus.write("AO4", freq)                     # 风机频率
