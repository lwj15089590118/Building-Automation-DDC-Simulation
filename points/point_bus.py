# -*- coding: utf-8 -*-
"""
points/point_bus.py —— 共享 I/O 映像区 PointBus（线程安全）
============================================================

PointBus 是整个仿真系统的"数据总线"，等价于真实 DDC 的 I/O 板卡映像区：

  - plant(受控对象) 写 AI（传感器测量值）、DI（设备反馈）；
  - ddc(控制器)     读 AI/DI，写 AO（执行器指令）、DO（启停指令）；
  - Modbus 从站(modbus_slave.py)把本映像区暴露给外部主站；上位机的 SP
    下发也写进本区，DDC 每个扫描周期都会读取，实现"设定值在线修改"。

所有读写均加互斥锁，保证 Flask 多线程 / Modbus 线程 / 仿真线程并发安全。
"""

import math
import threading

from points.point_defs import ALL_POINTS, POINT_AI


class PointBus:
    """共享 I/O 映像区：plant 写 AI/DI，ddc 读 AI/DI 写 AO/DO，Modbus 对外暴露。"""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        # 工程值映像：addr -> float（AI/AO）；DI/DO 用 0.0/1.0 表示
        self._eng: dict[str, float] = {p.addr: 0.0 for p in ALL_POINTS}
        self._fault: dict[str, bool] = {p.addr: False for p in POINT_AI}  # AI 故障标志
        # 初始合理值，避免未赋值时被误判为故障
        self._eng["AI4"] = 25.0    # 室外温度
        self._eng["AI6"] = 7.0     # 冷冻水供水温度
        self._eng["DI1"] = 1.0     # 防火阀正常
        self._eng["DI2"] = 1.0     # 自动模式
        self._eng["DI3"] = 1.0     # 风机无故障
        self._eng["DI4"] = 1.0     # 未到高液位
        # ---- 上位机接口区（HR 0x0010~0x0015）----
        self._manual_sp: list[float] = [24.0, 24.0, 24.0]  # 手动设定温度(℃)
        self._sp_manual_bits: int = 0                       # bit0/1/2 各房间手动SP使能
        self._energy_saving: bool = True                    # 节能模式开关
        self._auto_mode: bool = True                        # 手/自动(时间表)模式
        self._sys_enable_req: bool = False                  # 手动模式的系统启停请求

    # -------------------- 基础读写 --------------------
    def write(self, addr: str, value: float) -> None:
        """写工程值（plant 写 AI/DI，ddc 写 AO/DO）。"""
        with self._lock:
            if addr.startswith("AI") and isinstance(value, float) and math.isnan(value):
                self._eng[addr] = 0.0
                self._fault[addr] = True          # NaN → 标记为传感器故障
            else:
                self._eng[addr] = float(value)
                if addr.startswith("AI"):
                    self._fault[addr] = False

    def read(self, addr: str) -> float:
        """读工程值；传感器故障时返回 NaN。"""
        with self._lock:
            if addr.startswith("AI") and self._fault[addr]:
                return float("nan")
            return self._eng[addr]

    def read_bool(self, addr: str) -> bool:
        """
        按布尔语义读开关量(DI/DO)：工程值 >= 0.5 视为 True。
        统一收口"0.5 阈值判断"，避免调用方各自硬编码魔法数字。
        """
        return self.read(addr) >= 0.5

    def read_ai(self, addr: str) -> float | None:
        """
        读模拟量输入(AI)，传感器故障(NaN)时返回 None 而不是 NaN，
        调用方用 `is None` 判断坏值，避免各处重复 `v != v` 写法。
        """
        v = self.read(addr)
        return None if v != v else v

    def read_all(self) -> dict[str, float]:
        """一次性快照全部工程值（看板轮询用）。"""
        with self._lock:
            snap = {}
            for addr in self._eng:
                if addr.startswith("AI") and self._fault[addr]:
                    snap[addr] = float("nan")
                else:
                    snap[addr] = self._eng[addr]
            return snap

    # -------------------- 上位机接口区 --------------------
    def set_manual_sp(self, room_idx: int, sp: float) -> None:
        """设置某房间的手动设定温度（上位机经 HR16~18 写入）。"""
        with self._lock:
            self._manual_sp[room_idx] = min(32.0, max(18.0, sp))

    def get_manual_sp(self, room_idx: int) -> float:
        with self._lock:
            return self._manual_sp[room_idx]

    def set_sp_manual_bits(self, bits: int) -> None:
        """设置手动 SP 使能位（bit0 办公室 / bit1 会议室 / bit2 大堂）。"""
        with self._lock:
            self._sp_manual_bits = bits & 0x07

    def get_sp_manual_bits(self) -> int:
        with self._lock:
            return self._sp_manual_bits

    def is_sp_manual(self, room_idx: int) -> bool:
        with self._lock:
            return bool((self._sp_manual_bits >> room_idx) & 0x01)

    def set_mode_bits(self, bits: int) -> None:
        """设置控制模式字：bit0=节能模式，bit1=自动模式。"""
        with self._lock:
            self._energy_saving = bool(bits & 0x01)
            self._auto_mode = bool(bits & 0x02)

    def get_mode_bits(self) -> int:
        with self._lock:
            bits = (0x01 if self._energy_saving else 0x00) | (0x02 if self._auto_mode else 0x00)
            return bits

    @property
    def energy_saving(self) -> bool:
        with self._lock:
            return self._energy_saving

    @property
    def auto_mode(self) -> bool:
        with self._lock:
            return self._auto_mode

    def set_sys_enable_req(self, req: bool) -> None:
        """手动模式下的系统启停请求。"""
        with self._lock:
            self._sys_enable_req = req

    @property
    def sys_enable_req(self) -> bool:
        with self._lock:
            return self._sys_enable_req
