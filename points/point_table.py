# -*- coding: utf-8 -*-
"""
points/point_table.py —— BA 点表定义 + 共享 I/O 映像区(PointBus) + Modbus/TCP 从站
=================================================================================

【行业背景】
在真实楼宇自控(BA)项目中，DDC 控制器通过点表(Point List)管理所有现场点位：
  - AI  (Analog Input,  模拟量输入)：温度、湿度、液位、压力等传感器信号；
  - AO  (Analog Output, 模拟量输出)：电动阀开度、风机频率等执行器指令；
  - DI  (Digital Input, 数字量输入)：防火阀反馈、手/自动开关、故障反馈等状态信号；
  - DO  (Digital Output,数字量输出)：风机启停、水泵启停、电动阀开关等指令。

本模块用"共享 I/O 映像区(PointBus)"模拟 DDC 的 I/O 板卡：
  - plant(受控对象) 把传感器测量值写入 AI 区、设备状态写入 DI 区；
  - ddc(控制器)     从 AI/DI 读入，运算后把指令写进 AO/DO 区；
  - Modbus/TCP 从站 把同一映像区暴露给外部主站（上位机/BAS 平台），
    功能码映射：AI→输入寄存器(fc04)，AO→保持寄存器(fc03/06/16)，
               DI→离散输入(fc02)，DO→线圈(fc01/05)。
这样"仿真世界"与"通信世界"共用一份数据，与真实系统结构一致。

【寄存器地址映射表】(从站地址 Slave ID = 1，zero_mode，地址从 0 开始)
  输入寄存器 IR (fc04 读)：
      0x0000~0x0005 : AI1~AI6   工程值×10 取整(温度0.1℃分辨率；液位0.01m)
  保持寄存器 HR (fc03 读 / fc06、fc16 写)：
      0x0000~0x0003 : AO1~AO4   工程值×10 取整(开度0.1%、频率0.1Hz)
      0x0010~0x0012 : SP1~SP3   各房间温度设定值×10 (上位机在线修改)
      0x0013        : SP 手动使能位 bit0/1/2 → 办公室/会议室/大堂(1=手动SP生效)
      0x0014        : 控制模式字 bit0=节能模式(1开) bit1=手自动(1自动)
      0x0015        : 系统使能请求(手动模式下 1=启动 0=停止)
  离散输入 DI (fc02 读)：
      0x0000~0x0003 : DI1~DI4   (0/1)
  线圈 DO (fc01 读 / fc05 写)：
      0x0000~0x0005 : DO1~DO6   (0/1)

【运行方式】本文件既是库(被 plant/ddc/dashboard/run_day 导入 PointBus)，
也可以独立运行：`python -m points.point_table` 启动一个带演示数据的 Modbus 从站。
"""

import asyncio
import math
import threading
from dataclasses import dataclass

# ----------------------------------------------------------------------
# pymodbus 延迟导入说明：PointBus 是纯 Python 实现，不依赖 pymodbus；
# 只有真正启动 Modbus 从站时才需要 pymodbus，便于无网络环境做控制仿真。
# ----------------------------------------------------------------------
try:
    from pymodbus.datastore import (
        ModbusSequentialDataBlock,
        ModbusServerContext,
        ModbusSlaveContext,
    )
    from pymodbus.server import ModbusTcpServer

    PYMODBUS_AVAILABLE = True
except ImportError:  # pragma: no cover - 仅在未安装 pymodbus 时触发
    PYMODBUS_AVAILABLE = False


# ======================================================================
# 一、点表定义（BA 行业惯例：按 AI/AO/DI/DO 分组编号）
# ======================================================================

@dataclass
class Point:
    """单个点位定义。"""
    addr: str        # 点位地址，如 "AI1"
    ptype: str       # 点类型："AI"/"AO"/"DI"/"DO"
    desc: str        # 中文描述（工程师站上显示的点名）
    unit: str        # 工程单位
    scale: float     # 工程值 -> 寄存器值 换算系数（寄存器值 = round(工程值 × scale)）
    reg_addr: int    # Modbus 寄存器地址（16 进制见文件头注释）
    readonly: bool   # 对外部 Modbus 主站是否只读


# ---------------------------- AI 模拟量输入 ----------------------------
POINT_AI = [
    Point("AI1", "AI", "办公室室内温度", "℃", 10.0, 0x0000, True),
    Point("AI2", "AI", "会议室室内温度", "℃", 10.0, 0x0001, True),
    Point("AI3", "AI", "大堂室内温度",   "℃", 10.0, 0x0002, True),
    Point("AI4", "AI", "室外温度",       "℃", 10.0, 0x0003, True),
    Point("AI5", "AI", "生活水箱液位",   "m", 100.0, 0x0004, True),
    Point("AI6", "AI", "冷冻水供水温度", "℃", 10.0, 0x0005, True),
]
# ---------------------------- AO 模拟量输出 ----------------------------
POINT_AO = [
    Point("AO1", "AO", "办公室冷冻水阀开度",   "%",  10.0, 0x0000, False),
    Point("AO2", "AO", "会议室冷冻水阀开度",   "%",  10.0, 0x0001, False),
    Point("AO3", "AO", "大堂冷冻水阀开度",     "%",  10.0, 0x0002, False),
    Point("AO4", "AO", "送风机运行频率",       "Hz", 10.0, 0x0003, False),
]
# ---------------------------- DI 数字量输入 ----------------------------
POINT_DI = [
    # 行业惯例：防火阀正常时触点闭合=1；火灾时易熔环熔断→阀门关闭→触点断开=0
    Point("DI1", "DI", "防火阀关闭信号(0=已关闭联锁)", "-", 1.0, 0x0000, True),
    Point("DI2", "DI", "手/自动转换开关(1=自动)",      "-", 1.0, 0x0001, True),
    Point("DI3", "DI", "送风机故障反馈(1=无故障)",     "-", 1.0, 0x0002, True),
    Point("DI4", "DI", "水箱高液位浮球(1=未到高位)",   "-", 1.0, 0x0003, True),
]
# ---------------------------- DO 数字量输出 ----------------------------
POINT_DO = [
    Point("DO1", "DO", "新风阀开关(1=开)",         "-", 1.0, 0x0000, False),
    Point("DO2", "DO", "送风机启停(1=启动)",       "-", 1.0, 0x0001, False),
    Point("DO3", "DO", "冷冻水泵启停(1=启动)",     "-", 1.0, 0x0002, False),
    Point("DO4", "DO", "水箱进水阀开关(1=开)",     "-", 1.0, 0x0003, False),
    Point("DO5", "DO", "声光报警器(1=报警中)",     "-", 1.0, 0x0004, False),
    Point("DO6", "DO", "排风机启停(1=启动)",       "-", 1.0, 0x0005, False),
]

#: 完整点表 = AI + AO + DI + DO（供看板/文档生成用）
ALL_POINTS = POINT_AI + POINT_AO + POINT_DI + POINT_DO

#: 房间名称唯一来源（全项目唯一定义处，其余模块一律从此导入）：
#: 顺序与 AI1~AI3 / AO1~AO3 点位及 plant.thermal.ROOM_PARAMS 严格对应。
#: 新增房间时只需修改本列表、点表与 ROOM_PARAMS 三处同源定义。
ROOM_NAMES = ["办公室", "会议室", "大堂"]

# 寄存器哨兵值：传感器开路/故障时 AI 寄存器写 32767，读取方换算回 NaN
REG_SENSOR_FAULT = 32767


def print_point_table() -> None:
    """以表格形式打印完整点表（调试/文档用）。"""
    print("-" * 88)
    print(f"{'地址':<6}{'类型':<5}{'Modbus区':<12}{'寄存器':<10}{'中文描述':<26}{'单位':<6}{'读写'}")
    print("-" * 88)
    area_name = {"AI": "IR(fc04)", "AO": "HR(fc03/06)", "DI": "DI(fc02)", "DO": "CO(fc01/05)"}
    for p in ALL_POINTS:
        rw = "R(只读)" if p.readonly else "R/W"
        print(f"{p.addr:<6}{p.ptype:<5}{area_name[p.ptype]:<14}0x{p.reg_addr:04X}    "
              f"{p.desc:<24}{p.unit:<6}{rw}")
    print("-" * 88)


# ======================================================================
# 二、共享 I/O 映像区 PointBus（线程安全）
# ======================================================================

class PointBus:
    """
    共享 I/O 映像区：整个仿真系统的"数据总线"。

    - plant 写 AI（传感器测量值）、DI（设备反馈）；
    - ddc   读 AI/DI，写 AO（执行器指令）、DO（启停指令）；
    - Modbus 从站把本映像区暴露给外部主站；上位机的 SP 下发也写进本区，
      DDC 每个扫描周期都会读取，从而实现"设定值在线修改"。
    所有读写均加互斥锁，保证 Flask 多线程 / Modbus 线程 / 仿真线程并发安全。
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        # 工程值映像：addr -> float（AI/AO）；DI/DO 用 0.0/1.0 表示
        self._eng: dict[str, float] = {p.addr: 0.0 for p in ALL_POINTS}
        self._fault: dict[str, bool] = {p.addr: False for p in POINT_AI}  # AI 传感器故障标志
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


# ======================================================================
# 三、Modbus/TCP 从站（把 PointBus 暴露为标准 Modbus 寄存器）
# ======================================================================

class _BusInputBlock(ModbusSequentialDataBlock):
    """输入寄存器块(IR, fc04)：视图映射 PointBus 的 AI1~AI6。"""

    N = len(POINT_AI)  # 6

    def __init__(self, bus: PointBus) -> None:
        super().__init__(0x0000, [0] * self.N)
        self.bus = bus

    def validate(self, address: int, count: int = 1) -> bool:
        return 0 <= address and address + count <= self.N

    def getValues(self, address: int, count: int = 1):
        vals = []
        for i in range(address, address + count):
            eng = self.bus.read(f"AI{i + 1}")          # 工程值（故障时为 NaN）
            vals.append(_eng_to_reg(eng, POINT_AI[i]))
        return vals

    def setValues(self, address, values):  # AI 只读，忽略外部写
        pass


class _BusHoldBlock(ModbusSequentialDataBlock):
    """
    保持寄存器块(HR, fc03读/fc06、fc16写)：
      0x0000~0x0003 → AO1~AO4（DDC 每周期刷新；外部写入可被看作强制值，
                        自动模式下会被 DDC 下一周期的计算值覆盖）
      0x0010~0x0015 → 上位机接口区（手动 SP / 使能位 / 模式字 / 系统使能）
    """

    N = 0x0016  # 22 个保持寄存器

    def __init__(self, bus: PointBus) -> None:
        super().__init__(0x0000, [0] * self.N)
        self.bus = bus

    def validate(self, address: int, count: int = 1) -> bool:
        return 0 <= address and address + count <= self.N

    def getValues(self, address: int, count: int = 1):
        out = []
        for a in range(address, address + count):
            if a <= 0x0003:                       # AO 区
                out.append(_eng_to_reg(self.bus.read(f"AO{a + 1}"), POINT_AO[a]))
            elif 0x0010 <= a <= 0x0012:           # 手动 SP 区
                out.append(int(round(self.bus.get_manual_sp(a - 0x0010) * 10)))
            elif a == 0x0013:
                out.append(self.bus.get_sp_manual_bits())
            elif a == 0x0014:
                out.append(self.bus.get_mode_bits())
            elif a == 0x0015:
                out.append(1 if self.bus.sys_enable_req else 0)
            else:
                out.append(0)
        return out

    def setValues(self, address, values):
        for i, v in enumerate(values):
            a = address + i
            try:
                v = int(v)
            except (TypeError, ValueError):
                continue
            if 0x0010 <= a <= 0x0012:             # 上位机改设定温度
                self.bus.set_manual_sp(a - 0x0010, v / 10.0)
            elif a == 0x0013:
                self.bus.set_sp_manual_bits(v)
            elif a == 0x0014:
                self.bus.set_mode_bits(v)
            elif a == 0x0015:
                self.bus.set_sys_enable_req(bool(v))
            elif a <= 0x0003:                     # 外部强制 AO（演示用）
                self.bus.write(f"AO{a + 1}", v / 10.0)


class _BusDiscreteBlock(ModbusSequentialDataBlock):
    """离散输入块(DI, fc02)：视图映射 PointBus 的 DI1~DI4。"""

    N = len(POINT_DI)  # 4

    def __init__(self, bus: PointBus) -> None:
        super().__init__(0x0000, [0] * self.N)
        self.bus = bus

    def validate(self, address: int, count: int = 1) -> bool:
        return 0 <= address and address + count <= self.N

    def getValues(self, address: int, count: int = 1):
        return [int(self.bus.read(f"DI{i + 1}")) for i in range(address, address + count)]

    def setValues(self, address, values):  # DI 只读
        pass


class _BusCoilBlock(ModbusSequentialDataBlock):
    """线圈块(CO, fc01读/fc05写)：视图映射 PointBus 的 DO1~DO6。"""

    N = len(POINT_DO)  # 6

    def __init__(self, bus: PointBus) -> None:
        super().__init__(0x0000, [0] * self.N)
        self.bus = bus

    def validate(self, address: int, count: int = 1) -> bool:
        return 0 <= address and address + count <= self.N

    def getValues(self, address: int, count: int = 1):
        return [int(self.bus.read(f"DO{i + 1}")) for i in range(address, address + count)]

    def setValues(self, address, values):
        # 外部主站可以写线圈（相当于上位机远程启停），真实 DDC 中即遥控功能
        for i, v in enumerate(values):
            a = address + i
            if 0 <= a < self.N:
                self.bus.write(f"DO{a + 1}", 1.0 if int(v) else 0.0)


def _eng_to_reg(eng: float, point: Point) -> int:
    """工程值 → 16 位寄存器值（NaN/超限 → 哨兵值 32767）。"""
    if eng is None or (isinstance(eng, float) and math.isnan(eng)):
        return REG_SENSOR_FAULT
    return int(round(eng * point.scale))


class ModbusSlaveServer:
    """
    Modbus/TCP 从站服务器（独立后台线程运行 asyncio 事件循环）。

    用法：
        bus = PointBus()
        slave = ModbusSlaveServer(bus, host="127.0.0.1", port=5020)
        slave.start()      # 非阻塞启动
        ...
        slave.stop()       # 优雅停止
    """

    def __init__(self, bus: PointBus, host: str = "0.0.0.0", port: int = 5020) -> None:
        if not PYMODBUS_AVAILABLE:
            raise RuntimeError("未安装 pymodbus，请先执行: pip install pymodbus")
        self.bus = bus
        self.host = host
        self.port = port
        slave_ctx = ModbusSlaveContext(
            di=_BusDiscreteBlock(bus),
            co=_BusCoilBlock(bus),
            hr=_BusHoldBlock(bus),
            ir=_BusInputBlock(bus),
            zero_mode=True,   # 地址不偏移：客户端地址 == 点表寄存器地址
        )
        self.context = ModbusServerContext(slaves=slave_ctx, single=True)
        self._server = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._started = threading.Event()

    def start(self) -> None:
        """在后台线程中启动 Modbus/TCP 服务。"""
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, name="ModbusSlave", daemon=True)
        self._thread.start()
        self._started.wait(timeout=5)

    def _run(self) -> None:
        async def _serve():
            self._server = ModbusTcpServer(context=self.context,
                                           address=(self.host, self.port))
            self._started.set()
            await self._server.serve_forever()

        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(_serve())
        except Exception as exc:  # pragma: no cover
            print(f"[Modbus从站] 运行异常: {exc!r}")
        finally:
            self._loop.close()

    def stop(self) -> None:
        """请求从站优雅退出并回收线程。"""
        if self._server is not None and self._loop is not None:
            asyncio.run_coroutine_threadsafe(self._server.shutdown(), self._loop)
        if self._thread:
            self._thread.join(timeout=5)
        self._server = None
        self._loop = None
        self._thread = None


# ======================================================================
# 四、独立运行入口：python -m points.point_table
# ======================================================================

def _demo_main() -> None:
    """启动一个带演示数据的 Modbus 从站，供 modbus_client_test.py 联调。"""
    import random
    import sys
    import time as _time

    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print_point_table()
    bus = PointBus()
    slave = ModbusSlaveServer(bus, host="0.0.0.0", port=5020)
    slave.start()
    print(f"[Modbus从站] 已启动 tcp://{slave.host}:{slave.port} (Slave ID=1)，Ctrl+C 退出")
    rng = random.Random(2024)
    t0 = _time.time()
    try:
        while True:  # 演示数据源：缓慢变化的假温度
            elapsed = (_time.time() - t0) / 60.0
            bus.write("AI1", 24.0 + 0.8 * math.sin(elapsed / 30) + rng.uniform(-0.1, 0.1))
            bus.write("AI2", 24.3 + 0.6 * math.sin(elapsed / 25 + 1) + rng.uniform(-0.1, 0.1))
            bus.write("AI3", 25.0 + 0.5 * math.sin(elapsed / 40 + 2) + rng.uniform(-0.1, 0.1))
            bus.write("AI5", 1.2 + 0.3 * math.sin(elapsed / 20))
            bus.write("AO1", 35.0)
            bus.write("DO2", 1.0)
            _time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        slave.stop()
        print("[Modbus从站] 已停止")


if __name__ == "__main__":
    _demo_main()
