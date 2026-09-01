# -*- coding: utf-8 -*-
"""
points/modbus_slave.py —— Modbus/TCP 从站（把 PointBus 暴露为标准寄存器）
==========================================================================

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

数据块类都是 PointBus 的"视图"：读时实时换算工程值，写时回调写回总线，
本模块不保存任何第二份数据，避免双源不一致。

独立运行入口：`python -m points.modbus_slave` 启动带演示数据的从站。
"""

import asyncio
import math
import socket
import threading

# pymodbus 延迟导入说明：PointBus 是纯 Python 实现不依赖 pymodbus；
# 只有真正启动 Modbus 从站时才需要 pymodbus，便于无网络环境做控制仿真。
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

from points.point_bus import PointBus
from points.point_defs import (POINT_AI, POINT_AO, POINT_DI, POINT_DO,
                               REG_SENSOR_FAULT, eng_to_reg, print_point_table)


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
            vals.append(eng_to_reg(eng, POINT_AI[i]))
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
                out.append(eng_to_reg(self.bus.read(f"AO{a + 1}"), POINT_AO[a]))
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

    def __init__(self, bus: PointBus, host: str = "127.0.0.1", port: int = 5020) -> None:
        # 默认只绑定本机回环地址：Modbus 协议无鉴权且线圈可写(远程启停设备)，
        # 监听 0.0.0.0 会把控制权暴露给整个局域网；需要跨机联调时显式传参覆盖。
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
        """在后台线程中启动 Modbus/TCP 服务。

        启动失败必须显式暴露（不可静默）：端口被占用等错误原先只在线程内
        打印一行，start() 照常返回，调用方误以为从站已监听、联调失败无从
        排查。这里启动前先试绑定端口提前暴露占用，并检查后台线程是否在
        超时内完成初始化，失败一律抛出异常。
        """
        if self._thread and self._thread.is_alive():
            return
        # ---- 端口预检：提前暴露"端口被占用/地址无效"并给出可读错误 ----
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            probe.bind((self.host, self.port))
        except OSError as exc:
            raise RuntimeError(
                f"Modbus/TCP 从站无法绑定 {self.host}:{self.port}"
                f"（端口被占用或地址无效）: {exc}") from exc
        finally:
            probe.close()
        self._thread = threading.Thread(target=self._run, name="ModbusSlave", daemon=True)
        self._thread.start()
        if not self._started.wait(timeout=5):
            raise RuntimeError(f"Modbus/TCP 从站启动超时({self.host}:{self.port})，"
                               f"请查看后台线程的错误输出")

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


def _demo_main() -> None:
    """启动一个带演示数据的 Modbus 从站，供 modbus_client_test.py 联调。"""
    import random
    import sys
    import time as _time

    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print_point_table()
    bus = PointBus()
    slave = ModbusSlaveServer(bus, port=5020)   # 默认绑定 127.0.0.1(本机联调)
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
