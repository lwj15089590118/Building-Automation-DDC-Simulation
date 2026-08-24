# -*- coding: utf-8 -*-
"""
points 包：点表与通信模块。

包含：
- point_defs.py    BA 点表定义(AI/AO/DI/DO 中文描述、房间名常量、寄存器换算)
- point_bus.py     共享 I/O 映像区 PointBus（plant/ddc/dashboard 的数据总线）
- modbus_slave.py  Modbus/TCP 从站（把点表暴露为 IR/HR/DI/CO 四个寄存器区；
                   独立运行 `python -m points.modbus_slave` 可启动演示从站）
- modbus_client_test.py  Modbus/TCP 主站自测脚本（模拟上位机读写点表）
"""
