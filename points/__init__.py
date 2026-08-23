# -*- coding: utf-8 -*-
"""
points 包：点表与通信模块。

包含：
- point_table.py        BA 点表定义(AI/AO/DI/DO) + 共享 I/O 映像区(PointBus)
                        + Modbus/TCP 从站（把点表暴露为保持寄存器等）
- modbus_client_test.py Modbus/TCP 主站自测脚本（模拟上位机读写点表）
"""
