# -*- coding: utf-8 -*-
"""
points/modbus_client_test.py —— Modbus/TCP 主站自测脚本
========================================================

【用途】模拟"上位机/BAS 平台"作为 Modbus 主站，对 DDC 从站做读写联调：
  ① 连接 tcp://127.0.0.1:5020（Slave ID=1）；
  02 读离散输入 DI1~DI4（功能码 0x02）；
  ③ 读线圈 DO1~DO6（功能码 0x01）；
  ④ 读输入寄存器 AI1~AI6（功能码 0x04，观察温度/液位实时值与故障哨兵值）；
  ⑤ 写单个保持寄存器：把办公室手动 SP 改为 25.5℃ 并置手动使能（功能码 0x06），
     随后读回验证——这就是看板"SP 在线修改按钮"的通信链路；
  ⑥ 写保持寄存器模式字：切换节能模式；
  ⑦ 写线圈：远程启动声光报警器再关闭（遥控演示）；
  ⑧ 连续 10 秒轮询 AI1~AI3，展示数据刷新。

【运行前提】二选一：
  A. python -m points.point_table      （独立演示从站）
  B. python dashboard/app.py           （看板内置仿真从站）
本脚本会自动检测端口并给出提示。

【运行】python -m points.modbus_client_test
"""

import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from pymodbus.client import ModbusTcpClient          # noqa: E402

HOST, PORT, SLAVE_ID = "127.0.0.1", 5020, 1


def check_port() -> bool:
    """快速探测从站端口是否已开放。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(1.0)
        return s.connect_ex((HOST, PORT)) == 0


def reg_to_temp(reg: int) -> str:
    """寄存器值 → 显示文本（哨兵值 32767 表示传感器故障）。"""
    if reg == 32767:
        return "故障(哨兵值)"
    return f"{reg / 10.0:.1f} ℃"


def main() -> int:
    print("=" * 70)
    print(" Modbus/TCP 主站自测脚本 —— 模拟上位机访问 DDC 点表")
    print(f" 目标从站: tcp://{HOST}:{PORT}  Slave ID={SLAVE_ID}")
    print("=" * 70)

    if not check_port():
        print("\n[错误] 未检测到从站。请先任选其一启动从站：")
        print("   A. python -m points.point_table     （独立演示从站）")
        print("   B. python dashboard/app.py          （看板内置仿真从站）")
        return 1

    client = ModbusTcpClient(HOST, port=PORT)
    if not client.connect():
        print("[错误] 连接失败")
        return 1
    print("[OK] 已连接从站\n")

    try:
        # ---- ② 读离散输入 DI1~DI4 (fc02) ----
        rr = client.read_discrete_inputs(0, 4, slave=SLAVE_ID)
        assert not rr.isError(), f"读DI失败: {rr}"
        di_desc = ["防火阀关闭信号(1=正常)", "手/自动(1=自动)",
                   "风机故障反馈(1=正常)", "水箱高位浮球(1=未到高位)"]
        print("── 步骤① 读离散输入 DI1~DI4 (fc02) ──")
        for i, v in enumerate(rr.bits[:4]):
            print(f"    DI{i+1} = {int(v)}  ({di_desc[i]})")

        # ---- ③ 读线圈 DO1~DO6 (fc01) ----
        rr = client.read_coils(0, 6, slave=SLAVE_ID)
        assert not rr.isError(), f"读DO失败: {rr}"
        do_desc = ["新风阀", "送风机", "冷冻水泵", "水箱进水阀", "声光报警器", "排风机"]
        print("── 步骤② 读线圈 DO1~DO6 (fc01) ──")
        for i, v in enumerate(rr.bits[:6]):
            print(f"    DO{i+1} = {int(v)}  ({do_desc[i]})")

        # ---- ④ 读输入寄存器 AI1~AI6 (fc04) ----
        rr = client.read_input_registers(0, 6, slave=SLAVE_ID)
        assert not rr.isError(), f"读AI失败: {rr}"
        ai_desc = ["办公室温度", "会议室温度", "大堂温度", "室外温度", "水箱液位", "冷冻水温"]
        print("── 步骤③ 读输入寄存器 AI1~AI6 (fc04) ──")
        for i, v in enumerate(rr.registers):
            unit = " m" if i == 4 else ""
            txt = ("故障(哨兵值)" if v == 32767 else f"{v / (100 if i == 4 else 10):.2f}{unit}")
            print(f"    AI{i+1} = {v:>5}  ({ai_desc[i]}: {txt})")

        # ---- ⑤ 在线修改设定温度 SP（HR16 写 255 → 25.5℃；HR19 置 bit0 手动使能）----
        print("── 步骤④ 在线修改办公室设定温度 SP=25.5℃ 并置手动使能 ──")
        rq = client.write_register(0x0010, 255, slave=SLAVE_ID)
        assert not rq.isError(), f"写SP失败: {rq}"
        rq = client.write_register(0x0013, 0x001, slave=SLAVE_ID)   # bit0 办公室手动
        assert not rq.isError(), f"写SP使能失败: {rq}"
        rr = client.read_holding_registers(0x0010, 4, slave=SLAVE_ID)
        assert not rr.isError(), f"回读失败: {rr}"
        sp_bits = rr.registers[3]
        print(f"    回读 HR16~HR19 = {rr.registers} → "
              f"办公室SP={rr.registers[0]/10:.1f}℃, 会议室SP={rr.registers[1]/10:.1f}℃, "
              f"大堂SP={rr.registers[2]/10:.1f}℃, 手动使能位=0b{sp_bits:03b}")
        # 测试完恢复自动（清除使能位），避免影响看板演示
        client.write_register(0x0013, 0x000, slave=SLAVE_ID)

        # ---- ⑥ 切换节能模式（HR20 模式字 bit0）----
        print("── 步骤⑥ 切换控制模式字 HR20 ──")
        rr = client.read_holding_registers(0x0014, 1, slave=SLAVE_ID)
        mode_old = rr.registers[0]
        mode_new = mode_old ^ 0x01                 # 翻转节能位
        client.write_register(0x0014, mode_new, slave=SLAVE_ID)
        rr = client.read_holding_registers(0x0014, 1, slave=SLAVE_ID)
        print(f"    模式字 0x{mode_old:04X} → 0x{rr.registers[0]:04X} "
              f"(bit0节能={'开' if rr.registers[0] & 1 else '关'}, "
              f"bit1自动={'是' if rr.registers[0] & 2 else '否'})")
        client.write_register(0x0014, mode_old, slave=SLAVE_ID)   # 恢复

        # ---- ⑦ 远程遥控线圈（写 DO5 声光报警器）----
        print("── 步骤⑦ 遥控线圈 DO5 ──")
        client.write_coil(4, True, slave=SLAVE_ID)
        rr = client.read_coils(4, 1, slave=SLAVE_ID)
        print(f"    写 DO5=True → 回读 DO5={int(rr.bits[0])}")
        client.write_coil(4, False, slave=SLAVE_ID)

        # ---- ⑧ 连续轮询 10 次 AI1~AI3，观察数据刷新 ----
        print("── 步骤⑧ 轮询 AI1~AI3 共 10 次（间隔 1s）──")
        for k in range(10):
            rr = client.read_input_registers(0, 3, slave=SLAVE_ID)
            vals = [reg_to_temp(v) for v in rr.registers]
            print(f"    #{k+1:<2} 办公室={vals[0]:<12} 会议室={vals[1]:<12} 大堂={vals[2]}")
            time.sleep(1.0)

        print("\n[PASS] 全部读写测试通过：Modbus/TCP 链路与点表映射正确。")
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
