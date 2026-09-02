# -*- coding: utf-8 -*-
"""
points/point_defs.py —— BA 点表定义（唯一来源）
================================================

按楼宇自控行业惯例，以"点类型 + 序号"为现场点位编址：
  - AI  (Analog Input,  模拟量输入)：温度、液位等传感器信号；
  - AO  (Analog Output, 模拟量输出)：电动阀开度、风机频率等执行器指令；
  - DI  (Digital Input, 数字量输入)：防火阀反馈、手/自动模式反馈、故障反馈；
  - DO  (Digital Output,数字量输出)：风机/水泵启停、电动阀开关。

本模块只描述"有哪些点、什么含义、怎么换算"；工程值如何存取见 point_bus.py，
如何暴露为 Modbus 寄存器见 modbus_slave.py。
"""

import math
from dataclasses import dataclass


@dataclass
class Point:
    """单个点位定义。"""
    addr: str        # 点位地址，如 "AI1"
    ptype: str       # 点类型："AI"/"AO"/"DI"/"DO"
    desc: str        # 中文描述（工程师站上显示的点名）
    unit: str        # 工程单位
    scale: float     # 工程值 -> 寄存器值 换算系数（寄存器值 = round(工程值 × scale)）
    reg_addr: int    # Modbus 寄存器地址（16 进制见 modbus_slave.py 文件头注释）
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
    # 手/自动的实际判定来源是上位机模式字 HR20.bit1(经 PointBus)；
    # DI2 作为模式反馈点随模式字同步显示（同步实现在 PointBus.set_mode_bits，
    # 1=自动/0=手动），供看板/主站监视，不作为控制输入
    Point("DI2", "DI", "手/自动模式反馈(1=自动)",      "-", 1.0, 0x0001, True),
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

# 寄存器哨兵值：传感器开路/故障时 AI 寄存器写 32767，读取方据此识别坏值
REG_SENSOR_FAULT = 32767


def eng_to_reg(eng: float, point: Point) -> int:
    """工程值 → 16 位寄存器值（NaN/超限 → 哨兵值 32767）。"""
    if eng is None or (isinstance(eng, float) and math.isnan(eng)):
        return REG_SENSOR_FAULT
    return int(round(eng * point.scale))


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
