# -*- coding: utf-8 -*-
"""
plant/thermal.py —— 房间 RC 热网络模型（受控对象·核心之一）
============================================================

【物理原理：1R1C 热网络模型】
把每个房间等效为一个集中热容节点 C（空气+家具+围护结构的等效热容），
通过等效热阻 R 与室外环境相连，形成最简单的一阶 RC 热网络：

            Q_solar(日照得热)
               ↓
   T_out ○──R───────┬────→ 室温 T (节点温度)
   室外温度  │       │
        (UA导热)   C 等效热容   ← Q_people(人员) + Q_equip(设备)
                 │       │
                 └───────┘   ← Q_cool(空调盘管冷量, 由阀开度 OP 决定)

热平衡微分方程：
    C · dT/dt = (T_out − T)/R + Q_people + Q_equip + Q_solar − Q_cool
    其中 1/R = UA（围护结构综合传热系数，单位 W/K）

离散步长 dt=60s，采用显式欧拉法积分：
    T(k+1) = T(k) + (dt/C) · [ ΣQ ]

【负荷来源】
  1) 室外温度日曲线：正弦基线(谷值约凌晨5点、峰值约14点) + 缓变随机扰动；
  2) 人员热负荷：按作息时间表(上班/会议/午休)给出人数 → 显热 120W/人；
  3) 设备热负荷：电脑、投影、大堂显示设备等，按时间表启停；
  4) 日照得热：钟形太阳辐射曲线 × 云量系数 × 窗墙面积；
  5) 空调冷量：盘管制冷量 = 阀开度 OP% × 最大冷量 × (风机频率/50Hz)，
     风机未运行时盘管无冷量输出（与联锁逻辑呼应）。

【测量环节】真实温度上叠加高斯白噪声(N(0,0.15℃))模拟温度传感器，
并支持注入"传感器故障"(开路NaN/卡死/偏移)，供 DDC 的报警逻辑验证。

本模块只负责"物理演化"，不包含任何控制逻辑；执行器指令(AO/DO)
从 PointBus 读入，测量值写回 PointBus。
"""

import math
import random
from dataclasses import dataclass

from points.point_table import PointBus, ROOM_NAMES


# ======================================================================
# 一、房间参数（3 个典型功能房间）
# ======================================================================

class RoomParams:
    """单个房间的物理参数（数值参考暖通设计手册的典型量级）。"""

    def __init__(self, name: str, area_m2: float,
                 capacitance: float, conductance: float,
                 people_max: int, q_equip_max: float,
                 q_solar_max: float, q_cool_max: float) -> None:
        self.name = name              # 房间名称
        self.area_m2 = area_m2        # 建筑面积 m²
        self.capacitance = capacitance    # 等效热容 C (J/K)：决定温度变化快慢
        self.conductance = conductance    # 围护结构导热 UA (W/K)：1/R
        self.people_max = people_max      # 设计最多人数（每人显热按 120W 计）
        self.q_equip_max = q_equip_max    # 设备最大散热量 W
        self.q_solar_max = q_solar_max    # 正午最大日照得热 W
        self.q_cool_max = q_cool_max      # 盘管最大制冷量 W（阀开度100%、工频时）


#: 三个房间的参数表（C、UA 取值使时间常数 τ=C/UA 约 3~5 小时，接近真实房间）
#: 房间名称取自点表的 ROOM_NAMES（唯一来源），顺序与 AI1~AI3/AO1~AO3 对应
ROOM_PARAMS: list[RoomParams] = [
    RoomParams(ROOM_NAMES[0], area_m2=60.0, capacitance=2.5e6, conductance=160.0,
               people_max=8, q_equip_max=900.0, q_solar_max=2200.0, q_cool_max=6000.0),
    RoomParams(ROOM_NAMES[1], area_m2=40.0, capacitance=2.0e6, conductance=130.0,
               people_max=10, q_equip_max=350.0, q_solar_max=1500.0, q_cool_max=5000.0),
    RoomParams(ROOM_NAMES[2], area_m2=120.0, capacitance=4.0e6, conductance=280.0,
               people_max=10, q_equip_max=500.0, q_solar_max=4200.0, q_cool_max=12000.0),
]

#: 每人显热散热量 W（轻度办公活动，国标 GB50736 参考量级）
HEAT_PER_PERSON_W = 120.0


# ======================================================================
# 二、室外气象：温度日曲线 + 云量 + 日照得热
# ======================================================================

class OutdoorWeather:
    """
    室外气象模型（每天自动生成新的随机种子扰动）。

    - 温度日曲线：T_out(t) = T_base + A·sin(2π(t−t_peak+6h)/24h)
      相位选取使谷值约 05:00、峰值约 14:00；
      再叠加一阶惯性平滑的随机游走(幅度±0.8℃)，模拟天气波动。
    - 云量系数 cloud∈[0.75,1.10]：每天抽取一次，全天不变，影响日照得热。
    """

    def __init__(self, rng: random.Random) -> None:
        self.rng = rng
        self.t_base = rng.uniform(25.5, 27.5)   # 全天平均气温 ℃
        self.t_amp = rng.uniform(5.0, 6.5)      # 日较差幅值 ℃
        self.cloud = rng.uniform(0.75, 1.10)    # 云量系数(-)
        self._walk = 0.0                        # 当前随机游走偏移量
        self._last_min = -1                     # 上次更新时刻(防重复推进)

    def _advance_walk(self, minute: int, dt_min: float) -> None:
        """随机游走：每小时有概率发生小幅突变，用一阶惯性平滑。"""
        if minute <= self._last_min:
            return
        self._last_min = minute
        if self.rng.random() < dt_min / 90.0:          # 平均约 1.5 小时一次波动
            self._walk += self.rng.uniform(-0.45, 0.45)
            self._walk = max(-0.8, min(0.8, self._walk))

    def temperature(self, minute_of_day: float, dt_min: float = 1.0) -> float:
        """室外温度 ℃，minute_of_day 为当天第几分钟（可为小数）。"""
        self._advance_walk(int(minute_of_day), dt_min)
        hour = minute_of_day / 60.0
        # 正弦相位：t=14 时取最大值 → (hour-14)/24*2π = π/2 → 平移 8 小时
        base = self.t_base + self.t_amp * math.sin(2.0 * math.pi * (hour - 8.0) / 24.0)
        return base + self._walk

    def solar_gain(self, minute_of_day: float) -> float:
        """当前时刻单位日照强度系数(0~1]，钟形曲线：06:00~18:00 有日照，13:00 最强。"""
        hour = minute_of_day / 60.0
        if not (6.0 <= hour <= 18.0):
            return 0.0
        # 以 13 点为中心的高斯钟形，宽度约 ±3.5h
        bell = math.exp(-((hour - 13.0) / 3.5) ** 2)
        return bell * self.cloud


# ======================================================================
# 三、作息时间表（人员/设备负荷计划，表驱动）
# ======================================================================

def _lobby_people(h: float) -> int:
    """大堂人流曲线：07:00~20:00 呈单峰分布，峰值约 10 人。"""
    return round(6 + 4 * math.sin((h - 7.0) / 13.0 * math.pi))


@dataclass
class RoomScheduleSpec:
    """
    单个房间的作息计划规格（表驱动配置，替代按房间编号的 if 级联）。

    people_periods : [(起始小时, 结束小时, 人数 | 以小时为参的 callable), ...]
                     区间为左闭右开，未命中任何区间时人数为 0；
    equip_periods  : [(起始小时, 结束小时, 功率系数), ...]，同样左闭右开；
    equip_idle_k   : 未命中任何设备区间时的待机功率系数。
    """
    name: str                                   # 房间名(与 ROOM_NAMES 对应，便于阅读)
    people_periods: list[tuple]
    equip_periods: list[tuple]
    equip_idle_k: float

    @staticmethod
    def _hit(minute: int, start_h: float, end_h: float) -> bool:
        """判断当前分钟是否落入 [start_h, end_h) 小时区间。"""
        return start_h * 60 <= minute < end_h * 60

    def people_at(self, minute: int) -> int:
        """查询当前计划人数（不含随机抖动）。"""
        h = minute / 60.0
        for start_h, end_h, value in self.people_periods:
            if self._hit(minute, start_h, end_h):
                # 第三元素可为整数，也可为"小时→人数"的曲线函数(如大堂人流)
                return value(h) if callable(value) else value
        return 0

    def equip_at(self, minute: int) -> float:
        """查询当前设备功率系数(0~1)。"""
        for start_h, end_h, k in self.equip_periods:
            if self._hit(minute, start_h, end_h):
                return k
        return self.equip_idle_k


#: 三房间的作息规格表：新增/修改房间只改这一张表
ROOM_SCHEDULES: list[RoomScheduleSpec] = [
    RoomScheduleSpec(
        name=ROOM_NAMES[0],                    # 办公室：08:30~18:00 上班，
                                               # 12:00~13:30 午休留值守人员
        people_periods=[(8.5, 12.0, 8), (12.0, 13.5, 2), (13.5, 18.0, 8)],
        equip_periods=[(8.0, 18.5, 1.0)],      # 设备白天全开，夜间仅服务器约 10%
        equip_idle_k=0.1,
    ),
    RoomScheduleSpec(
        name=ROOM_NAMES[1],                    # 会议室：上午十人例会+下午六人会
        people_periods=[(9.5, 11.5, 10), (14.0, 16.0, 6)],
        equip_periods=[(9.5, 11.5, 1.0), (14.0, 16.0, 1.0)],  # 有会才开投影等
        equip_idle_k=0.05,
    ),
    RoomScheduleSpec(
        name=ROOM_NAMES[2],                    # 大堂：营业时段人流量呈单峰曲线
        people_periods=[(7.0, 20.0, _lobby_people)],
        equip_periods=[(7.0, 20.0, 1.0)],      # 营业时段显示屏/照明全开，
        equip_idle_k=0.15,                      # 夜间保留安防最低负荷
    ),
]


class OccupancySchedule:
    """
    作息时间表：返回某时刻各房间的人数与设备功率系数。
    具体计划集中在 ROOM_SCHEDULES 规格表中，本类只负责查表 + 到岗率抖动。

    时间表是 BA 系统"时间表控制(Schedule)"的物理侧对应物——
    DDC 按时间表调设定值，而负荷本身也按作息出现，两者共同决定能耗。
    """

    def __init__(self, rng: random.Random) -> None:
        self.rng = rng

    def people_count(self, room_idx: int, minute: int) -> int:
        """各房间当前人数（含到岗率的随机抖动，更贴近实际）。"""
        n = ROOM_SCHEDULES[room_idx].people_at(minute)
        # 到岗率抖动：±1 人，且不低于 0、不超过设计人数
        jitter = self.rng.choice((-1, 0, 0, 1))
        return max(0, min(n + jitter, ROOM_PARAMS[room_idx].people_max))

    def equip_power(self, room_idx: int, minute: int) -> float:
        """各房间当前设备功率系数(0~1)。"""
        return ROOM_SCHEDULES[room_idx].equip_at(minute)


# ======================================================================
# 四、单房间 RC 模型
# ======================================================================

class RoomThermalModel:
    """单个房间的 1R1C 热网络模型（状态：真实室温 self.temp）。"""

    def __init__(self, idx: int, params: RoomParams,
                 weather: OutdoorWeather, schedule: OccupancySchedule,
                 rng: random.Random) -> None:
        self.idx = idx
        self.params = params
        self.weather = weather
        self.schedule = schedule
        self.rng = rng
        self.temp = 26.5 + rng.uniform(-0.5, 0.5)   # 初始真实室温 ℃
        # ---- 负荷记录（供看板/日报展示）----
        self.q_people = 0.0
        self.q_equip = 0.0
        self.q_solar = 0.0
        self.q_envelope = 0.0
        self.q_cool = 0.0
        # ---- 传感器故障注入 ----
        self.fault_mode: str | None = None   # None / "nan" / "stuck" / "offset"
        self.fault_until = -1                # 故障持续到的时刻(绝对分钟)

    # ---------- 负荷计算 ----------
    def update_loads(self, minute_of_day: int) -> None:
        """按时间表刷新人员/设备/日照/围护结构热负荷(W)。"""
        p = self.params
        n = self.schedule.people_count(self.idx, minute_of_day)
        self.q_people = n * HEAT_PER_PERSON_W
        self.q_equip = p.q_equip_max * self.schedule.equip_power(self.idx, minute_of_day)
        self.q_solar = p.q_solar_max * self.weather.solar_gain(minute_of_day)
        t_out = self.weather.temperature(minute_of_day)
        self.q_envelope = (t_out - self.temp) * p.conductance   # 通过围护结构的得热(可为负)

    # ---------- 物理积分 ----------
    def step(self, minute_of_day: int, cool_op: float, fan_ratio: float,
             dt_min: float = 1.0) -> None:
        """
        推进一个控制周期。
        :param cool_op:   冷冻水阀开度 0~1（来自 AO）
        :param fan_ratio: 风机频率比 freq/50Hz（风机停止时为 0，此时盘管无冷量）
        :param dt_min:    步长（分钟），默认 1 分钟
        """
        p = self.params
        self.update_loads(minute_of_day)
        # 盘管冷量：开度 × 最大冷量 × 频率比（风机停 → 无冷量输出）
        self.q_cool = max(0.0, min(1.0, cool_op)) * p.q_cool_max * max(0.0, min(1.0, fan_ratio))
        net_q = self.q_envelope + self.q_people + self.q_equip + self.q_solar - self.q_cool
        d_temp = net_q * (dt_min * 60.0) / p.capacitance      # ΔT = ΣQ·Δt / C
        self.temp += d_temp

    # ---------- 测量环节 ----------
    def measure(self, now_min_abs: int) -> float:
        """
        返回传感器测量值 PV（真实温度 + 高斯噪声，或故障注入值）。
        :param now_min_abs: 绝对仿真分钟数（从仿真 0 时起算）
        """
        if self.fault_mode is not None and now_min_abs <= self.fault_until:
            if self.fault_mode == "nan":       # 开路故障：信号中断
                return float("nan")
            if self.fault_mode == "offset":    # 漂移故障：读数整体偏高
                return self.temp + 9.0 + self.rng.gauss(0, 0.1)
            # "stuck" 卡死故障：冻结在进入故障时的读数（此处简化为固定 24.0）
            return 24.0 + self.rng.gauss(0, 0.01)
        return self.temp + self.rng.gauss(0, 0.15)   # 正常测量噪声 N(0, 0.15℃)


# ======================================================================
# 五、整栋楼宇对象：3 房间 + 水箱，对接 PointBus
# ======================================================================

class BuildingPlant:
    """
    受控对象总成：
      - 三个独立 RC 房间模型；
      - 一台室外气象模型；
      - 一座生活水箱（见 water_tank.py）。

    对外接口：
      step(minute_of_day, abs_minute, dt_min)  推进一个周期（先读指令，再演化物理，后写测量值）
      inject_sensor_fault(room_idx, start, dur, mode)  注入传感器故障
    """

    #: 房间序号 ↔ AI/AO 点位对照
    ROOM_AI = ["AI1", "AI2", "AI3"]
    ROOM_AO = ["AO1", "AO2", "AO3"]

    def __init__(self, bus: PointBus, seed: int | None = 2024) -> None:
        self.bus = bus
        self.rng = random.Random(seed)
        self.weather = OutdoorWeather(self.rng)
        self.schedule = OccupancySchedule(self.rng)
        from plant.water_tank import WaterTank   # 局部导入避免循环依赖
        self.tank = WaterTank()
        self.rooms = [RoomThermalModel(i, p, self.weather, self.schedule, self.rng)
                      for i, p in enumerate(ROOM_PARAMS)]
        self.abs_minute = 0                       # 绝对仿真时间（分钟）

    # ---------------- 执行器指令读取 ----------------
    def _read_fan_ratio(self) -> float:
        """送风机频率比 freq/50（DO2 未启动或频率为 0 时视为停机）。"""
        fan_on = self.bus.read_bool("DO2")
        freq_hz = self.bus.read("AO4")
        if not fan_on or freq_hz <= 0.5:
            return 0.0
        return min(1.0, freq_hz / 50.0)

    # ---------------- 传感器故障注入 ----------------
    def inject_sensor_fault(self, room_idx: int, start_abs_min: int,
                            duration_min: int, mode: str = "nan") -> None:
        """
        在指定时刻对指定房间注入传感器故障（用于验证 DDC 报警逻辑）。
        :param mode: "nan"(开路) / "stuck"(卡死) / "offset"(漂移偏高)
        """
        room = self.rooms[room_idx]
        room.fault_mode = mode
        room.fault_until = start_abs_min + duration_min

    # ---------------- 主推进函数 ----------------
    def step(self, minute_of_day: int, dt_min: float = 1.0) -> dict:
        """
        推进一个控制周期（默认 1 分钟）：
          ① 从 PointBus 读取 DDC 输出的执行器指令(AO 阀开度/频率、DO 启停)；
          ② 各房间 RC 模型积分一步、水箱液位积分一步；
          ③ 把测量值(含噪声/故障)写回 PointBus 的 AI 区、反馈写 DI 区。
        :return: 本周期的工况快照（供记录器使用）
        """
        self.abs_minute += 1
        fan_ratio = self._read_fan_ratio()
        ops = []
        for i, room in enumerate(self.rooms):
            op = self.bus.read(self.ROOM_AO[i]) / 100.0     # AO 工程值为 %
            ops.append(op)
            room.step(minute_of_day, op, fan_ratio, dt_min)
        # ---- 水箱：进水阀 DO4 开度即开关量，出水负载按时间表 ----
        inflow_open = self.bus.read_bool("DO4")
        self.tank.step(inflow_open, minute_of_day, dt_min)
        # ---- 写测量值回总线 ----
        for i, room in enumerate(self.rooms):
            pv = room.measure(self.abs_minute)
            self.bus.write(self.ROOM_AI[i], pv)
        self.bus.write("AI4", self.weather.temperature(minute_of_day, dt_min))
        self.bus.write("AI5", self.tank.level)
        # 冷冻水供水温度：近似恒定 7℃ 加小幅波动
        self.bus.write("AI6", 7.0 + self.rng.gauss(0, 0.2))
        # DI 反馈：防火阀由外部事件驱动（run_day 注入火灾演练），此处保持原值；
        # 水箱高位浮球硬限位
        self.bus.write("DI4", 1.0 if self.tank.level < self.tank.height_hard_max - 0.05 else 0.0)
        return {
            "minute_of_day": minute_of_day,
            "abs_minute": self.abs_minute,
            "fan_ratio": fan_ratio,
            "ops": ops,
            "q_cools": [r.q_cool for r in self.rooms],
            "temps": [r.temp for r in self.rooms],
            "tank_level": self.tank.level,
            "outflow": self.tank.current_outflow,
        }


if __name__ == "__main__":
    # 简单自测：无控制自由漂移 48 小时，观察室温随室外温度和负荷变化趋势
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    bus = PointBus()
    bus.write("DO2", 1.0)
    bus.write("AO4", 0.0)     # 风机频率 0 → 无冷量
    plant = BuildingPlant(bus, seed=7)
    print("分钟  室外℃   办公室℃  会议室℃  大堂℃   水箱m")
    for m in range(1440 * 2):
        mod = m % 1440
        plant.step(mod)
        if m % 180 == 0:
            t = [round(bus.read(a), 2) for a in ("AI1", "AI2", "AI3")]
            print(f"{m:>5} {bus.read('AI4'):6.2f} {t[0]:8.2f} {t[1]:8.2f} "
                  f"{t[2]:8.2f} {bus.read('AI5'):7.2f}")
