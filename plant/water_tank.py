# -*- coding: utf-8 -*-
"""
plant/water_tank.py —— 生活水箱液位对象（受控对象）
====================================================

【物理模型】
圆柱形水箱，液位动态由水量平衡决定：

    A · dL/dt = Qin − Qout

    A     : 水箱截面积 m²（等效值，已把流量单位统一为 m³/min）
    L     : 液位 m
    Qin   : 进水流量 = DO4 阀状态(0/1) × 阀全开流量
    Qout  : 出水负载扰动（按一天时间表变化：早晚用水高峰、夜间低谷，
            并叠加随机波动）——这是液位控制的"负荷扰动"

【限位保护】（真实水箱的机械/电气保护，控制程序之外的最后防线）
    高限 hard_max：到达后进水强制无效(浮球阀机械关闭)，防止溢流；
    低限 0      ：放空。

该对象只描述物理规律；"什么时候开/关进水阀"由 DDC 的位式控制决定。
"""

import random


class WaterTank:
    """生活水箱液位仿真对象。"""

    def __init__(self,
                 area_m2: float = 2.0,          # 截面积 m²
                 height_hard_max: float = 1.95,  # 硬限位(溢流保护) m
                 q_valve_max: float = 0.20) -> None:  # 进水阀全开流量 m³/min
        self.area = area_m2
        self.height_hard_max = height_hard_max
        self.q_valve_max = q_valve_max
        self.level = 1.40                       # 初始液位 m（处于回差区间内）
        self.current_inflow = 0.0               # 当前进水流量 m³/min（记录用）
        self.current_outflow = 0.0              # 当前出水流量 m³/min（记录用）
        self._rng = random.Random(99)

    # ---------------- 出水负载时间表 ----------------
    def demand(self, minute_of_day: int) -> float:
        """
        出水负载扰动 m³/min：
          06:30~08:30 晨峰 0.14 | 17:00~19:30 晚峰 0.12 |
          白天平段 0.05       | 夜间低谷 0.02 ，叠加 ±15% 随机波动。
        """
        h = minute_of_day / 60.0
        if 6.5 <= h < 8.5:
            base = 0.14
        elif 17.0 <= h < 19.5:
            base = 0.12
        elif 8.5 <= h < 17.0:
            base = 0.05
        else:
            base = 0.02
        return base * (1.0 + self._rng.uniform(-0.15, 0.15))

    # ---------------- 物理积分 ----------------
    def step(self, valve_open: bool, minute_of_day: int, dt_min: float = 1.0) -> None:
        """
        推进一个周期。
        :param valve_open: 进水阀是否打开（来自 DO4）
        """
        # 进水：阀门开且未顶到硬限位（模拟浮球阀机械自锁）
        if valve_open and self.level < self.height_hard_max:
            self.current_inflow = self.q_valve_max
        else:
            self.current_inflow = 0.0
        # 出水：负载时间表 + 随机扰动；液位见底时自然断水
        self.current_outflow = self.demand(minute_of_day)
        if self.level <= 0.0:
            self.current_outflow = min(self.current_outflow, 0.0)

        d_level = (self.current_inflow - self.current_outflow) / self.area * dt_min
        self.level += d_level
        # ---- 高低液位限位 ----
        if self.level > self.height_hard_max:
            self.level = self.height_hard_max
        if self.level < 0.0:
            self.level = 0.0

    def percent(self) -> float:
        """液位百分比（看板显示用）。"""
        return self.level / self.height_hard_max * 100.0


if __name__ == "__main__":
    # 自测：手动开关阀验证充放水动态
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    tank = WaterTank()
    print("分钟  阀  液位m   进水m³/min  出水m³/min")
    for m in range(1440):
        open_valve = tank.level < 1.0 or (tank.level < 1.8 and m % 200 < 100)
        tank.step(open_valve, m)
        if m % 60 == 0:
            print(f"{m:>5}  {'开' if open_valve else '关'}  {tank.level:5.3f}"
                  f"   {tank.current_inflow:6.3f}      {tank.current_outflow:6.3f}")
