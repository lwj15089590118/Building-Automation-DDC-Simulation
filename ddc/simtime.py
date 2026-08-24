# -*- coding: utf-8 -*-
"""
ddc/simtime.py —— 仿真时刻值对象
=================================

统一表达"仿真世界的当前时刻"，绝对分钟数与中文显示串均由此派生，
取代在控制器各方法间结伴穿透的 (abs_minute, day, mod) 三参数泥团。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SimTime:
    """
    仿真时刻值对象：第几天 + 当天分钟（不可变，可安全跨线程传递）。
    """
    day: int      # 第几天(从 1 开始)
    minute: int   # 当天第几分钟(0~1439)

    @property
    def abs_minute(self) -> int:
        """从仿真 0 时起算的绝对分钟数。"""
        return (self.day - 1) * 1440 + self.minute

    @property
    def fmt(self) -> str:
        """显示串："第X天 HH:MM"。"""
        return f"第{self.day}天 {self.minute // 60:02d}:{self.minute % 60:02d}"
