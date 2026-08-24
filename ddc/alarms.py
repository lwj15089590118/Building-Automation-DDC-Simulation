# -*- coding: utf-8 -*-
"""
ddc/alarms.py —— 报警记录与队列管理
====================================

把"报警"从控制逻辑中独立出来：
  - AlarmRecord   一条报警的结构化记录；
  - AlarmManager  队列的增删查与去重、计数、导出（看板/日报只跟它打交道，
                  不再深入控制器内部结构）。

去重规则：同一 key 的报警在"活动期间"只产生一条新记录；
复位规则：条件消失后调用 clear() 解除活动标记，允许下次再次触发。
"""

from ddc.simtime import SimTime


class AlarmRecord:
    """一条报警记录（进入报警队列）。"""

    __slots__ = ("abs_minute", "time_str", "level", "source", "message")

    def __init__(self, abs_minute: int, time_str: str,
                 level: str, source: str, message: str) -> None:
        self.abs_minute = abs_minute   # 绝对仿真分钟
        self.time_str = time_str       # "第X天 HH:MM"
        self.level = level             # "提示" / "警告" / "重要"
        self.source = source           # 报警来源点位，如 "AI1"
        self.message = message         # 中文报警描述


class AlarmManager:
    """报警队列管理器：触发(去重) / 复位 / 活动查询 / 导出。"""

    def __init__(self) -> None:
        self.queue: list[AlarmRecord] = []     # 只增不减的历史队列
        self._active: dict[str, bool] = {}     # 活动(未复位)报警去重表
        self.count = 0                         # 报警发生次数(含已复位的)

    def trigger(self, now: SimTime, key: str, level: str,
                source: str, message: str) -> bool:
        """
        触发一条报警（同 key 活动期间自动去重）。
        :return: 是否真正产生了新记录。
        """
        if self._active.get(key):
            return False
        self._active[key] = True
        self.queue.append(
            AlarmRecord(now.abs_minute, now.fmt, level, source, message))
        self.count += 1
        return True

    def clear(self, key: str) -> None:
        """报警条件消失后复位活动标记（允许下次再报）。"""
        self._active[key] = False

    def any_active(self) -> bool:
        """是否有任何处于活动状态的报警（联动声光报警器用）。"""
        return any(self._active.values())

    def recent(self, limit: int | None = None) -> list[dict]:
        """
        导出报警记录（新在前），供看板/日报等外部展示使用。
        :param limit: 最多返回条数；None=全部导出。
        """
        q = self.queue[-limit:] if limit else self.queue
        return [
            {"time": a.time_str, "level": a.level,
             "source": a.source, "message": a.message}
            for a in q[::-1]
        ]
