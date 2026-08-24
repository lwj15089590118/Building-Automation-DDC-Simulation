# -*- coding: utf-8 -*-
"""
ddc 包：DDC（Direct Digital Control，直接数字控制器）程序模块。

包含：
- simtime.py        仿真时刻值对象 SimTime(天+分钟，绝对分钟/显示串派生)
- pid.py            PID 连续调节算法(位置式+抗积分饱和)
- alarms.py         报警记录 AlarmRecord 与队列管理器 AlarmManager
- interlock.py      风机链路联锁顺序状态机 FanInterlock
- ddc_controller.py DDC 主控制器：时间表/PID死区/水箱位式/报警条件判定，
                    并组合上述模块完成每周期"采样→运算→输出"
"""
