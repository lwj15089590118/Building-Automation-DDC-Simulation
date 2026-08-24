# -*- coding: utf-8 -*-
"""
ddc/pid.py —— PID 控制算法（独立模块，与具体被控对象解耦）
============================================================

位置式 PID + 抗积分饱和，输出限幅 [out_min, out_max]。
算法不感知"温度/阀开度"等业务语义——那由调用方(DdcController)解释；
因此本类可直接复用于任何连续量调节回路。
"""


class PIDController:
    """
    制冷工况 PID：
      偏差 e = PV − SP（PV 越高于设定值，需要越大的执行器输出）
      u(k) = Kp·e + Ki·Σ(e·Δt) + Kd·(e−e_{k−1})/Δt ，输出限幅 [0, 100]%
    抗积分饱和：输出到达限幅且偏差继续同向时，冻结积分（防止退饱和超调）。
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = 0.0, out_max: float = 100.0) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0      # 积分累计量 Σ(e·Δt)
        self.prev_err: float | None = None   # 上一拍偏差(None 表示刚复位)
        self.output = 0.0        # 当前输出 %

    def reset(self) -> None:
        """复位（系统重新启动/死区关闭后调用，避免旧积分引起突跳）。"""
        self.integral = 0.0
        self.prev_err = None
        # 注意：output 不清零——死区内要求"保持原开度"

    def update(self, err: float, dt_min: float) -> float:
        """按当前偏差计算新输出（%）。"""
        # ---- 比例项 ----
        p_term = self.kp * err
        # ---- 微分项（对偏差微分，首拍不微分）----
        if self.prev_err is None:
            d_term = 0.0
        else:
            d_term = self.kd * (err - self.prev_err) / dt_min
        self.prev_err = err
        # ---- 试探性加入积分，检查是否饱和 ----
        trial_integral = self.integral + self.ki * err * dt_min
        raw = p_term + trial_integral + d_term
        if (raw > self.out_max and trial_integral > self.integral) or \
           (raw < self.out_min and trial_integral < self.integral):
            pass                      # 饱和且积分还在恶化 → 冻结积分(抗饱和)
        else:
            self.integral = trial_integral
        self.output = max(self.out_min, min(self.out_max,
                                            p_term + self.integral + d_term))
        return self.output
