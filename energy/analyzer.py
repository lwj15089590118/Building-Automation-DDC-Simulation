# -*- coding: utf-8 -*-
"""
energy/analyzer.py —— 能耗统计与策略对比分析
============================================

【能耗折算原理】
1) 冷量能耗（空调主侧）：
   盘管每个周期输出的冷量 Q_i(W) 在 Δt(s) 内累计：
       E_cool = Σ Q_i · Δt / 3.6e6   →  kWh
   工程上冷量由"冷冻水流量×供回水温差"计量，本仿真中冷量直接来自
   RC 模型的物理计算，等价于装了理想冷量计。
2) 风机能耗：按额定功率 × (频率/工频)^3 的风机相似定律近似：
       P_fan = P_rated · (f/50)³ ，运行时积分
3) 水泵能耗：额定功率 × 启停状态积分。

【策略对比】
分别以"节能模式开"(夜间 setback+停机)与"节能模式关"(全天 24℃ 连续供冷)
跑完一整天，得到两种策略的总能耗：
       节能率 = (E_关 − E_开) / E_关 × 100%
"""

from dataclasses import dataclass


@dataclass
class EnergyResult:
    """一段运行时间的能耗结果(kWh)。"""
    name: str                      # 策略名称，如 "节能模式开"
    cooling_kwh: float = 0.0       # 冷量能耗
    fan_kwh: float = 0.0           # 送/排风机能耗
    pump_kwh: float = 0.0          # 冷冻水泵能耗

    @property
    def total_kwh(self) -> float:
        """总能耗 = 冷量 + 风机 + 水泵。"""
        return self.cooling_kwh + self.fan_kwh + self.pump_kwh


class EnergyAnalyzer:
    """
    实时能耗累加器：仿真主循环每分钟调用一次 update()。
    功率单位 W、时间单位分钟的组合内部统一换算为 kWh。
    """

    #: 设备铭牌参数(仿真假设值)
    FAN_RATED_W = 5500.0      # 送风机额定功率 W（50Hz 工频）
    FAN_RATED_EXH_W = 2200.0  # 排风机额定功率 W
    PUMP_RATED_W = 1500.0     # 冷冻水泵额定功率 W

    def __init__(self, result_name: str = "运行段") -> None:
        self.result = EnergyResult(result_name)
        self._dt_h = 1.0 / 60.0        # 每周期 1 分钟 = 1/60 小时

    def update(self, total_cool_w: float, fan_on: bool, fan_hz: float,
               pump_on: bool, dt_min: float = 1.0) -> None:
        """
        累计一个周期的能耗。
        :param total_cool_w: 三房间盘管瞬时制冷量之和 W
        :param fan_on:       送风机是否运行
        :param fan_hz:       送风机当前频率 Hz（0~50）
        :param pump_on:      冷冻水泵是否运行
        """
        dt_h = dt_min / 60.0
        # ---- 冷量 ----
        self.result.cooling_kwh += max(0.0, total_cool_w) * dt_h / 1000.0
        # ---- 风机（相似定律：功耗 ∝ 频率³）----
        if fan_on and fan_hz > 0.5:
            ratio = min(1.0, fan_hz / 50.0)
            self.result.fan_kwh += ((EnergyAnalyzer.FAN_RATED_W * ratio ** 3)
                                    + EnergyAnalyzer.FAN_RATED_EXH_W) * dt_h / 1000.0
        # ---- 水泵 ----
        if pump_on:
            self.result.pump_kwh += EnergyAnalyzer.PUMP_RATED_W * dt_h / 1000.0


def savings_rate(base_kwh: float, saving_kwh: float) -> float:
    """
    计算节能率 %：以 base(节能关，基准工况) 为分母。
    若基准为 0（异常），返回 0.0。
    """
    if base_kwh <= 0:
        return 0.0
    return (base_kwh - saving_kwh) / base_kwh * 100.0


def compare_strategies(result_off: EnergyResult,
                       result_on: EnergyResult) -> dict:
    """
    对比两种策略，返回日报所需的对比字典。
    :param result_off: 节能模式关（基准）
    :param result_on:  节能模式开
    """
    rate = savings_rate(result_off.total_kwh, result_on.total_kwh)
    return {
        "基准策略": {"name": result_off.name, "cooling": round(result_off.cooling_kwh, 2),
                  "fan": round(result_off.fan_kwh, 2), "pump": round(result_off.pump_kwh, 2),
                  "total": round(result_off.total_kwh, 2)},
        "对比策略": {"name": result_on.name, "cooling": round(result_on.cooling_kwh, 2),
                  "fan": round(result_on.fan_kwh, 2), "pump": round(result_on.pump_kwh, 2),
                  "total": round(result_on.total_kwh, 2)},
        "节能量_kwh": round(result_off.total_kwh - result_on.total_kwh, 2),
        "节能率_percent": round(rate, 1),
    }


if __name__ == "__main__":
    # 自测：模拟恒定满负荷冷量 6kW + 风机 50Hz + 水泵常开，验证 1 天能耗数量级
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ea = EnergyAnalyzer("自测")
    for _ in range(1440):
        ea.update(total_cool_w=6000.0, fan_on=True, fan_hz=50.0, pump_on=True)
    r = ea.result
    print(f"冷量={r.cooling_kwh:.2f} kWh(期望144.00)")
    print(f"风机={(r.fan_kwh):.2f} kWh(期望185.60=7.7kW×24h)")
    print(f"水泵={r.pump_kwh:.2f} kWh(期望36.00)")
    print(f"总计={r.total_kwh:.2f} kWh")
