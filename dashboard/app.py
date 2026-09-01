# -*- coding: utf-8 -*-
"""
dashboard/app.py —— Web 监控看板后端（Flask + ECharts）
========================================================

【架构说明】
本文件把"整栋楼的虚拟世界"跑在一个后台仿真线程里：
    BuildingPlant(受控对象) ⇄ PointBus(I/O 映像区) ⇄ DDCController(控制器)
                     ↓
          Modbus/TCP 从站(:5020)   ←—— 外部可用 modbus_client_test.py 访问
                     ↓
          Flask HTTP API(:5000)    ←—— 浏览器 index.html(ECharts) 轮询渲染

【功能】
  - 三房间温度趋势曲线(含 SP 死区阴影带)、设备状态灯、水箱液位；
  - 报警列表(实时滚动)、能耗柱状图(自动引用 run_day.py 生成的策略对比数据)；
  - 24h 时间轴回放滑块(拖动查看历史任意时刻)；
  - 设定温度(SP)在线修改按钮(经 PointBus→DDC 立即生效，等价上位机遥控);
  - 节能模式开关、仿真速度调节(暂停/60×/300×/600×)。

【运行】
  python dashboard/app.py     然后浏览器打开 http://127.0.0.1:5000
建议先运行一次 `python run_day.py`，看板能耗图即可显示两种策略对比。
"""

import json
import os
import sys
import threading
import time

# 保证在任何工作目录下都能导入项目包
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from flask import Flask, jsonify, render_template, request   # noqa: E402

from points.point_bus import PointBus                        # noqa: E402
from points.modbus_slave import ModbusSlaveServer            # noqa: E402
from points.point_defs import ROOM_NAMES                     # noqa: E402
from plant.thermal import BuildingPlant                      # noqa: E402
from ddc.ddc_controller import DDCController, SimTime         # noqa: E402
from energy.analyzer import EnergyAnalyzer                   # noqa: E402

#: 状态机状态 → 中文说明（看板显示用）
STATE_CN = {
    DDCController.ST_STOPPED: "系统停止",
    DDCController.ST_DAMPER_OPENING: "新风阀开启中(延时)",
    DDCController.ST_FAN_STARTING: "风机启动中(延时)",
    DDCController.ST_RUNNING: "系统运行",
    DDCController.ST_STOPPING: "停机中-退出冷冻水",
    DDCController.ST_DAMPER_CLOSING: "停机中-延迟关新风阀",
}


# ======================================================================
# 一、后台仿真引擎（线程）
# ======================================================================

class SimulationEngine(threading.Thread):
    """
    连续仿真线程：按设定的倍速推进"虚拟楼宇时间"，
    每仿真 1 分钟记录一条历史，供看板曲线与回放使用。
    """

    #: 历史缓冲长度（分钟）：2880 ≈ 两天滚动窗口
    HISTORY_MAX = 2880

    def __init__(self) -> None:
        super().__init__(name="SimEngine", daemon=True)
        # ---- 虚拟世界 ----
        self.bus = PointBus()
        self.bus.set_mode_bits(0x03)              # bit0 节能开, bit1 自动
        self.plant = BuildingPlant(self.bus, seed=2024)
        self.ddc = DDCController(self.bus, energy_saving=True)
        self.analyzer = EnergyAnalyzer("看板实时工况")
        # ---- 时钟与倍速 ----
        self.day = 1
        self.minute_of_day = 6 * 60               # 从第 1 天 06:00 起跑
        self.speed = 300                          # 时间倍速(0=暂停)
        # ---- 历史缓冲 ----
        self.history: list[dict] = []
        self._stop = threading.Event()

    # ---------------- 主循环 ----------------
    def run(self) -> None:
        """循环推进仿真；每步耗时约几毫秒，剩余时间按倍速休眠。"""
        while not self._stop.is_set():
            t0 = time.perf_counter()
            if self.speed > 0:
                self._step()
                # 1 个仿真分钟对应的真实秒数 = 60/倍速
                delay = 60.0 / self.speed - (time.perf_counter() - t0)
                if delay > 0:
                    time.sleep(delay)
            else:                                  # 暂停：低频空转等待恢复
                time.sleep(0.2)

    def _step(self) -> None:
        """推进一步（1 仿真分钟）并记录历史。"""
        snapshot = self.plant.step(self.minute_of_day)
        self.ddc.scan(SimTime(self.day, self.minute_of_day))
        fan_on = self.bus.read_bool("DO2")
        self.analyzer.update(sum(snapshot["q_cools"]), fan_on,
                             self.bus.read("AO4"), self.bus.read_bool("DO3"))
        # ---- 记录历史 ----
        pvs = []                       # 传感器故障时该房间为 None
        for i in range(3):
            v = self.bus.read_ai(f"AI{i+1}")
            pvs.append(round(v, 2) if v is not None else None)
        rec = {
            "t": (self.day - 1) * 1440 + self.minute_of_day,   # 绝对分钟
            "day": self.day,
            "mod": self.minute_of_day,
            "pv": pvs,
            "sp": [round(s, 1) for s in self.ddc.current_sp],
            "op": [round(self.bus.read(f"AO{i+1}"), 1) for i in range(3)],
            "fan": int(fan_on),
            "pump": int(self.bus.read_bool("DO3")),
            "damper": int(self.bus.read_bool("DO1")),
            "tank": round(self.bus.read("AI5"), 3),
            "tank_v": int(self.bus.read_bool("DO4")),
            "state": self.ddc.state,
            "alarms": self.ddc.alarm_count,
            "energy": round(self.analyzer.result.total_kwh, 3),
        }
        self.history.append(rec)
        if len(self.history) > SimulationEngine.HISTORY_MAX:
            del self.history[: len(self.history) - SimulationEngine.HISTORY_MAX]
        # ---- 时钟推进 ----
        self.minute_of_day += 1
        if self.minute_of_day >= 1440:
            self.minute_of_day = 0
            self.day += 1

    def stop(self) -> None:
        self._stop.set()


# ======================================================================
# 二、Flask 应用与 HTTP API
# ======================================================================

app = Flask(__name__)
engine = SimulationEngine()
slave = ModbusSlaveServer(engine.bus, port=5020)   # 默认绑定 127.0.0.1(可传参覆盖)


def _load_strategy_data() -> dict | None:
    """
    读取 run_day.py 生成的两份全天数据（若存在），
    供看板能耗柱状图直接展示"节能开/关"策略对比。
    """
    out = {}
    for tag, key in (("开", "on"), ("关", "off")):
        path = os.path.join(BASE_DIR, "docs", f"运行数据_节能{tag}.json")
        if os.path.exists(path):
            try:
                with open(path, encoding="utf-8") as f:
                    meta = json.load(f)["meta"]
                out[key] = {
                    "name": meta["name"],
                    "cooling": meta["energy"]["cooling"],
                    "fan": meta["energy"]["fan"],
                    "pump": meta["energy"]["pump"],
                    "total": meta["energy"]["total"],
                    "comfort_ok": meta.get("comfort_ok"),
                    "comfort_total": meta.get("comfort_total"),
                }
            except (OSError, KeyError, ValueError):
                pass
    return out if out else None


@app.route("/")
def index():
    """看板主页。"""
    return render_template("index.html")


@app.route("/api/state")
def api_state():
    """实时状态快照（前端 1 秒轮询）。"""
    bus = engine.bus
    snap = bus.read_all()

    def _pv(addr):
        v = snap[addr]
        return None if v != v else round(v, 2)      # NaN → null

    alarms = engine.ddc.recent_alarms(50)     # 最近 50 条，新在前
    energy_sum = engine.analyzer.summary()    # 能耗汇总(避免深入内部字段)
    return jsonify({
        "day": engine.day,
        "minute": engine.minute_of_day,
        "time_str": SimTime(engine.day, engine.minute_of_day).fmt,
        "speed": engine.speed,
        "paused": engine.speed == 0,
        "rooms": ROOM_NAMES,          # 房间名下发给前端，前端不硬编码
        "pv": [_pv(f"AI{i+1}") for i in range(3)],
        "outdoor": _pv("AI4"),
        "tank_level": _pv("AI5"),
        "chws_temp": _pv("AI6"),
        "sp": [round(s, 1) for s in engine.ddc.current_sp],
        "sp_manual": [bus.is_sp_manual(i) for i in range(3)],
        "op": [round(snap[f"AO{i+1}"], 1) for i in range(3)],
        "fan_hz": round(snap["AO4"], 1),
        "do": {n: int(bus.read(n)) for n in
               ("DO1", "DO2", "DO3", "DO4", "DO5", "DO6")},
        "di": {n: int(bus.read(n)) for n in ("DI1", "DI2", "DI3", "DI4")},
        "state": engine.ddc.state,
        "state_cn": STATE_CN.get(engine.ddc.state, engine.ddc.state),
        "mode": {"energy_saving": bus.energy_saving, "auto": bus.auto_mode},
        "alarm_count": engine.ddc.alarm_count,
        "interlock_count": engine.ddc.interlock_count,
        "startup_count": engine.ddc.startup_count,
        "alarms": alarms,
        "energy_today": energy_sum["total"],
        "energy_detail": {
            "cooling": energy_sum["cooling"],
            "fan": energy_sum["fan"],
            "pump": energy_sum["pump"],
        },
    })


@app.route("/api/history")
def api_history():
    """
    历史趋势数据。
    参数 window：返回最近多少分钟的记录（默认 1440 = 24h）。
    """
    window = request.args.get("window", default=1440, type=int)
    window = max(10, min(window, len(engine.history)))
    data = engine.history[-window:]
    return jsonify({
        "t": [r["t"] for r in data],
        "label": [f"{r['mod']//60:02d}:{r['mod']%60:02d}" for r in data],
        "day": [r["day"] for r in data],
        "pv": [[r["pv"][i] for r in data] for i in range(3)],
        "sp": [[r["sp"][i] for r in data] for i in range(3)],
        "op": [[r["op"][i] for r in data] for i in range(3)],
        "tank": [r["tank"] for r in data],
        "state": [r["state"] for r in data],
    })


@app.route("/api/sp", methods=["POST"])
def api_sp():
    """
    在线修改房间设定温度（等价于上位机经 Modbus 写 HR16~19）。
    请求体: {"room": 0~2, "action": "inc"|"dec"|"auto"}
    """
    body = request.get_json(force=True, silent=True) or {}
    try:
        room = int(body.get("room", -1))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "msg": "room 必须是 0/1/2 的整数"}), 400
    action = str(body.get("action", ""))
    if room not in (0, 1, 2):
        return jsonify({"ok": False, "msg": "房间编号必须是 0/1/2"}), 400
    bus = engine.bus
    if action == "auto":
        bits = bus.get_sp_manual_bits() & ~(1 << room)     # 清除手动位→回到时间表
        bus.set_sp_manual_bits(bits)
    elif action in ("inc", "dec"):
        cur = bus.get_manual_sp(room)
        new = cur + (0.5 if action == "inc" else -0.5)
        bus.set_manual_sp(room, new)
        bits = bus.get_sp_manual_bits() | (1 << room)      # 置手动位
        bus.set_sp_manual_bits(bits)
    else:
        return jsonify({"ok": False, "msg": "action 必须是 inc/dec/auto"}), 400
    return jsonify({"ok": True})


@app.route("/api/mode", methods=["POST"])
def api_mode():
    """切换节能模式/手自动模式。请求体: {"key": "energy_saving"|"auto"}"""
    body = request.get_json(force=True, silent=True) or {}
    bus = engine.bus
    bits = bus.get_mode_bits()
    if body.get("key") == "energy_saving":
        bus.set_mode_bits(bits ^ 0x01)
    elif body.get("key") == "auto":
        bus.set_mode_bits(bits ^ 0x02)
    else:
        return jsonify({"ok": False, "msg": "key 必须是 energy_saving/auto"}), 400
    return jsonify({"ok": True,
                    "bits": bus.get_mode_bits()})


@app.route("/api/speed", methods=["POST"])
def api_speed():
    """设置仿真倍速。请求体: {"speed": 0|60|300|600}"""
    body = request.get_json(force=True, silent=True) or {}
    try:
        speed = int(body.get("speed", engine.speed))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "msg": "speed 必须是整数"}), 400
    if speed not in (0, 60, 300, 600):
        return jsonify({"ok": False, "msg": "speed 只能是 0/60/300/600"}), 400
    engine.speed = speed
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/energy")
def api_energy():
    """能耗对比：run_day 全天结果(如有) + 看板本次运行的累计值。"""
    live = engine.analyzer.summary()
    live["name"] = "看板实时(自启动起)"
    return jsonify({
        "strategies": _load_strategy_data(),
        "live": live,
    })


# ======================================================================
# 三、入口
# ======================================================================

def main() -> None:
    print("=" * 70)
    print(" 楼宇 DDC 控制与能源管理仿真 —— Web 监控看板")
    print("=" * 70)
    try:
        slave.start()                   # 启动 Modbus/TCP 从站(端口 5020)
        print(f"[OK] Modbus/TCP 从站已启动: tcp://127.0.0.1:{slave.port} (Slave ID=1)")
    except RuntimeError as exc:
        # 从站启动失败(如端口被占用)不再静默：显式提示且不影响看板本体
        print(f"[警告] Modbus/TCP 从站未启动: {exc}")
        print("       (Web 看板功能不受影响，Modbus 联调请先释放端口后重启)")
    engine.start()                      # 启动后台仿真线程
    print("[OK] 仿真引擎已启动: 初始 第1天 06:00, 默认 300 倍速")
    print("[OK] 请用浏览器打开: http://127.0.0.1:5000")
    print("     另开终端可运行: python -m points.modbus_client_test  (通信联调)")
    print("-" * 70)
    try:
        # 关闭 Flask 自动重载，避免后台仿真线程被重复拉起
        app.run(host="127.0.0.1", port=5000, debug=False, use_reloader=False)
    finally:
        engine.stop()
        slave.stop()


if __name__ == "__main__":
    main()
