"""NMPC Wi-Fi テレメトリのリアルタイムグラフ (nmpc_udp_client --show-graph)."""

from __future__ import annotations

from collections import deque
from typing import Any

import matplotlib.pyplot as plt

CONTROL_HZ = 100.0
SAMPLE_HZ = 10.0

U_COLS = ["u_fr", "u_fl", "u_mr", "u_ml", "u_rr", "u_rl"]
U_COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00", "#a65628"]

MOTOR_GROUPS = {
    "roll_right": ["u_fr", "u_mr", "u_rr"],
    "roll_left": ["u_fl", "u_ml", "u_rl"],
    "pitch_front": ["u_fr", "u_fl"],
    "pitch_rear": ["u_rr", "u_rl"],
    "yaw_cw": ["u_fr", "u_ml", "u_rr"],
    "yaw_ccw": ["u_fl", "u_mr", "u_rl"],
}


def row_time_s(row: dict[str, Any]) -> float:
    return float(row["seq"]) / CONTROL_HZ


def _group_mean_row(row: dict[str, Any], cols: list[str]) -> float:
    return sum(float(row[c]) for c in cols) / len(cols)


class RealtimeGraph:
    """スクロール表示するリアルタイムプロット。"""

    def __init__(self, window_s: float = 60.0) -> None:
        self.window_s = window_s
        maxlen = max(int(window_s * SAMPLE_HZ) + 20, 50)
        self._t: deque[float] = deque(maxlen=maxlen)
        self._series: dict[str, deque[float]] = {
            "phi": deque(maxlen=maxlen),
            "theta": deque(maxlen=maxlen),
            "phi_ref": deque(maxlen=maxlen),
            "theta_ref": deque(maxlen=maxlen),
            "p": deque(maxlen=maxlen),
            "q": deque(maxlen=maxlen),
            "r": deque(maxlen=maxlen),
            "psi_ref": deque(maxlen=maxlen),
            "u_base": deque(maxlen=maxlen),
            "T_ref": deque(maxlen=maxlen),
            "iter_us": deque(maxlen=maxlen),
            "cost": deque(maxlen=maxlen),
            "roll_right": deque(maxlen=maxlen),
            "roll_left": deque(maxlen=maxlen),
            "pitch_front": deque(maxlen=maxlen),
            "pitch_rear": deque(maxlen=maxlen),
            "yaw_cw": deque(maxlen=maxlen),
            "yaw_ccw": deque(maxlen=maxlen),
        }
        for name in U_COLS:
            self._series[name] = deque(maxlen=maxlen)

        plt.ion()
        self.fig, self.axes = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
        self.fig.suptitle("NMPC realtime telemetry")
        self._lines: dict[str, Any] = {}
        self._ax_cost: plt.Axes
        self._ax_tref: plt.Axes
        self._setup_axes()
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _setup_axes(self) -> None:
        ax = self.axes[0]
        for key, style in [
            ("phi", "-"), ("theta", "-"),
            ("phi_ref", "--"), ("theta_ref", "--"),
        ]:
            line, = ax.plot([], [], style, label=key, linewidth=1.2)
            self._lines[key] = line
        ax.set_ylabel("angle [rad]")
        ax.set_title("Attitude (solid: measured, dashed: stick ref)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

        ax = self.axes[1]
        for key, style in [
            ("p", "-"), ("q", "-"), ("r", "-"),
            ("psi_ref", "--"),
        ]:
            line, = ax.plot([], [], style, label=key, linewidth=1.2)
            self._lines[key] = line
        ax.set_ylabel("rate [rad/s]")
        ax.set_title("Angular rates (dashed: stick yaw rate ref)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

        ax = self.axes[2]
        for name, color in zip(U_COLS, U_COLORS):
            line, = ax.plot([], [], color=color, linewidth=0.9, label=name)
            self._lines[name] = line
        line, = ax.plot([], [], "k--", linewidth=1.0, alpha=0.6, label="u_base")
        self._lines["u_base_line"] = line
        ax_tref = ax.twinx()
        line_tref, = ax_tref.plot(
            [], [], color="gray", linestyle=":", linewidth=1.0, alpha=0.8, label="T_ref",
        )
        self._lines["T_ref"] = line_tref
        self._ax_tref = ax_tref
        ax.set_ylabel("u [duty]")
        ax_tref.set_ylabel("T_ref [V]")
        ax.set_ylim(-0.05, 1.05)
        ax.set_title("Motor inputs")
        ax.grid(True, alpha=0.3)
        leg_motors = ax.legend(
            [self._lines[name] for name in U_COLS],
            U_COLS,
            loc="upper right",
            ncol=3,
            fontsize=7,
        )
        ax.add_artist(leg_motors)
        ax.legend(
            [self._lines["u_base_line"], self._lines["T_ref"]],
            ["u_base", "T_ref"],
            loc="upper left",
            fontsize=7,
        )

        ax = self.axes[3]
        for key, color in [
            ("roll_right", "#d62728"), ("roll_left", "#1f77b4"),
            ("pitch_front", "#2ca02c"), ("pitch_rear", "#ff7f0e"),
            ("yaw_cw", "#9467bd"), ("yaw_ccw", "#8c564b"),
        ]:
            line, = ax.plot([], [], color=color, linewidth=1.2, label=key.replace("_", " "))
            self._lines[key] = line
        ax.set_ylabel("u [duty]")
        ax.set_ylim(-0.05, 1.05)
        ax.set_title("Grouped motor u (roll / pitch / yaw)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", ncol=2, fontsize=7)

        ax = self.axes[4]
        line, = ax.plot([], [], color="tab:blue", label="iter_us", linewidth=1.0)
        self._lines["iter_us"] = line
        ax2 = ax.twinx()
        line2, = ax2.plot([], [], color="tab:purple", label="cost", linewidth=1.0)
        self._lines["cost"] = line2
        ax.set_ylabel("iter_us [us]")
        ax2.set_ylabel("cost")
        ax.set_xlabel("time [s]")
        ax.set_title("Solver timing & cost")
        ax.grid(True, alpha=0.3)
        lines = [line, line2]
        ax.legend(lines, [ln.get_label() for ln in lines], loc="upper right", fontsize=8)
        self._ax_cost = ax2

    def append(self, row: dict[str, Any]) -> None:
        t = row_time_s(row)
        self._t.append(t)
        self._series["phi"].append(float(row["phi"]))
        self._series["theta"].append(float(row["theta"]))
        self._series["phi_ref"].append(float(row["phi_ref"]))
        self._series["theta_ref"].append(float(row["theta_ref"]))
        self._series["p"].append(float(row["p"]))
        self._series["q"].append(float(row["q"]))
        self._series["r"].append(float(row["r"]))
        self._series["psi_ref"].append(float(row["psi_ref"]))
        self._series["u_base"].append(float(row["u_base"]))
        self._series["T_ref"].append(float(row["T_ref"]))
        self._series["iter_us"].append(float(row["iter_us"]))
        self._series["cost"].append(float(row["cost"]))
        for name in U_COLS:
            self._series[name].append(float(row[name]))
        self._series["roll_right"].append(_group_mean_row(row, MOTOR_GROUPS["roll_right"]))
        self._series["roll_left"].append(_group_mean_row(row, MOTOR_GROUPS["roll_left"]))
        self._series["pitch_front"].append(_group_mean_row(row, MOTOR_GROUPS["pitch_front"]))
        self._series["pitch_rear"].append(_group_mean_row(row, MOTOR_GROUPS["pitch_rear"]))
        self._series["yaw_cw"].append(_group_mean_row(row, MOTOR_GROUPS["yaw_cw"]))
        self._series["yaw_ccw"].append(_group_mean_row(row, MOTOR_GROUPS["yaw_ccw"]))
        self._redraw()

    def _redraw(self) -> None:
        if not self._t:
            return
        t = list(self._t)
        t_min = max(t[0], t[-1] - self.window_s)
        t_max = t[-1] if t[-1] > t_min else t_min + 1.0

        for key, line in self._lines.items():
            if key == "u_base_line":
                line.set_data(t, list(self._series["u_base"]))
            elif key in self._series:
                line.set_data(t, list(self._series[key]))

        for ax in self.axes:
            ax.set_xlim(t_min, t_max)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
        self._ax_cost.relim()
        self._ax_cost.autoscale_view(scalex=False, scaley=True)
        self._ax_tref.relim()
        self._ax_tref.autoscale_view(scalex=False, scaley=True)
        self.axes[2].set_ylim(-0.05, 1.05)
        self.axes[3].set_ylim(-0.05, 1.05)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def poll(self) -> bool:
        """GUI イベント処理。ウィンドウが開いていれば True。"""
        if not plt.fignum_exists(self.fig.number):
            return False
        self.fig.canvas.flush_events()
        return True

    def close(self) -> None:
        if plt.fignum_exists(self.fig.number):
            plt.close(self.fig)
