# -*- coding: utf-8 -*-
"""飛行ログから回転ダンピング Cp/Cq/Cr の識別性を診断する。

角加速度を直接差分せず、運動方程式を時間窓で積分して回帰する。
ただし入力は指令 duty であり実モータ推力ではないため、遅延・低域通過・
積分窓を振った感度を必ず併記し、物理的に一貫しない場合は値を採用しない。

実行例:
  python3 tools/estimate_rotational_damping.py \
    tools/0904_v12.csv tools/0904_v11.csv tools/0904_100Hz_v4.csv
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
import numpy as np
from scipy.optimize import lsq_linear
from scipy.signal import butter, sosfiltfilt

for _font in ("Hiragino Sans", "AppleGothic", "Noto Sans CJK JP"):
    if any(_font == item.name for item in font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.family"] = _font
        break


MOTORS = ("fr", "fl", "mr", "ml", "rr", "rl")
MOTOR_X = np.array([0.085, 0.085, 0.0, 0.0, -0.085, -0.085])
MOTOR_Y = np.array([0.050, -0.050, 0.100, -0.100, 0.050, -0.050])
MOTOR_S = np.array([-1.0, 1.0, 1.0, -1.0, -1.0, 1.0])
INERTIA = np.array([1.8e-3, 1.9e-3, 3.0e-3])
C_QT = 5.8e-9 / 5.21e-7
AXES = ("p", "q", "r")


@dataclass
class LogData:
    path: Path
    columns: Dict[str, np.ndarray]
    time: np.ndarray
    sample_rate_hz: float


def load_log(path: Path) -> LogData:
    names: List[str] | None = None
    rows: List[List[float]] = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if line.startswith("#cols:"):
            names = [x.strip() for x in line.split(":", 1)[1].split(",")]
            continue
        if not line or line.startswith("#") or names is None:
            continue
        fields = [x.strip() for x in line.split(",")]
        if fields == names or len(fields) != len(names):
            continue
        try:
            rows.append([float(x) for x in fields])
        except ValueError:
            continue
    if not rows or names is None:
        return LogData(path, {}, np.array([]), 0.0)
    array = np.asarray(rows, dtype=float)
    columns = {name: array[:, i] for i, name in enumerate(names)}
    if "t" in columns:
        time = columns["t"].copy()
    elif "seq" in columns:
        # seq は 100 Hz 制御周期カウンタ。UDP欠測を含む実時刻として使う。
        time = (columns["seq"] - columns["seq"][0]) / 100.0
    else:
        raise ValueError(f"時刻列 t/seq がありません: {path}")
    valid_dt = np.diff(time)
    valid_dt = valid_dt[valid_dt > 0]
    fs = 1.0 / np.median(valid_dt) if valid_dt.size else 0.0
    return LogData(path, columns, time, float(fs))


def thrust_from_duty(duty: np.ndarray) -> np.ndarray:
    """f1404 + GEMFAN-3630-3、2Sベンチの T(u) [N]。"""
    duty = np.clip(duty, 0.0, 1.0)
    thrust = -0.2265 * duty * duty + 1.0525 * duty + 0.0452
    return np.where(duty <= 1e-6, 0.0, np.maximum(thrust, 0.0))


def lowpass(values: np.ndarray, fs: float, cutoff_hz: float) -> np.ndarray:
    sos = butter(4, cutoff_hz, btype="low", fs=fs, output="sos")
    return sosfiltfilt(sos, values, axis=0)


def motor_torque(duty: np.ndarray) -> np.ndarray:
    thrust = thrust_from_duty(duty)
    return np.column_stack((
        -thrust @ MOTOR_Y,
        thrust @ MOTOR_X,
        C_QT * (thrust @ MOTOR_S),
    ))


def integrated_regression_rows(
    time: np.ndarray,
    omega: np.ndarray,
    torque: np.ndarray,
    axis: int,
    delay_samples: int,
    window_samples: int,
    stride: int = 2,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """y = gain*tau_int - C*quad_int - b*linear_int + bias*dt."""
    if delay_samples:
        t = time[delay_samples:]
        w = omega[delay_samples:]
        tau = torque[:-delay_samples]
    else:
        t, w, tau = time, omega, torque
    p, q, r = w.T
    coupling = (
        (INERTIA[2] - INERTIA[1]) * q * r,
        (INERTIA[0] - INERTIA[2]) * r * p,
        (INERTIA[1] - INERTIA[0]) * p * q,
    )[axis]
    x_rows: List[List[float]] = []
    y_rows: List[float] = []
    mid_time: List[float] = []
    for start in range(0, len(t) - window_samples, stride):
        stop = start + window_samples
        sl = slice(start, stop + 1)
        tw = t[sl]
        if np.max(np.diff(tw)) > 1.5 / 100.0:
            continue
        integ = lambda v: float(np.trapezoid(v[sl], tw))
        duration = float(t[stop] - t[start])
        y = INERTIA[axis] * (w[stop, axis] - w[start, axis]) + integ(coupling)
        om = w[:, axis]
        x_rows.append([
            integ(tau[:, axis]),
            -integ(om * np.abs(om)),
            -integ(om),
            duration,
        ])
        y_rows.append(y)
        mid_time.append(0.5 * (t[start] + t[stop]))
    return np.asarray(x_rows), np.asarray(y_rows), np.asarray(mid_time)


def fit_unconstrained(x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, float]:
    beta = np.linalg.lstsq(x, y, rcond=None)[0]
    pred = x @ beta
    denom = np.sum((y - np.mean(y)) ** 2)
    r2 = 1.0 - np.sum((y - pred) ** 2) / denom if denom > 0 else float("nan")
    return beta, float(r2)


def fit_physical(x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, float]:
    # gain, C, b は非負。定常バイアストルクだけ正負を許す。
    result = lsq_linear(
        x, y,
        bounds=(np.array([0.0, 0.0, 0.0, -np.inf]),
                np.array([3.0, 0.1, 0.1, np.inf])),
        lsmr_tol="auto",
    )
    beta = result.x
    pred = x @ beta
    denom = np.sum((y - np.mean(y)) ** 2)
    r2 = 1.0 - np.sum((y - pred) ** 2) / denom if denom > 0 else float("nan")
    return beta, float(r2)


def blocked_cv_nrmse(x: np.ndarray, y: np.ndarray, mid: np.ndarray, blocks: int = 5) -> float:
    edges = np.linspace(np.min(mid), np.max(mid) + 1e-12, blocks + 1)
    scores: List[float] = []
    for k in range(blocks):
        test = (mid >= edges[k]) & (mid < edges[k + 1])
        train = ~test
        if np.sum(test) < 10 or np.sum(train) < 20:
            continue
        beta, _ = fit_physical(x[train], y[train])
        err = y[test] - x[test] @ beta
        scale = np.std(y[test])
        if scale > 0:
            scores.append(float(np.sqrt(np.mean(err * err)) / scale))
    return float(np.mean(scores)) if scores else float("nan")


def analyze_100hz(log: LogData) -> dict:
    d = log.columns
    duty = np.column_stack([d[f"u_{m}"] for m in MOTORS])
    omega = np.column_stack([d[a] for a in AXES])
    configurations = []
    for cutoff in (2.0, 3.0, 5.0, 8.0):
        duty_f = lowpass(duty, log.sample_rate_hz, cutoff)
        omega_f = lowpass(omega, log.sample_rate_hz, cutoff)
        torque = motor_torque(duty_f)
        for delay_s in (0.03, 0.05, 0.07, 0.09, 0.12):
            delay_samples = int(round(delay_s * log.sample_rate_hz))
            for window_s in (0.10, 0.20, 0.50):
                window_samples = int(round(window_s * log.sample_rate_hz))
                for axis in range(3):
                    x, y, mid = integrated_regression_rows(
                        log.time, omega_f, torque, axis,
                        delay_samples, window_samples,
                    )
                    if len(y) < 30:
                        continue
                    beta_u, r2_u = fit_unconstrained(x, y)
                    beta_p, r2_p = fit_physical(x, y)
                    configurations.append({
                        "axis": AXES[axis],
                        "cutoff_hz": cutoff,
                        "delay_s": delay_s,
                        "window_s": window_s,
                        "rows": len(y),
                        "unconstrained": {
                            "torque_gain": float(beta_u[0]),
                            "C": float(beta_u[1]),
                            "linear_b": float(beta_u[2]),
                            "bias_torque_Nm": float(beta_u[3]),
                            "r2": r2_u,
                        },
                        "physical": {
                            "torque_gain": float(beta_p[0]),
                            "C": float(beta_p[1]),
                            "linear_b": float(beta_p[2]),
                            "bias_torque_Nm": float(beta_p[3]),
                            "r2": r2_p,
                            "blocked_cv_nrmse": blocked_cv_nrmse(x, y, mid),
                        },
                        "condition_number": float(np.linalg.cond(x)),
                    })
    summary = {}
    for axis in AXES:
        rows = [r for r in configurations if r["axis"] == axis]
        c_u = np.array([r["unconstrained"]["C"] for r in rows])
        gain_u = np.array([r["unconstrained"]["torque_gain"] for r in rows])
        c_p = np.array([r["physical"]["C"] for r in rows])
        cv = np.array([r["physical"]["blocked_cv_nrmse"] for r in rows])
        r2 = np.array([r["physical"]["r2"] for r in rows])
        summary[axis] = {
            "configurations": len(rows),
            "unconstrained_C_median": float(np.median(c_u)),
            "unconstrained_C_min": float(np.min(c_u)),
            "unconstrained_C_max": float(np.max(c_u)),
            "unconstrained_C_positive_fraction": float(np.mean(c_u > 0.0)),
            "unconstrained_torque_gain_median": float(np.median(gain_u)),
            "physical_C_median": float(np.median(c_p)),
            "physical_C_at_zero_fraction": float(np.mean(c_p < 1e-7)),
            "physical_r2_median": float(np.median(r2)),
            "blocked_cv_nrmse_median": float(np.nanmedian(cv)),
            "identifiable": bool(
                np.mean(c_u > 0.0) >= 0.8
                and np.mean(c_p < 1e-7) <= 0.2
                and np.nanmedian(cv) < 1.0
            ),
        }
    return {
        "log": str(log.path),
        "samples": len(log.time),
        "duration_s": float(log.time[-1] - log.time[0]),
        "sample_rate_hz": log.sample_rate_hz,
        "duty_min": float(np.min(duty)),
        "duty_max": float(np.max(duty)),
        "max_abs_rate_rad_s": dict(zip(AXES, np.max(np.abs(omega), axis=0).tolist())),
        "summary": summary,
        "configurations": configurations,
    }


def make_plot(result: dict, output: Path) -> None:
    configs = result["configurations"]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    colors = {"p": "tab:blue", "q": "tab:orange", "r": "tab:green"}
    for axis in AXES:
        rows = [r for r in configs if r["axis"] == axis]
        c = np.array([r["unconstrained"]["C"] for r in rows])
        gain = np.array([r["unconstrained"]["torque_gain"] for r in rows])
        cv = np.array([r["physical"]["blocked_cv_nrmse"] for r in rows])
        axes[0, 0].scatter(gain, c, s=18, alpha=0.55, color=colors[axis], label=axis)
        axes[0, 1].hist(c, bins=18, alpha=0.45, color=colors[axis], label=axis)
        axes[1, 0].hist(cv[np.isfinite(cv)], bins=18, alpha=0.45,
                        color=colors[axis], label=axis)
        delay = np.array([r["delay_s"] for r in rows])
        axes[1, 1].scatter(delay, c, s=18, alpha=0.5, color=colors[axis], label=axis)
    axes[0, 0].axhline(0, color="black", lw=0.8)
    axes[0, 0].set(xlabel="推力トルク倍率", ylabel="非拘束 C [N m s²]",
                   title="推力倍率と二乗抵抗係数")
    axes[0, 1].axvline(0, color="black", lw=0.8)
    axes[0, 1].set(xlabel="非拘束 C [N m s²]", ylabel="条件数",
                   title="フィルタ・遅延・積分窓に対する C の分布")
    axes[1, 0].axvline(1, color="red", ls="--", lw=1)
    axes[1, 0].set(xlabel="5分割交差検証 NRMSE", ylabel="条件数",
                   title="1未満が最低限の予測性")
    axes[1, 1].axhline(0, color="black", lw=0.8)
    axes[1, 1].set(xlabel="仮定した入力遅延 [s]", ylabel="非拘束 C [N m s²]",
                   title="遅延仮定への感度")
    for ax in axes.flat:
        ax.grid(alpha=0.25)
        ax.legend()
    fig.suptitle("0904_100Hz_v4 回転ダンピング識別性診断")
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument(
        "--json",
        type=Path,
        default=Path("docs/rotational_damping_sysid_results.json"),
    )
    parser.add_argument(
        "--plot",
        type=Path,
        default=Path("docs/rotational_damping_sysid_diagnostics.png"),
    )
    args = parser.parse_args()

    logs = [load_log(path) for path in args.logs]
    inventory = []
    source_100hz = None
    for log in logs:
        if not log.columns:
            inventory.append({
                "log": str(log.path), "samples": 0, "use": "除外",
                "reason": "データ行なし",
            })
        elif log.sample_rate_hz < 50.0:
            inventory.append({
                "log": str(log.path), "samples": len(log.time),
                "sample_rate_hz": log.sample_rate_hz, "use": "除外",
                "reason": "10 Hzでは角加速度と50 Hz dutyチャタリングを再構成不能",
            })
        else:
            inventory.append({
                "log": str(log.path), "samples": len(log.time),
                "sample_rate_hz": log.sample_rate_hz, "use": "識別性診断",
                "reason": "100 Hzの角速度・6発指令dutyあり",
            })
            source_100hz = log
    if source_100hz is None:
        raise SystemExit("50 Hz以上で必要列を持つログがありません")

    result = analyze_100hz(source_100hz)
    output = {
        "method": {
            "equation": "window-integrated rigid-body rotational dynamics",
            "thrust_curve_N": "-0.2265*u^2 + 1.0525*u + 0.0452",
            "inertia_kg_m2": INERTIA.tolist(),
            "C_QT_m": C_QT,
            "filter_cutoff_hz": [2.0, 3.0, 5.0, 8.0],
            "assumed_input_delay_s": [0.03, 0.05, 0.07, 0.09, 0.12],
            "integration_window_s": [0.10, 0.20, 0.50],
            "acceptance": "Cの80%以上が正、境界0が20%以下、CV NRMSE中央値<1",
        },
        "inventory": inventory,
        "analysis": result,
    }
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n",
                         encoding="utf-8")
    make_plot(result, args.plot)

    print("ログ:")
    for item in inventory:
        print(f"  {item['log']}: {item['samples']}行, {item['use']} — {item['reason']}")
    print("識別性:")
    for axis in AXES:
        s = result["summary"][axis]
        print(
            f"  {axis}: identifiable={s['identifiable']} "
            f"C中央値={s['unconstrained_C_median']:.5g} "
            f"C正率={100*s['unconstrained_C_positive_fraction']:.1f}% "
            f"拘束解C=0率={100*s['physical_C_at_zero_fraction']:.1f}% "
            f"CV-NRMSE={s['blocked_cv_nrmse_median']:.2f}"
        )
    print(f"保存: {args.json}")
    print(f"保存: {args.plot}")


if __name__ == "__main__":
    main()
