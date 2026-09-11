#!/usr/bin/env python3
"""NMPC bench log (nmpc_bench CSV) をグラフ化するスクリプト."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

# control.cpp の #cols: 行と一致（フォールバック用の既定値）。
# 2026-09-11: ログに #cols: 行があればそれを優先するようにした。
# control.cpp 側で列を足したときにここを直し忘れても、新しい列が黙って捨てられない。
COLUMNS = [
    "seq", "arm", "nmpc", "phi", "theta", "p", "q", "r",
    "phi_ref", "theta_ref", "psi_ref", "T_ref", "u_base",
    "schedule_alpha", "Q_phi_eff",
    "iter_us", "loop_us", "cost", "reset", "iter_max",
    "reset_cause", "path", "tr1", "cycles",
    "cnt_tr1", "cnt_tr1_ok", "cnt_tr1_skip",
    "cnt_rst_period", "cnt_rst_ext", "cnt_sr2_c", "cnt_sr2_llt", "cnt_rst_after_sr2",
    "u_fr", "u_fl", "u_mr", "u_ml", "u_rr", "u_rl",
    "d_fr", "d_fl", "d_mr", "d_ml", "d_rr", "d_rl",
]

SAMPLE_HZ = 10.0  # control.cpp の bench CSV 出力レート

RESET_CAUSE_LABELS = {
    0: "none",
    1: "init",
    2: "external",
    3: "period",
    4: "sr2_bad_c",
    5: "sr2_unhealthy",
}

U_COLS = ["u_fr", "u_fl", "u_mr", "u_ml", "u_rr", "u_rl"]

# nmpc_model.cpp / plan.md の Hexa-X 配置 (x: 前+, y: 右+)
#   FR 前右  FL 前左  MR 右  ML 左  RR 後右  RL 後左
MOTOR_GROUPS = {
    # roll: B_c[2,k] ∝ -y → 右(y>0) vs 左(y<0)
    "roll_right": ["u_fr", "u_mr", "u_rr"],
    "roll_left":  ["u_fl", "u_ml", "u_rl"],
    # pitch: B_c[3,k] ∝ +x → 前(x>0) vs 後(x<0)
    # MR/ML は x=0 のためピッチモーメントに寄与せず除外
    "pitch_front": ["u_fr", "u_fl"],
    "pitch_rear":  ["u_rr", "u_rl"],
    # yaw: B_c[4,k] ∝ s_k → CW(s=-1) vs CCW(s=+1)  (nmpc_model.cpp kS)
    "yaw_cw":  ["u_fr", "u_ml", "u_rr"],
    "yaw_ccw": ["u_fl", "u_mr", "u_rl"],
}


def parse_log(path: Path) -> tuple[pd.DataFrame, dict[str, str]]:
    """ログファイルを読み込み、DataFrame とメタデータを返す."""
    meta: dict[str, str] = {}
    rows: list[list[str]] = []

    cols_from_log: list[str] = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if line.startswith("#cols:"):
                    cols_from_log = [c.strip() for c in line.split(":", 1)[1].split(",") if c.strip()]
                    continue
                if ":" in line:
                    key, _, value = line[1:].partition(":")
                    meta[key.strip()] = value.strip()
                else:
                    meta.setdefault("notes", "")
                    meta["notes"] += (line[1:] + " ")
                continue
            rows.append(line.split(","))

    if not rows:
        raise ValueError(f"データ行がありません: {path}")

    columns = cols_from_log if cols_from_log else list(COLUMNS)
    n_cols = len(columns)
    normalized: list[list[str]] = []
    for i, row in enumerate(rows, start=1):
        if len(row) < n_cols:
            row = row + [""] * (n_cols - len(row))
        elif len(row) > n_cols:
            row = row[:n_cols]
        normalized.append(row)

    df = pd.DataFrame(normalized, columns=columns)
    for col in columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # 10 Hz で記録された CSV の行番号 → ログ先頭からの経過秒
    df["time_s"] = df.index.to_numpy(dtype=float) / SAMPLE_HZ
    return df, meta


def _group_mean(df: pd.DataFrame, cols: list[str]) -> pd.Series:
    """指定モータ duty の平均."""
    return df[cols].mean(axis=1)


def _plot_attitude_with_grouped_control(
    ax_left: plt.Axes,
    df: pd.DataFrame,
    time_col: str,
    att_cols: list[tuple[str, str, str]],
    att_ylabel: str,
    title: str,
    u_groups: list[tuple[str, list[str], str, str]],
) -> None:
    """左軸: 姿勢/角速度、右軸: グループ化した NMPC モータ入力."""
    for col, style, label in att_cols:
        ax_left.plot(df[time_col], df[col], style, label=label, linewidth=1.2)

    ax_left.set_ylabel(att_ylabel)
    ax_left.grid(True, alpha=0.3)
    ax_left.set_title(title)

    ax_right = ax_left.twinx()
    for label, cols, color, linestyle in u_groups:
        ax_right.plot(
            df[time_col], _group_mean(df, cols),
            color=color, linestyle=linestyle, linewidth=1.4, label=label,
        )
    ax_right.plot(
        df[time_col], df["u_base"],
        "k--", linewidth=1.0, alpha=0.6, label="u_base",
    )
    ax_right.set_ylabel("motor u [duty]")
    ax_right.set_ylim(-0.05, 1.05)

    lines_l, labels_l = ax_left.get_legend_handles_labels()
    lines_r, labels_r = ax_right.get_legend_handles_labels()
    ax_left.legend(
        lines_l + lines_r, labels_l + labels_r,
        loc="upper right", fontsize=8,
    )


def _add_reset_markers(ax, df: pd.DataFrame) -> None:
    resets = df[df["reset"] > 0]
    for _, row in resets.iterrows():
        ax.axvline(row["time_s"], color="red", alpha=0.25, linewidth=0.8)
        cause = int(row["reset_cause"]) if pd.notna(row["reset_cause"]) else -1
        label = RESET_CAUSE_LABELS.get(cause, f"cause={cause}")
        ax.text(
            row["time_s"],
            ax.get_ylim()[1],
            label,
            rotation=90,
            va="top",
            ha="right",
            fontsize=7,
            color="red",
            alpha=0.7,
        )


def plot_all(df: pd.DataFrame, meta: dict[str, str], out_dir: Path, show: bool) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    title_suffix = meta.get("nmpc_bench CSV (10Hz) iter_us_limit=10000 (100Hz)", "")
    if title_suffix:
        suptitle = f"NMPC bench log ({title_suffix})"
    else:
        suptitle = "NMPC bench log"

    # --- Figure 1: 姿勢・角速度 ---
    fig1, axes1 = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig1.suptitle(suptitle)

    ax = axes1[0]
    ax.plot(df["time_s"], df["phi"], label="phi")
    ax.plot(df["time_s"], df["theta"], label="theta")
    ax.plot(df["time_s"], df["phi_ref"], "--", label="phi_ref", alpha=0.8)
    ax.plot(df["time_s"], df["theta_ref"], "--", label="theta_ref", alpha=0.8)
    ax.set_ylabel("angle [rad]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("Attitude")
    _add_reset_markers(ax, df)

    ax = axes1[1]
    ax.plot(df["time_s"], df["p"], label="p")
    ax.plot(df["time_s"], df["q"], label="q")
    ax.plot(df["time_s"], df["r"], label="r")
    if "psi_ref" in df.columns:
        ax.plot(df["time_s"], df["psi_ref"], "--", label="psi_ref", alpha=0.8)
    ax.set_ylabel("rate [rad/s]")
    ax.set_xlabel("time [s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("Angular rates")
    _add_reset_markers(ax, df)

    fig1.tight_layout()
    fig1.savefig(out_dir / "01_attitude_rates.png", dpi=150)
    if show:
        fig1.show()

    # --- Figure 2: NMPC 計算時間・コスト ---
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig2.suptitle(suptitle)

    ax = axes2[0]
    ax.plot(df["time_s"], df["iter_us"], label="iter_us", linewidth=0.8)
    ax.plot(df["time_s"], df["loop_us"], label="loop_us", linewidth=0.8, alpha=0.7)
    limit_match = re.search(r"iter_us_limit=(\d+)", " ".join(meta.values()))
    if limit_match:
        limit = float(limit_match.group(1))
        ax.axhline(limit, color="red", linestyle="--", label=f"limit={limit:.0f} us")
    ax.set_ylabel("time [us]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("Solver timing")
    _add_reset_markers(ax, df)

    ax = axes2[1]
    ax.plot(df["time_s"], df["cost"], color="tab:purple", linewidth=0.8)
    ax.set_ylabel("cost")
    ax.grid(True, alpha=0.3)
    ax.set_title("NMPC cost")
    _add_reset_markers(ax, df)

    ax = axes2[2]
    ax.step(df["time_s"], df["nmpc"], where="post", label="nmpc_active")
    ax.step(df["time_s"], df["arm"], where="post", label="arm", alpha=0.7)
    ax.set_ylabel("flag")
    ax.set_xlabel("time [s]")
    ax.set_ylim(-0.1, 2.5)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("Status flags")
    _add_reset_markers(ax, df)

    fig2.tight_layout()
    fig2.savefig(out_dir / "02_nmpc_timing_cost.png", dpi=150)
    if show:
        fig2.show()

    # --- Figure 3: モータ入力 ---
    fig3, axes3 = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig3.suptitle(suptitle)

    ax = axes3[0]
    for name in U_COLS:
        ax.plot(df["time_s"], df[name], label=name, linewidth=0.8)
    ax.plot(df["time_s"], df["u_base"], "k--", label="u_base", alpha=0.6)
    if "T_ref" in df.columns:
        ax.plot(df["time_s"], df["T_ref"], color="gray", linestyle=":", label="T_ref", alpha=0.7)
    ax.set_ylabel("u")
    ax.legend(loc="upper right", ncol=4, fontsize=7)
    ax.grid(True, alpha=0.3)
    ax.set_title("Motor inputs")
    _add_reset_markers(ax, df)

    ax = axes3[1]
    for name in ["d_fr", "d_fl", "d_mr", "d_ml", "d_rr", "d_rl"]:
        ax.plot(df["time_s"], df[name], label=name, linewidth=0.8)
    ax.set_ylabel("delta u")
    ax.set_xlabel("time [s]")
    ax.legend(loc="upper right", ncol=3, fontsize=7)
    ax.grid(True, alpha=0.3)
    ax.set_title("NMPC delta (u - u_base)")
    _add_reset_markers(ax, df)

    fig3.tight_layout()
    fig3.savefig(out_dir / "03_motor_inputs.png", dpi=150)
    if show:
        fig3.show()

    # --- Figure 4: 診断カウンタ ---
    fig4, ax4 = plt.subplots(figsize=(12, 4))
    fig4.suptitle(suptitle)
    counter_cols = [
        "cnt_tr1", "cnt_tr1_ok", "cnt_tr1_skip",
        "cnt_rst_period", "cnt_rst_ext", "cnt_sr2_c", "cnt_sr2_llt", "cnt_rst_after_sr2",
    ]
    for name in counter_cols:
        ax4.plot(df["time_s"], df[name], label=name, linewidth=0.9)
    ax4.set_xlabel("time [s]")
    ax4.set_ylabel("count")
    ax4.legend(loc="upper left", ncol=2, fontsize=8)
    ax4.grid(True, alpha=0.3)
    ax4.set_title("Diagnostic counters")
    fig4.tight_layout()
    fig4.savefig(out_dir / "04_diagnostics.png", dpi=150)
    if show:
        fig4.show()

    # --- Figure 5: roll / pitch / r と NMPC 制御入力の重ね合わせ ---
    fig5, axes5 = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig5.suptitle(suptitle + " — attitude vs control input")

    _plot_attitude_with_grouped_control(
        axes5[0], df, "time_s",
        att_cols=[
            ("phi", "-", "phi (roll)"),
            ("phi_ref", "--", "phi_ref"),
        ],
        att_ylabel="roll [rad]",
        title="Roll vs motor u (right / left)",
        u_groups=[
            ("u right (FR,MR,RR)", MOTOR_GROUPS["roll_right"], "#d62728", "-"),
            ("u left  (FL,ML,RL)", MOTOR_GROUPS["roll_left"],  "#1f77b4", "-"),
        ],
    )
    _add_reset_markers(axes5[0], df)

    _plot_attitude_with_grouped_control(
        axes5[1], df, "time_s",
        att_cols=[
            ("theta", "-", "theta (pitch)"),
            ("theta_ref", "--", "theta_ref"),
        ],
        att_ylabel="pitch [rad]",
        title="Pitch vs motor u (front / rear)",
        u_groups=[
            ("u front (FR,FL)", MOTOR_GROUPS["pitch_front"], "#2ca02c", "-"),
            ("u rear  (RR,RL)", MOTOR_GROUPS["pitch_rear"],  "#ff7f0e", "-"),
        ],
    )
    _add_reset_markers(axes5[1], df)

    _plot_attitude_with_grouped_control(
        axes5[2], df, "time_s",
        att_cols=[("r", "-", "r (yaw rate)")],
        att_ylabel="yaw rate [rad/s]",
        title="Yaw rate vs motor u (CW / CCW)",
        u_groups=[
            ("u CW  (FR,ML,RR)", MOTOR_GROUPS["yaw_cw"],  "#9467bd", "-"),
            ("u CCW (FL,MR,RL)", MOTOR_GROUPS["yaw_ccw"], "#8c564b", "-"),
        ],
    )
    _add_reset_markers(axes5[2], df)
    axes5[2].set_xlabel("time [s]")

    fig5.tight_layout()
    fig5.savefig(out_dir / "05_attitude_control_overlay.png", dpi=150)
    if show:
        fig5.show()

    if not show:
        plt.close("all")


def main() -> None:
    parser = argparse.ArgumentParser(description="NMPC bench ログをグラフ化")
    parser.add_argument(
        "log_file",
        nargs="?",
        default="log/nmpc_1.log",
        help="入力ログファイル (default: log/nmpc_1.log)",
    )
    parser.add_argument(
        "-o", "--output",
        default=None,
        help="PNG 出力ディレクトリ (default: log/plots/<log名>)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="グラフを画面表示する",
    )
    args = parser.parse_args()

    log_path = Path(args.log_file)
    if not log_path.is_file():
        raise SystemExit(f"ファイルが見つかりません: {log_path}")

    out_dir = Path(args.output) if args.output else log_path.parent / "plots" / log_path.stem

    df, meta = parse_log(log_path)
    print(f"Loaded {len(df)} rows from {log_path}")
    print(f"Time range: {df['time_s'].min():.2f} - {df['time_s'].max():.2f} s")
    print(f"NMPC active samples: {int((df['nmpc'] > 0).sum())}")
    print(f"Reset events: {int((df['reset'] > 0).sum())}")

    plot_all(df, meta, out_dir, show=args.show)
    print(f"Saved plots to {out_dir}/")


if __name__ == "__main__":
    main()
