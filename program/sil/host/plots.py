#!/usr/bin/env python3
"""SIL ログのオフライン描画。

使い方:
    python3 plots.py                 # 最新のランを描画
    python3 plots.py <run_dir>       # 指定したランを描画
    python3 plots.py --show          # 画面表示も行う
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _use_japanese_font():
    """日本語が豆腐にならないようフォントを選ぶ。

    ラベルを英語に逃がすのではなく、リポジトリの記法（日本語）に合わせる。
    見つからない環境では警告だけ出して英語フォントのまま続行する。
    """
    import matplotlib.font_manager as fm
    available = {f.name for f in fm.fontManager.ttflist}
    for cand in ("Hiragino Sans", "Hiragino Maru Gothic Pro", "YuGothic",
                 "Noto Sans CJK JP", "Noto Sans JP", "IPAexGothic",
                 "Arial Unicode MS", "Apple SD Gothic Neo"):
        if cand in available:
            plt.rcParams["font.family"] = cand
            plt.rcParams["axes.unicode_minus"] = False
            return cand
    print("警告: 日本語フォントが見つかりません。図のラベルが豆腐になります。",
          file=sys.stderr)
    return None


_use_japanese_font()

sys.path.insert(0, str(Path(__file__).resolve().parent))
import firmware_consts as fc  # noqa: E402
import logger  # noqa: E402

DEG = 180.0 / np.pi
MOTORS = ("fr", "fl", "mr", "ml", "rr", "rl")


def _mark_arm_transitions(ax, t, arm):
    """アーム状態が変わった時刻に縦線を引く。"""
    if arm is None:
        return
    change = np.nonzero(np.diff(arm))[0]
    for i in change:
        ax.axvline(t[i], color="0.7", lw=0.8, ls=":", zorder=0)


def plot_overview(data, meta, out_path: Path, show=False):
    t = data["t"]
    arm = data.get("arm")

    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    fig.suptitle(
        f"SIL run — {meta.get('note') or ''}\n"
        f"throttle={meta.get('throttle')}  "
        f"T_max={meta.get('plant_params', {}).get('T_max')} N  "
        f"fw={meta.get('firmware_source_sha256', '?')}",
        fontsize=11)

    # 1. 姿勢: 真値 vs 推定 vs 目標
    ax = axes[0]
    ax.plot(t, data["phi_true"] * DEG, label="phi 真値", lw=1.4)
    ax.plot(t, data["phi_est"] * DEG, label="phi 推定(EKF)", lw=1.0, ls="--")
    ax.plot(t, data["phi_ref"] * DEG, label="phi 目標", lw=0.9, ls=":", color="k")
    ax.plot(t, data["theta_true"] * DEG, label="theta 真値", lw=1.4)
    ax.plot(t, data["theta_est"] * DEG, label="theta 推定(EKF)", lw=1.0, ls="--")
    ax.plot(t, data["theta_ref"] * DEG, label="theta 目標", lw=0.9, ls=":", color="0.4")
    ax.set_ylabel("角度 [deg]")
    ax.legend(ncol=3, fontsize=8)
    ax.grid(alpha=0.3)
    _mark_arm_transitions(ax, t, arm)

    # 2. 推定誤差（真値 - 推定）
    ax = axes[1]
    ax.plot(t, (data["phi_true"] - data["phi_est"]) * DEG, label="phi 誤差", lw=1.0)
    ax.plot(t, (data["theta_true"] - data["theta_est"]) * DEG, label="theta 誤差", lw=1.0)
    ax.axhline(0, color="k", lw=0.6)
    ax.set_ylabel("EKF 誤差 [deg]")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # 3. 角速度と目標レート
    ax = axes[2]
    for k, lbl in (("p_true", "p"), ("q_true", "q"), ("r_true", "r")):
        ax.plot(t, data[k] * DEG, label=lbl, lw=1.1)
    for k, lbl in (("Pref", "Pref"), ("Qref", "Qref"), ("Rref", "Rref")):
        if k in data:
            ax.plot(t, data[k] * DEG, label=lbl, lw=0.8, ls="--")
    ax.set_ylabel("角速度 [deg/s]")
    ax.legend(ncol=3, fontsize=8)
    ax.grid(alpha=0.3)

    # 4. 6 モータ duty
    ax = axes[3]
    for m in MOTORS:
        key = f"duty_{m}"
        if key in data:
            ax.plot(t, data[key], label=m.upper(), lw=1.0)
    ax.axhline(fc.MIN_DUTY, color="r", lw=0.7, ls=":", label=f"min {fc.MIN_DUTY}")
    ax.axhline(fc.MAX_DUTY, color="r", lw=0.7, ls="--", label=f"max {fc.MAX_DUTY}")
    ax.set_ylabel("duty")
    ax.legend(ncol=4, fontsize=8)
    ax.grid(alpha=0.3)

    # 5. 推力指令と高度
    ax = axes[4]
    ax.plot(t, data["T_ref"], label="T_ref", lw=1.2, color="C3")
    ax.axhline(fc.DISABLE_DUTY * fc.BATTERY_VOLTAGE, color="r", lw=0.7, ls=":",
               label="Disable_duty 閾値")
    ax.set_ylabel("T_ref")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.3)
    if "alt" in data:
        ax2 = ax.twinx()
        ax2.plot(t, data["alt"], color="C4", lw=1.0, alpha=0.8, label="高度(開ループ)")
        alt_max = float(np.nanmax(data["alt"]))
        note = "開ループ・制御に未フィードバック"
        if alt_max > 100.0:
            note += " / 発散: T_max 較正前"
        ax2.set_ylabel(f"高度 [m]\n({note})", fontsize=8)
        ax2.legend(loc="upper right", fontsize=8)
    ax.set_xlabel("時刻 [s]")

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_path, dpi=120)
    if show:
        plt.show()
    plt.close(fig)
    return out_path


def plot_saturation(data, out_path: Path):
    """duty の飽和率。M5 の最悪ケース評価で効く指標。"""
    fig, ax = plt.subplots(figsize=(9, 4))
    names, lo, hi = [], [], []
    for m in MOTORS:
        key = f"duty_{m}"
        if key not in data:
            continue
        d = data[key]
        names.append(m.upper())
        lo.append(100.0 * np.mean(d <= fc.MIN_DUTY + 1e-6))
        hi.append(100.0 * np.mean(d >= fc.MAX_DUTY - 1e-6))
    x = np.arange(len(names))
    ax.bar(x - 0.2, lo, 0.4, label=f"下限({fc.MIN_DUTY}) 張り付き")
    ax.bar(x + 0.2, hi, 0.4, label=f"上限({fc.MAX_DUTY}) 張り付き")
    ax.set_xticks(x); ax.set_xticklabels(names)
    ax.set_ylabel("時間比率 [%]")
    ax.set_title("モータ duty の飽和率")
    ax.legend(); ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def plot_trajectory3d(data, out_path: Path):
    """開ループ積分による見かけの軌跡（制御にはフィードバックしていない）。"""
    if not {"pos_n", "pos_e", "alt"} <= set(data):
        return None
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(data["pos_e"], data["pos_n"], data["alt"], lw=1.2)
    ax.scatter([data["pos_e"][0]], [data["pos_n"][0]], [data["alt"][0]],
               color="g", s=40, label="開始")
    ax.scatter([data["pos_e"][-1]], [data["pos_n"][-1]], [data["alt"][-1]],
               color="r", s=40, label="終了")
    ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]"); ax.set_zlabel("高度 [m]")
    ax.set_title("軌跡（開ループ積分・参考値）")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


# --- 自動機動の解析 -------------------------------------------------------
MANEUVER_NAMES = {20: "自動水平旋回", 21: "自動 8 の字", 22: "自動上昇旋回"}


def find_maneuvers(data):
    """mode 列から自動機動の区間 [(mode, i0, i1), ...] を切り出す。"""
    if "mode" not in data:
        return []
    m = np.round(data["mode"]).astype(int)
    segs, cur, start = [], 0, 0
    for i, v in enumerate(m):
        if v != cur:
            if cur in MANEUVER_NAMES:
                segs.append((cur, start, i))
            cur, start = v, i
    if cur in MANEUVER_NAMES:
        segs.append((cur, start, len(m)))
    return segs


def plot_maneuvers(data, meta, out_path):
    """機動ごとに「指令 vs 実測」を並べる。M5 の要点が一目で分かる形にする。"""
    segs = find_maneuvers(data)
    if not segs:
        return None
    t = data["t"]
    fig, axes = plt.subplots(3, len(segs), figsize=(6.0 * len(segs), 11), squeeze=False)

    for c, (mode, i0, i1) in enumerate(segs):
        sl = slice(i0, i1)
        tt = t[sl] - t[i0]
        name = MANEUVER_NAMES[mode]

        # 1段目: ロール/ピッチ 指令 vs 実測
        ax = axes[0][c]
        ax.plot(tt, data["phi_ref"][sl] * DEG, "k:", lw=1.4, label="ロール 目標")
        ax.plot(tt, data["phi_true"][sl] * DEG, lw=1.8, color="C0", label="ロール 実測")
        ax.plot(tt, data["theta_ref"][sl] * DEG, ":", color="0.5", lw=1.4, label="ピッチ 目標")
        ax.plot(tt, data["theta_true"][sl] * DEG, lw=1.8, color="C3", label="ピッチ 実測")
        # 定常区間のバンク達成率を注記
        rref = data["Rref"][sl]
        hold = np.abs(rref) > 0.9 * np.max(np.abs(rref)) if np.max(np.abs(rref)) > 0 else None
        if hold is not None and hold.any():
            ref_h = np.mean(np.abs(data["phi_ref"][sl][hold])) * DEG
            act_h = np.mean(np.abs(data["phi_true"][sl][hold])) * DEG
            if ref_h > 1e-3:
                ax.set_title(f"{name}\nバンク達成率 {100*act_h/ref_h:.0f} % "
                             f"（指令 {ref_h:.1f}° → 実測 {act_h:.1f}°）", fontsize=11)
        else:
            ax.set_title(name, fontsize=11)
        ax.set_ylabel("角度 [deg]")
        ax.legend(fontsize=8, ncol=2)
        ax.grid(alpha=0.3)

        # 2段目: ヨー角速度と積算方位
        ax = axes[1][c]
        ax.plot(tt, data["Rref"][sl] * DEG, "k:", lw=1.4, label="ヨー角速度 目標")
        ax.plot(tt, data["r_true"][sl] * DEG, lw=1.6, color="C2", label="ヨー角速度 実測")
        ax.set_ylabel("ヨー角速度 [deg/s]")
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(alpha=0.3)
        ax2 = ax.twinx()
        psi_rel = (data["psi_true"][sl] - data["psi_true"][i0]) * DEG
        ax2.plot(tt, psi_rel, color="C1", lw=1.6, alpha=0.9, label="積算方位")
        for lvl in (360, -360):
            if np.nanmin(psi_rel) - 20 < lvl < np.nanmax(psi_rel) + 20:
                ax2.axhline(lvl, color="C1", lw=0.8, ls="--", alpha=0.6)
        ax2.set_ylabel("積算方位 [deg]")
        ax2.legend(loc="lower right", fontsize=8)

        # 3段目: duty と推力指令
        ax = axes[2][c]
        for m_ in MOTORS:
            k = f"duty_{m_}"
            if k in data:
                ax.plot(tt, data[k][sl], lw=1.0, label=m_.upper())
        ax.axhline(fc.MAX_DUTY, color="r", lw=0.7, ls="--")
        ax.axhline(fc.MIN_DUTY, color="r", lw=0.7, ls=":")
        ax.set_ylabel("duty")
        ax.set_xlabel("機動開始からの時刻 [s]")
        ax.legend(ncol=6, fontsize=7)
        ax.grid(alpha=0.3)
        ax3 = ax.twinx()
        ax3.plot(tt, data["T_ref"][sl], color="C3", lw=1.4, alpha=0.8)
        ax3.set_ylabel("T_ref", color="C3")

    fig.suptitle("自動機動の指令と実測", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def maneuver_report(data):
    """機動ごとの数値サマリ。"""
    segs = find_maneuvers(data)
    if not segs:
        return "自動機動の区間が見つかりません"
    t = data["t"]
    out = []
    for mode, i0, i1 in segs:
        sl = slice(i0, i1)
        name = MANEUVER_NAMES[mode]
        dur = t[i1 - 1] - t[i0]
        psi_rel = (data["psi_true"][sl] - data["psi_true"][i0]) * DEG
        rref = data["Rref"][sl]
        peak = np.max(np.abs(rref)) 
        hold = np.abs(rref) > 0.9 * peak if peak > 0 else np.zeros(len(rref), bool)
        ref_h = np.mean(np.abs(data["phi_ref"][sl][hold])) * DEG if hold.any() else float("nan")
        act_h = np.mean(np.abs(data["phi_true"][sl][hold])) * DEG if hold.any() else float("nan")
        th_h = np.mean(data["theta_true"][sl][hold]) * DEG if hold.any() else float("nan")
        tr = data["T_ref"][sl]
        boost = (np.max(tr) - tr[0]) / tr[0] * 100 if tr[0] > 1e-6 else 0.0
        dmax = max(np.max(data[f"duty_{m_}"][sl]) for m_ in MOTORS if f"duty_{m_}" in data)
        out.append(
            f"■ {name}   所要 {dur:.1f} s\n"
            f"    バンク    指令 {ref_h:5.1f}°  実測 {act_h:5.1f}°  達成率 {100*act_h/ref_h:3.0f} %\n"
            f"    ピッチ    定常 {th_h:+5.1f}°\n"
            f"    ヨー      最大 {np.max(psi_rel):+7.1f}°  最小 {np.min(psi_rel):+7.1f}°  正味 {psi_rel[-1]:+7.1f}°\n"
            f"    推力補正  最大 +{boost:.1f} %\n"
            f"    duty      最大 {dmax:.3f}（上限 {fc.MAX_DUTY}）")
    return "\n".join(out)


def summarize(data, meta):
    """数値サマリ。目で見る前にここで異常に気づけるようにする。"""
    t = data["t"]
    flying = data["arm"] >= 2 if "arm" in data else np.ones_like(t, dtype=bool)
    out = [f"期間 {t[0]:.2f} 〜 {t[-1]:.2f} s  ({len(t)} 行, {len(t)/max(t[-1]-t[0],1e-9):.0f} Hz)"]
    for name, key in (("phi", "phi_true"), ("theta", "theta_true")):
        v = data[key][flying] * DEG
        if v.size:
            out.append(f"  {name:6s} 平均 {v.mean():+7.3f}  最大|.| {np.abs(v).max():6.3f} deg")
    for name in ("phi", "theta"):
        e = (data[f"{name}_true"] - data[f"{name}_est"])[flying] * DEG
        if e.size:
            out.append(f"  {name:6s} EKF誤差 平均|.| {np.abs(e).mean():.4f}  最大 {np.abs(e).max():.4f} deg")
    sat_hi = [100.0 * np.mean(data[f"duty_{m}"][flying] >= fc.MAX_DUTY - 1e-6)
              for m in MOTORS if f"duty_{m}" in data]
    if sat_hi:
        out.append(f"  duty 上限張り付き 最大 {max(sat_hi):.2f} %")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", nargs="?", default=None)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    run_dir = Path(args.run_dir) if args.run_dir else logger.latest_run()
    data, meta = logger.load(run_dir)
    print(f"=== {run_dir} ===")
    print(summarize(data, meta))

    made = [plot_overview(data, meta, run_dir / "overview.png", show=args.show),
            plot_saturation(data, run_dir / "saturation.png")]
    traj = plot_trajectory3d(data, run_dir / "trajectory.png")
    if traj:
        made.append(traj)
    mv = plot_maneuvers(data, meta, run_dir / "maneuvers.png")
    if mv:
        made.append(mv)
        print()
        print(maneuver_report(data))
    print("\n生成:")
    for p in made:
        print(f"  {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
