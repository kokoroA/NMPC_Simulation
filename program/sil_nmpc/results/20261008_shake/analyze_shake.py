"""実機並みに揺れる条件（shake）の集計と図。

    <matplotlib のある python> analyze_shake.py <図の出力ディレクトリ>
"""
import csv, glob, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '20261008_plant'))
sys.path.insert(0, os.path.join(HERE, '..', '20261008'))
from metrics_plant import run_metrics, load, real_metrics   # noqa: E402
from shake_stats import sil_stats, real_stats, KEYS as SKEYS  # noqa: E402

GROUPS = [('old', '旧プラント'), ('sf', '比力+抗力'), ('full', '全部入り(full)'), ('gust', 'full+乱流\n(突風0.3)'),
          ('wind', 'full+風1.5\n+突風0.5'), ('rp10full', 'full\n再構築10周期')]
KEYS = ['est_err_rms', 'est_err_max', 'hover_true_rms', 'true_err_max', 'roll_true_17', 'roll_est_17', 'pitch_true_17',
        'drift_m', 'vh_max', 'du_max', 'du_p99', 'du_rms', 'wrong', 'path2']


def collect():
    R = {}
    for g, _ in GROUPS:
        paths = sorted(glob.glob(os.path.join(HERE, 'runs', f'{g}_s*.npz')))
        R[g] = [dict(run_metrics(p), **{f'sh_{k}': v for k, v in sil_stats(p).items()}) for p in paths]
    return R


def summary(R):
    rows = []
    for g, lab in GROUPS:
        rs = R.get(g, [])
        if not rs: continue
        row = dict(group=g, n=len(rs), crash=sum(bool(r['crash']) for r in rs))
        for k in KEYS: row[k] = float(np.nanmean([r[k] for r in rs]))
        row['true_err_max_worst'] = float(np.nanmax([r['true_err_max'] for r in rs]))
        row['est_err_max_worst'] = float(np.nanmax([r['est_err_max'] for r in rs]))
        for k in SKEYS: row['sh_' + k] = float(np.nanmean([r['sh_' + k] for r in rs]))
        rows.append(row)
    return rows


if __name__ == '__main__':
    R = collect(); S = summary(R)
    with open(os.path.join(HERE, 'shake_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(S[0])); w.writeheader()
        for r in S: w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
    for r in S:
        print(r['group'], r['n'], 'crash', r['crash'], ' '.join(f'{k}={r[k]:.3g}' for k in KEYS + ['true_err_max_worst', 'est_err_max_worst']))
        print('   shake:', ' '.join(f"{k}={r['sh_' + k]:.2f}" for k in SKEYS))
    for r in [x for g in R.values() for x in g if x['crash']]:
        print('crash', r['name'], r['crash'])
    RS = real_stats()
    for k, v in RS.items(): print('real', k, ' '.join(f'{x}={v[x]:.2f}' for x in SKEYS))
    if len(sys.argv) < 2: sys.exit()

    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for fam in ('Hiragino Sans', 'Hiragino Maru Gothic Pro', 'AppleGothic'):
        try:
            matplotlib.font_manager.findfont(fam, fallback_to_default=False); plt.rcParams['font.family'] = fam; break
        except Exception:
            pass
    plt.rcParams['axes.unicode_minus'] = False
    OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
    COL = {'old': '#8a8986', 'sf': '#1baf7a', 'full': '#eb6834', 'gust': '#e87ba4', 'wind': '#e34948', 'rp10full': '#2a78d6'}
    REALC = '#52514e'

    # 図9: 揺れの較正（実機 3 便 vs SIL）
    items = [('p_rms', 'p rms'), ('q_rms', 'q rms'), ('r_rms', 'r rms'), ('p_0.2_2', 'p 0.2–2 Hz'), ('p_2_5', 'p 2–5 Hz'),
             ('p_5_15', 'p 5–15 Hz'), ('p_15_50', 'p 15–50 Hz'), ('acc_std', '加速度ノルム std [m/s²]'),
             ('ekf_dphi_rms', 'EKF 補正 ロール'), ('ekf_dth_rms', 'EKF 補正 ピッチ')]
    old_prev = [sil_stats(p) for p in sorted(glob.glob(os.path.join(HERE, '..', '20261008_plant', 'full_real_s*.npz')))]
    fig, axs = plt.subplots(2, 5, figsize=(19, 7.5))
    for ax, (k, lab) in zip(axs.flat, items):
        vals = [RS[v][k] for v in RS]
        ax.bar(range(3), vals, color=REALC)
        ax.bar(3, np.nanmean([s[k] for s in old_prev]), color='#b9b8b4')
        ax.bar(4, np.nanmean([r['sh_' + k] for r in R['old']]), color=COL['old'])
        ax.bar(5, np.nanmean([r['sh_' + k] for r in R['full']]), color=COL['full'])
        ax.axhspan(min(vals), max(vals), color=REALC, alpha=0.08)
        ax.set_xticks(range(6)); ax.set_xticklabels(['実機\nv6', '実機\nv8', '実機\nv5', 'full\n前回雑音', '旧\nshake', 'full\nshake'], fontsize=7)
        ax.set_title(lab + ('' if 'std' in k else ' [deg/s]'), fontsize=9)
    fig.suptitle('図9  揺れの較正: 静かな区間（|参照| < 5°）の角速度の大きさ・帯域、加速度ノルムのばらつき、EKF の観測補正量\n'
                 '「full shake」が今回の実機並み条件。EKF 補正量は旧プラントだと実機の 1/4 しか出ず、比力を入れると実機に並ぶ', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.9]); fig.savefig(os.path.join(OUT, 'fig09_shake_calibration.png'), dpi=130, bbox_inches='tight'); plt.close(fig)

    # 図10: shake 条件での比較
    items = [('est_err_rms', 'EKF 推定 − 真値 rms [deg]'), ('true_err_max', '真の姿勢誤差の最大 [deg]'),
             ('roll_true_17', 'ロール 15° 2 s 後 真値/参照'), ('drift_m', '水平移動 [m]'),
             ('du_max', '|Δduty| 最大'), ('wrong', '逆向き指令の割合')]
    fig, axs = plt.subplots(2, 3, figsize=(17, 9))
    gl = [g for g, _ in GROUPS if R.get(g)]
    for ax, (k, lab) in zip(axs.flat, items):
        for i, g in enumerate(gl):
            v = np.array([r[k] for r in R[g]], float)
            ax.bar(i, np.nanmean(v), color=COL[g], alpha=0.85)
            ax.scatter(np.full(v.size, i) + np.linspace(-0.25, 0.25, v.size), v, s=8, color='k', zorder=3)
            nc = sum(bool(r['crash']) for r in R[g])
            if k == 'true_err_max':
                ax.text(i, np.nanmax(v), f'墜落 {nc}/{len(v)}', ha='center', va='bottom', color='#e34948' if nc else REALC, fontsize=8)
        ax.set_xticks(range(len(gl))); ax.set_xticklabels([dict(GROUPS)[g] for g in gl], fontsize=8); ax.set_title(lab, fontsize=10)
    fig.suptitle('図10  実機並みに揺れる条件（shake）で 12 本ずつ: 並進（比力・抗力・ブローバック・風）と、ソルバ（再構築 10 周期）の効き', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.95]); fig.savefig(os.path.join(OUT, 'fig10_shake_comparison.png'), dpi=130, bbox_inches='tight'); plt.close(fig)

    # 図11: 推定誤差の時系列（full shake の代表 1 本）と墜落した本
    crashes = [x for g in ('full', 'gust', 'wind', 'sf', 'old', 'rp10full') for x in R.get(g, []) if x['crash']]
    picks = [os.path.join(HERE, 'runs', 'full_s0.npz')] + [os.path.join(HERE, 'runs', c['name'] + '.npz') for c in crashes[:2]]
    fig, axs = plt.subplots(len(picks), 1, figsize=(12, 3.4 * len(picks)), sharex=True)
    axs = np.atleast_1d(axs)
    for ax, p in zip(axs, picks):
        d, L, c, P, pc, tP, t, idx = load(p)
        ax.plot(tP, np.degrees(P[:, pc['phi']]), color='#2a78d6', label='φ 真値'); ax.plot(t, np.degrees(L[:, c['phi']]), '--', color='#2a78d6', lw=1, label='φ 推定')
        ax.plot(tP, np.degrees(P[:, pc['theta']]), color='#eb6834', label='θ 真値'); ax.plot(t, np.degrees(L[:, c['theta']]), '--', color='#eb6834', lw=1, label='θ 推定')
        ax.plot(t, np.degrees(L[:, c['phi_ref']]), 'k:', lw=1, label='φ 参照')
        ax.set_ylabel('[deg]'); ax.set_ylim(-40, 40)
        ax.set_title(f"{os.path.basename(p)[:-4]}  {('墜落 ' + str(d['crash'])) if str(d['crash']) else '完走'}", fontsize=10)
        ax.axvspan(7, 9, color='#2a78d6', alpha=0.05); ax.axvspan(11, 13, color='#eb6834', alpha=0.05)
    axs[0].legend(fontsize=8, ncol=5); axs[-1].set_xlabel('LOG 開始からの時間 [s]')
    fig.suptitle('図11  実機並み条件の時系列（実線 = 真値、破線 = ファームの推定）: 墜落した本でも崩れ始めは推定が真値に 1° 以内で追従しており、崩れはソルバの逆向き指令', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.96]); fig.savefig(os.path.join(OUT, 'fig11_shake_timeseries.png'), dpi=130, bbox_inches='tight'); plt.close(fig)
    print('figs', sorted(os.listdir(OUT)))
