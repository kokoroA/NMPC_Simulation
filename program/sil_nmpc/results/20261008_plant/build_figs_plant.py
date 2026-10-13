"""プラント作り込みの資料（docs/20261008_plant_6dof_report.md）用の図。

    <matplotlib のある python> build_figs_plant.py <出力ディレクトリ>
"""
import math, os, sys
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
for fam in ('Hiragino Sans', 'Hiragino Maru Gothic Pro', 'AppleGothic'):
    try:
        matplotlib.font_manager.findfont(fam, fallback_to_default=False); plt.rcParams['font.family'] = fam; break
    except Exception:
        pass
plt.rcParams['axes.unicode_minus'] = False

from metrics_plant import load, all_runs, group, real_metrics, LADDER, HERE

OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
C_BLUE, C_ORANGE, C_AQUA, C_YELLOW, C_MAG, C_VIOLET, C_RED = '#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#4a3aa7', '#e34948'
INK2 = '#52514e'
LCOL = {'old': '#8a8986', 'legacy': '#b9b8b4', 'sf_nodrag': C_YELLOW, 'sf': C_AQUA, 'bf': C_BLUE, 'motor': C_VIOLET,
        'full': C_ORANGE, 'wind': C_RED}
LNAME = {'old': '旧プラント', 'legacy': '6自由度\n(旧と同扱い)', 'sf_nodrag': '比力\n抗力なし*', 'sf': '+比力\n+抗力',
         'bf': '+ブロー\nバック', 'motor': '+回転数\nモデル', 'full': '+地面効果\n+軸流', 'wind': '+風\n1.5 m/s'}


def save(fig, name):
    fig.savefig(os.path.join(OUT, name), dpi=130, bbox_inches='tight'); plt.close(fig)


def shade(ax):
    ax.axvspan(7, 9, color=C_BLUE, alpha=0.06); ax.axvspan(11, 13, color=C_ORANGE, alpha=0.06)


rows = all_runs(); G = group(rows)

# ---------- 図1: 理想条件のロール/ピッチ（真値・推定・参照）と横速度 ----------
fig, axs = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
for pl in ('old', 'sf_nodrag', 'sf', 'full'):
    path = os.path.join(HERE, f'{pl}_ideal_s0.npz')
    if not os.path.exists(path): continue
    d, L, c, P, pc, tP, t, idx = load(path)
    col = LCOL[pl]; nm = LNAME[pl].replace('\n', '')
    axs[0].plot(tP, np.degrees(P[:, pc['phi']]), color=col, lw=1.6, label=f'{nm} 真値')
    axs[0].plot(t, np.degrees(L[:, c['phi']]), color=col, lw=1, ls='--', label=f'{nm} EKF 推定')
    axs[1].plot(tP, np.degrees(P[:, pc['theta']]), color=col, lw=1.6)
    axs[1].plot(t, np.degrees(L[:, c['theta']]), color=col, lw=1, ls='--')
    if 've' in pc:
        axs[2].plot(tP, P[:, pc['ve']], color=col, lw=1.4, label=nm)
        axs[3].plot(tP, P[:, pc['fy']], color=col, lw=1.2, label=nm)
d, L, c, *_ = load(os.path.join(HERE, 'old_ideal_s0.npz')); t = np.arange(len(L)) * 0.01
axs[0].plot(t, np.degrees(L[:, c['phi_ref']]), 'k:', lw=1, label='参照'); axs[1].plot(t, np.degrees(L[:, c['theta_ref']]), 'k:', lw=1)
axs[0].set_ylabel('ロール φ [deg]'); axs[0].set_ylim(-12, 25); axs[0].legend(fontsize=7, ncol=3, loc='upper right')
axs[1].set_ylabel('ピッチ θ [deg]'); axs[1].set_ylim(-8, 16)
axs[2].set_ylabel('東向き速度 [m/s]'); axs[2].set_ylim(-2, 10); axs[2].legend(fontsize=8)
axs[3].set_ylabel('加速度計 y（比力）[m/s²]'); axs[3].set_xlabel('LOG 開始からの時間 [s]')
for ax in axs: shade(ax)
fig.suptitle('図1  理想条件（雑音なし）のロール 15°・ピッチ 10° ステップ: 実線 = 真値、破線 = ファームの EKF 推定\n'
             '比力を入れると傾いて加速する間 EKF が「水平寄り」に引っ張られ（偽の傾き）、ブローバックを入れると傾きが戻される（垂れ）。'
             '\n* 抗力なしは説明用の極端（非物理）', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.93]); save(fig, 'fig01_ideal_step_truth_vs_estimate.png')

# ---------- 図2: 段階ごとの指標（実機並み条件 6 seed） ----------
items = [('est_err_rms', 'EKF 推定 − 真値 rms [deg]'), ('true_err_max', '真の姿勢誤差の最大 [deg]'),
         ('roll_true_17', 'ロール 15° を 2 s 保った末の真値 / 参照'), ('drift_m', '19.5 s 後の水平移動 [m]'),
         ('du_max', '1 周期の |Δduty| 最大'), ('wrong', '戻す向きと逆の指令の割合')]
fig, axs = plt.subplots(2, 3, figsize=(17, 9))
keys = [p for p in LADDER if f'{p}_real' in G]
for ax, (k, lab) in zip(axs.flat, items):
    vals = [np.array([r[k] for r in G[f'{p}_real']], float) for p in keys]
    for i, (p, v) in enumerate(zip(keys, vals)):
        ax.bar(i, np.nanmean(v), color=LCOL[p], alpha=0.85)
        ax.scatter(np.full(v.size, i) + np.linspace(-0.2, 0.2, v.size), v, s=10, color='k', zorder=3)
        nc = sum(bool(r['crash']) for r in G[f'{p}_real'])
        if nc and k == 'true_err_max':
            ax.text(i, np.nanmax(v), f'墜落 {nc}', ha='center', va='bottom', color=C_RED, fontsize=8)
    ax.set_xticks(range(len(keys))); ax.set_xticklabels([LNAME[p] for p in keys], fontsize=7); ax.set_title(lab, fontsize=10)
fig.suptitle('図2  プラントの要素を 1 つずつ足したときの SIL 結果（実機ファーム、実機並み条件 6 seed。棒 = 平均、点 = 各 seed）', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.95]); save(fig, 'fig02_ladder_metrics.png')

# ---------- 図3: ブローバックの強さ ----------
scales = [('bfx0', 0.0), ('bfx0.25', 0.25), ('bfx0.5', 0.5), ('full', 1.0), ('bfx2', 2.0)]
fig, axs = plt.subplots(1, 2, figsize=(14, 5))
for ax, cond in zip(axs, ('ideal', 'real')):
    xs, r07, r17, p07, p17 = [], [], [], [], []
    for nm, k in scales:
        R = G.get(f'{nm}_{cond}')
        if not R: continue
        xs.append(k)
        r07.append(np.nanmean([r['roll_true_07'] for r in R])); r17.append(np.nanmean([r['roll_true_17'] for r in R]))
        p07.append(np.nanmean([r['pitch_true_07'] for r in R])); p17.append(np.nanmean([r['pitch_true_17'] for r in R]))
    ax.plot(xs, r07, 'o-', color=C_BLUE, label='ロール 0.7–1.0 s'); ax.plot(xs, r17, 'o--', color=C_BLUE, label='ロール 1.7–2.0 s')
    ax.plot(xs, p07, 's-', color=C_ORANGE, label='ピッチ 0.7–1.0 s'); ax.plot(xs, p17, 's--', color=C_ORANGE, label='ピッチ 1.7–2.0 s')
    ax.axhspan(0.47, 0.78, color=C_RED, alpha=0.08); ax.text(xs[-1], 0.80, '実機 v6 ピッチ 0.7–1.0 s: 47–78 %', ha='right', color=C_RED, fontsize=8)
    ax.axhline(1.0, color=INK2, lw=0.5)
    ax.set_xlabel('ブローバック係数の倍率（1 = 推定値 roll 7.5e-3 / pitch 2.8e-3 N·m/(m/s)）'); ax.set_ylabel('真の姿勢 / 参照')
    ax.set_title({'ideal': '理想条件', 'real': '実機並み条件（3 seed 平均）'}[cond], fontsize=10); ax.legend(fontsize=8)
fig.suptitle('図3  ブローバックの強さと「垂れ」: 係数は未同定なので倍率を振った（ステップ開始からの時間で区切った真の姿勢の平均）', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.93]); save(fig, 'fig03_blowback_sweep.png')

# ---------- 図4: 水平軌跡（理想条件） ----------
fig, ax = plt.subplots(1, 1, figsize=(8, 7))
for pl in ('legacy', 'sf_nodrag', 'sf', 'bf', 'full', 'wind'):
    path = os.path.join(HERE, f'{pl}_ideal_s0.npz')
    if not os.path.exists(path): continue
    d, L, c, P, pc, *_ = load(path)
    ax.plot(P[:, pc['pe']], P[:, pc['pn']], color=LCOL[pl], label=LNAME[pl].replace('\n', ''))
    ax.plot(P[-1, pc['pe']], P[-1, pc['pn']], 'o', color=LCOL[pl])
ax.set_xlabel('東 [m]'); ax.set_ylabel('北 [m]'); ax.set_aspect('equal', 'datalim'); ax.legend(fontsize=8); ax.grid(alpha=0.3)
ax.set_title('図4  水平軌跡（理想条件、0–19.5 s）\n旧プラントは位置を制御に戻さないので軌跡は飾り。比力・抗力から位置が姿勢へ効くようになった', fontsize=10)
save(fig, 'fig04_trajectories.png')

# ---------- 図5: 実機ログとの比較 ----------
real = real_metrics()
items = [('acc_std', '加速度ノルムの標準偏差 [m/s²]'), ('p_rms', 'p rms [deg/s]'), ('path2', 'TR1 棄却の割合'),
         ('du_rms', '|Δduty| rms'), ('du_max', '|Δduty| 最大'), ('wrong', '逆向き指令の割合')]
fig, axs = plt.subplots(2, 3, figsize=(17, 8))
groups = [(f'実機\n{r["name"][5:]}', [r], '#52514e') for r in real] + \
         [(LNAME[p].replace('\n', '') + '\n(SIL)', G.get(f'{p}_real', []), LCOL[p]) for p in ('old', 'sf', 'full', 'wind')]
for ax, (k, lab) in zip(axs.flat, items):
    for i, (nm, R, col) in enumerate(groups):
        if not R: continue
        ax.bar(i, np.nanmean([r[k] for r in R]), color=col)
    ax.set_xticks(range(len(groups))); ax.set_xticklabels([g[0] for g in groups], fontsize=7); ax.set_title(lab, fontsize=10)
fig.suptitle('図5  実機ログ（巡航区間）と SIL（実機並み条件 6 seed 平均）: プラントを作り込んでも、指令の荒れ・逆向き指令・TR1 棄却率の実機とのずれは埋まらない\n（加速度ノルムのばらつきは SIL では加えた 60 mg の雑音が大半を占める）', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.92]); save(fig, 'fig05_sil_vs_real.png')

# ---------- 図6: 風（実機並み条件、seed 0） ----------
path = os.path.join(HERE, 'wind_real_s0.npz')
if os.path.exists(path):
    d, L, c, P, pc, tP, t, idx = load(path)
    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axs[0].plot(tP, np.degrees(P[:, pc['phi']]), color=C_BLUE, label='φ 真値'); axs[0].plot(t, np.degrees(L[:, c['phi']]), '--', color=C_BLUE, lw=1, label='φ 推定')
    axs[0].plot(tP, np.degrees(P[:, pc['theta']]), color=C_ORANGE, label='θ 真値'); axs[0].plot(t, np.degrees(L[:, c['theta']]), '--', color=C_ORANGE, lw=1, label='θ 推定')
    axs[0].set_ylabel('[deg]'); axs[0].legend(fontsize=8, ncol=4)
    axs[1].plot(tP, P[:, pc['wind_n']], color=INK2, label='風 北向き'); axs[1].plot(tP, P[:, pc['vn']], color=C_VIOLET, label='機体 北向き速度')
    axs[1].set_ylabel('[m/s]'); axs[1].legend(fontsize=8)
    axs[2].plot(tP, P[:, pc['pn']], color=C_VIOLET, label='北'); axs[2].plot(tP, P[:, pc['pe']], color=C_AQUA, label='東')
    axs[2].set_ylabel('位置 [m]'); axs[2].legend(fontsize=8); axs[2].set_xlabel('LOG 開始からの時間 [s]')
    for ax in axs: shade(ax)
    fig.suptitle('図6  風 1.5 m/s（北へ吹く）+ 突風 0.5 m/s、実機並み条件 seed 0（6 本中この 1 本が墜落）: 位置制御が無いので風に流され、\n対気速度によるブローバックとヨーの振れで傾きが 24° まで育ち、推定は真値より 4〜7° 浅く、高度を失って接地', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.93]); save(fig, 'fig06_wind.png')
# ---------- 図7: Python シミュ（PKG-E10）----------
AP = os.path.join(HERE, '..', '..', '..', 'NMPC_simulation', 'img', '20261008_aero', 'aero_compare.npz')
if os.path.exists(AP):
    Z = np.load(AP)
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    for nm, col, lab in (('off', '#8a8986', '空力なし（従来）'), ('full_bf0', C_AQUA, '抗力のみ'),
                         ('full_bf05', C_BLUE, '抗力 + ブローバック ×0.5'), ('full', C_ORANGE, '抗力 + ブローバック ×1')):
        t = Z[f'{nm}_s42__t']
        axs[0].plot(t, np.degrees(Z[f'{nm}_s42__phi']), color=col, label=lab)
        axs[1].plot(t, np.degrees(Z[f'{nm}_s42__theta']), color=col)
        axs[2].plot(t, np.hypot(Z[f'{nm}_s42__vx'], Z[f'{nm}_s42__vy']), color=col)
    for ax in axs[:2]:
        ax.axhline(0, color=INK2, lw=0.5)
    axs[0].plot([5, 5, 7, 7], [0, 15, 15, 0], 'k:', lw=1); axs[1].plot([9, 9, 11, 11], [0, 10, 10, 0], 'k:', lw=1)
    axs[0].set_ylabel('φ 真値 [deg]'); axs[0].legend(fontsize=8); axs[1].set_ylabel('θ 真値 [deg]')
    axs[2].set_ylabel('水平速度 [m/s]'); axs[2].set_xlabel('時間 [s]')
    fig.suptitle('図7  Python シミュ（stress 版 R1・DC6、seed 42）に同じ空力を入れた結果: SIL と同じ「垂れ」が出る（EKF が無いので偽の傾きは出ない）', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.95]); save(fig, 'fig07_python_sim_aero.png')
# ---------- 図8: 前回の結論（再構築 10 周期）が作り込んだプラントでも成り立つか ----------
import glob as _g
from metrics_plant import run_metrics as _rm
A = [_rm(p) for p in sorted(_g.glob(os.path.join(HERE, 'full_real_s*.npz')))]
B = [_rm(p) for p in sorted(_g.glob(os.path.join(HERE, 'rp10', 'full_real_s*.npz')))]
if A and B:
    items = [('du_max', '|Δduty| 最大'), ('du_p99', '|Δduty| p99'), ('wrong', '逆向き指令の割合'),
             ('true_err_max', '真の姿勢誤差の最大 [deg]'), ('roll_true_17', 'ロール 2 s 後の真値/参照（垂れ）')]
    fig, axs = plt.subplots(1, 5, figsize=(18, 4.2))
    for ax, (k, lab) in zip(axs, items):
        for i, (R, col, nm) in enumerate(((A, C_ORANGE, '100 周期\n(実機)'), (B, C_AQUA, '10 周期'))):
            v = np.array([r[k] for r in R], float)
            ax.bar(i, np.nanmean(v), color=col); ax.scatter(np.full(v.size, i), v, s=10, color='k', zorder=3)
        ax.set_xticks([0, 1]); ax.set_xticklabels(['100 周期\n(実機)', '10 周期']); ax.set_title(lab, fontsize=10)
    fig.suptitle('図8  作り込んだプラント（full・実機並み条件 6 seed）でも、再構築 10 周期は指令の荒れ・逆向き指令を消す。ただし「垂れ」（物理）は変わらない', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.9]); save(fig, 'fig08_rebuild_period_with_full_plant.png')
print('done', sorted(os.listdir(OUT)))
