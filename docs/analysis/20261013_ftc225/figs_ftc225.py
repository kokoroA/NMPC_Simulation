"""225 g FTC の SIL 評価の図（docs/20261013_ftc_225g_sil_eval.md 用）。

    <matplotlib のある python> figs_ftc225.py <出力dir>

入力: program/sil_nmpc/results/20261013_ftc225/runs/*.npz（集計関数は同じ場所の analyze_ftc.py）
"""
import glob, math, os, sys
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
for fam in ('Hiragino Sans', 'Hiragino Maru Gothic Pro', 'AppleGothic'):
    try:
        matplotlib.font_manager.findfont(fam, fallback_to_default=False); plt.rcParams['font.family'] = fam; break
    except Exception:
        pass
plt.rcParams['axes.unicode_minus'] = False
HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.normpath(os.path.join(HERE, '../../../program/sil_nmpc/results/20261013_ftc225'))
sys.path.insert(0, RES)
from analyze_ftc import one, cmd_stats, MOTORS, DIAG  # noqa: E402
OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
RUNS = os.path.join(RES, 'runs')
# 色は制御器（エンティティ）に固定。validate_palette.js で light 全項目 PASS（aqua/yellow はコントラスト WARN → 値を文字でも出す）
COL = {'base': '#2a78d6', 'qr2': '#eb6834', 'qr0': '#1baf7a', 'qr10': '#eda100'}
LAB = {'base': '現行（Q_r=40 のまま）', 'qr10': 'FTC 中 Q_r=10', 'qr2': 'FTC 中 Q_r=2', 'qr0': 'FTC 中 Q_r=0'}
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'


def style(ax):
    ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True)
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'): ax.spines[s].set_color('#b9b8b4')
    ax.tick_params(colors=INK2, labelsize=9)


def save(fig, fn): fig.savefig(os.path.join(OUT, fn), dpi=130, bbox_inches='tight', facecolor='#fcfcfb'); plt.close(fig)


def group(prefix):
    fs = sorted(glob.glob(os.path.join(RUNS, prefix + '_s*.npz')))
    return fs, [one(f) for f in fs]


# ---- 図1: 生存マトリクス ----
rows = [('FR 空中（右前の角）', 'FR_air_m225'), ('FL 空中（左前の角）', 'FL_air_m225'),
        ('RR 空中（右後の角）', 'RR_air_m225'), ('RL 空中（左後の角）', 'RL_air_m225'),
        ('MR 空中（右中央）', 'MR_air_m225'), ('ML 空中（左中央）', 'ML_air_m225'),
        ('FR 離陸前に停止', 'FR_gnd_m225'), ('FR 慣性 真値 −20〜25 %', 'FR_air_m225_I08'),
        ('FR 検知 0.2 s 遅れ', 'FR_air_m225_dd02'), ('FR 風 1.5 m/s＋突風', 'FR_air_m225_wind'),
        ('FR ヨー物理ずれ（Cr×3.7, 反トルク×0.69）', 'FR_air_m225_yaw'),
        ('FR 250 g', 'FR_air_m250'), ('FR 271 g', 'FR_air_m271'), ('FR 285 g', 'FR_air_m285')]
cols = ['base', 'qr10', 'qr2', 'qr0']
M = np.full((len(rows), len(cols)), np.nan); N = np.zeros_like(M)
for i, (_, key) in enumerate(rows):
    for j, c in enumerate(cols):
        fs, rs = group(f'{c}_{key}')
        if rs:
            M[i, j] = sum(r['ok'] for r in rs); N[i, j] = len(rs)
fig, ax = plt.subplots(figsize=(8.6, 7.4))
seq = matplotlib.colors.LinearSegmentedColormap.from_list('b', ['#f0efec', '#86b6ef', '#2a78d6', '#184f95'])
F = np.where(N > 0, M / np.maximum(N, 1), np.nan)
ax.imshow(np.ma.masked_invalid(F), cmap=seq, vmin=0, vmax=1, aspect='auto')
for i in range(len(rows)):
    for j in range(len(cols)):
        if N[i, j] > 0:
            ax.text(j, i, f'{int(M[i, j])}/{int(N[i, j])}', ha='center', va='center', fontsize=10,
                    color='#ffffff' if F[i, j] > 0.6 else INK)
        else:
            ax.text(j, i, '—', ha='center', va='center', fontsize=9, color=INK2)
ax.set_xticks(range(len(cols))); ax.set_xticklabels([LAB[c].replace('（', '\n（') for c in cols], fontsize=9, color=INK)
ax.set_yticks(range(len(rows))); ax.set_yticklabels([r[0] for r in rows], fontsize=9, color=INK)
ax.xaxis.tick_top()
for s in ax.spines.values(): s.set_visible(False)
ax.set_xticks(np.arange(-.5, len(cols)), minor=True); ax.set_yticks(np.arange(-.5, len(rows)), minor=True)
ax.grid(which='minor', color='#fcfcfb', lw=2); ax.tick_params(which='minor', length=0); ax.tick_params(length=0)
ax.set_title('生存した本数（19.5 s を墜落・フェイルセーフなしで飛び切った / 試行数）\n特記なしは 225 g・故障は 7 s・実機並みの揺れ', fontsize=10, color=INK, pad=38)
save(fig, 'fig01_survival_matrix.png')

# ---- 図2: FR 空中 seed0 の時系列（現行 vs Q_r=2）----
fig, axs = plt.subplots(4, 1, figsize=(11, 9.5), sharex=True)
for c in ('base', 'qr2'):
    d = np.load(os.path.join(RUNS, f'{c}_FR_air_m225_s0.npz'), allow_pickle=True)
    P = d['plant']; k = {n: i for i, n in enumerate(d['plant_cols'])}
    t = P[:, k['t']] - P[0, k['t']]
    tilt = np.degrees(np.arccos(np.clip(np.cos(P[:, k['phi']]) * np.cos(P[:, k['theta']]), -1, 1)))
    carry = np.max(P[:, [k[f'd{i}'] for i in (1, 2, 3, 4)]], axis=1)
    for ax, y in zip(axs, (tilt, np.degrees(P[:, k['r']]), P[:, k['alt']], carry)):
        ax.plot(t, y, color=COL[c], lw=1.4, label=LAB[c])
    axs[0].annotate(LAB[c] + ('：墜落' if str(d['crash']) else '：飛び切った'), xy=(t[-1], tilt[-1]),
                    xytext=(6, -10 if str(d['crash']) else 0), textcoords='offset points', fontsize=8, color=INK2, va='center')
ylab = ['機体の傾き [deg]', 'ヨー率 r [deg/s]', '高度 [m]', '担当 4 発の duty 最大']
for ax, yl in zip(axs, ylab):
    style(ax); ax.set_ylabel(yl, fontsize=9, color=INK2); ax.axvline(7.0, color=INK2, lw=0.8, ls='--')
axs[1].axhline(360, color='#d03b3b', lw=0.8); axs[1].text(0.2, 340, '角速度フェイルセーフ 360 deg/s', fontsize=8, color=INK2, va='top')
axs[3].axhline(0.95, color=INK2, lw=0.8, ls=':'); axs[3].text(0.2, 0.93, '上限 0.95', fontsize=8, color=INK2, va='top')
axs[0].legend(fontsize=8, loc='upper left', frameon=False)
axs[2].text(7.1, 0.12, '7 s: FR 停止（FTC スイッチ）', fontsize=8, color=INK2)
axs[-1].set_xlabel('時間 [s]（LOG 開始が 0）', fontsize=9, color=INK2)
fig.suptitle('FR 停止・225 g・seed 0（プラント真値）', fontsize=11, color=INK)
save(fig, 'fig02_timeseries_FR.png')

# ---- 図3: 仕組み（故障モータ別の指令の向き、現行 vs Q_r=2）----
fig, axs = plt.subplots(1, 2, figsize=(13, 4.4), sharey=True)
mets = [('yaw_oppose', 'ヨー指令が自転を止める向き', '#4a3aa7'), ('wrong_pitch', 'ピッチ指令が姿勢を崩す向き', '#eb6834'),
        ('wrong_roll', 'ロール指令が姿勢を崩す向き', '#1baf7a')]
for ax, c in zip(axs, ('base', 'qr2')):
    x = np.arange(6)
    for j, (m, lab, colr) in enumerate(mets):
        v = []
        for mot in MOTORS:
            fs = sorted(glob.glob(os.path.join(RUNS, f'{c}_{mot}_air_m225_s*.npz')))
            v.append(100 * np.nanmedian([cmd_stats(f)[m] for f in fs]) if fs else np.nan)
        b = ax.bar(x + (j - 1) * 0.26, np.nan_to_num(v), 0.24, color=colr, label=lab, edgecolor='#fcfcfb', lw=1)
        for xi, vi in zip(x, v):
            if np.isfinite(vi) and vi >= 5:
                ax.text(xi + (j - 1) * 0.26, vi + 1.5, f'{vi:.0f}', ha='center', fontsize=7, color=INK2)
    surv = []
    for mot in MOTORS:
        _, rs = group(f'{c}_{mot}_air_m225')
        surv.append(f'{sum(r["ok"] for r in rs)}/{len(rs)}')
    ax.set_xticks(x); ax.set_xticklabels([f'{m}\n生存 {s}' for m, s in zip(MOTORS, surv)], fontsize=9, color=INK)
    style(ax); ax.set_title(LAB[c], fontsize=10, color=INK); ax.set_ylim(0, 100)
axs[0].set_ylabel('故障後 5 s の周期に占める割合 [%]（6 本の中央値）', fontsize=9, color=INK2)
axs[1].legend(fontsize=8, frameon=False, loc='upper right')
fig.suptitle('止まったモータ別: Q_r=40 ではヨーを止めようとして姿勢を崩す向きの指令が出る（225 g・空中）', fontsize=11, color=INK)
save(fig, 'fig03_command_direction_by_motor.png')

# ---- 図4: 質量（FR 空中）----
masses = [225, 250, 271, 285]
fig, axs = plt.subplots(1, 2, figsize=(12, 4.2))
for c in ('base', 'qr2', 'qr0'):
    ok, sat = [], []
    for m in masses:
        _, rs = group(f'{c}_FR_air_m{m}')
        ok.append(sum(r['ok'] for r in rs) if rs else np.nan)
        sat.append(100 * np.median([r['carry_sat'] for r in rs]) if rs else np.nan)
    axs[0].plot(masses, ok, '-o', color=COL[c], lw=2, ms=8, label=LAB[c], mec='#fcfcfb', mew=2)
    axs[1].plot(masses, sat, '-o', color=COL[c], lw=2, ms=8, label=LAB[c], mec='#fcfcfb', mew=2)
axs[0].set_ylabel('生存本数（6 本中）', fontsize=9, color=INK2); axs[0].set_ylim(-0.3, 6.5)
axs[1].set_ylabel('担当 4 発が 0.949 以上の時間 [%]（中央値）', fontsize=9, color=INK2)
for ax in axs:
    style(ax); ax.set_xlabel('機体質量 [g]', fontsize=9, color=INK2); ax.set_xticks(masses)
axs[0].legend(fontsize=8, frameon=False, loc='center left')
axs[0].set_title('FR 停止の生存本数（Q_r=2 と Q_r=0 は全点同じ値で線が重なる）', fontsize=10, color=INK); axs[1].set_title('推力の飽和（285 g は Q_r に関係なく飽和で落ちる）', fontsize=10, color=INK)
save(fig, 'fig04_mass_sweep.png')
print('done')
