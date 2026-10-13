"""Su 毎周期厳密化の解説資料（docs/20261009_su_exact_explained.md）の図。

    <matplotlib のある python> figs_su_exact.py <出力dir>

入力: program/sil_nmpc/results/20261009_su_diag/d{0,1}_s{0,1,2}.csv
  （ファーム複製に su_diag_hook.patch を当てた SIL。d0 = TR1/SR2（NMPC_SU_EXACT=0）、d1 = Su 毎周期厳密）
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
SRC = os.path.normpath(os.path.join(HERE, '../../../program/sil_nmpc/results/20261009_su_diag'))
OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
CT, CE, CS = '#8a8986', '#eb6834', '#2a78d6'   # TR1/SR2, Su 厳密, 何もしない


def load(v):
    out = []
    for f in sorted(glob.glob(os.path.join(SRC, f'd{v}_s*.csv'))):
        d = np.genfromtxt(f, delimiter=',', names=True)
        out.append(d[np.argmax(d['reset'] == 1):])     # 最初の周期再構築より後（e_stale の基準が揃う）
    return out


T, E = load(0), load(1)
TT, EE = np.concatenate(T), np.concatenate(E)
nT, nE = TT['reset'] == 0, EE['reset'] == 0


def save(fig, fn): fig.savefig(os.path.join(OUT, fn), dpi=130, bbox_inches='tight'); plt.close(fig)


def by_csr(D, n, key, q=(25, 50, 75)):
    xs = np.arange(1, 100); ys = []
    for c in xs:
        m = n & (D['csr'] == c)
        ys.append(np.percentile(D[key][m], q) if m.sum() > 5 else [np.nan] * len(q))
    return xs, np.array(ys)


# 図1: Su の誤差 vs 前回のフル再構築からの周期数
fig, axs = plt.subplots(1, 2, figsize=(13, 4.6))
for D, n, key, col, lab in ((TT, nT, 'e_su', CT, 'TR1 で差分更新した Su（従来）'),
                            (TT, nT, 'e_stale', CS, '更新せず前回再構築の Su のまま（仮想）'),
                            (EE, nE, 'e_su', CE, 'Su 毎周期厳密（今回）')):
    x, y = by_csr(D, n, key)
    axs[0].plot(x, 100 * y[:, 1], color=col, lw=2, label=lab)
    axs[0].fill_between(x, 100 * y[:, 0], 100 * y[:, 2], color=col, alpha=0.18)
axs[0].set_xlabel('前回のフル再構築からの周期数（10 ms / 周期）'); axs[0].set_ylabel('‖Su − Su厳密‖ / ‖Su厳密‖ [%]')
axs[0].set_title('Su の相対誤差（中央値と 25–75 %）'); axs[0].legend(fontsize=8, loc='upper left'); axs[0].set_ylim(0, 70); axs[0].grid(alpha=0.3)
for D, n, col, lab in ((TT, nT, CT, 'TR1/SR2（従来）'), (EE, nE, CE, 'Su 毎周期厳密（H は周期再構築のまま）')):
    x, y = by_csr(D, n, 'e_h')
    axs[1].plot(x, 100 * y[:, 1], color=col, lw=2, label=lab)
    axs[1].fill_between(x, 100 * y[:, 0], 100 * y[:, 2], color=col, alpha=0.18)
axs[1].set_xlabel('前回のフル再構築からの周期数'); axs[1].set_ylabel('‖H − H厳密‖ / ‖H厳密‖ [%]')
axs[1].set_title('H（ヘシアン）の相対誤差'); axs[1].legend(fontsize=8, loc='upper left'); axs[1].set_ylim(0, 150); axs[1].grid(alpha=0.3)
fig.suptitle('SIL（full プラント・実機並みの揺れ、3 本）: 差分更新は「何もしない」より Su を悪くする', fontsize=12)
save(fig, 'fig01_su_error_vs_cycles.png')

# 図2: 1 本の時系列（のこぎり波）
d0, d1 = T[0], E[0]
fig, axs = plt.subplots(3, 1, figsize=(13, 8.5), sharex=True)
t0 = np.arange(len(d0)) * 0.01; t1 = np.arange(len(d1)) * 0.01
axs[0].plot(t0, 100 * d0['e_su'], color=CT, lw=1, label='Su 誤差（TR1）'); axs[0].plot(t0, 100 * d0['e_stale'], color=CS, lw=1, label='更新しない場合の Su 誤差（仮想）')
axs[0].plot(t1, 100 * d1['e_su'], color=CE, lw=1.5, label='Su 誤差（Su 毎周期厳密）= 0')
for tr in t0[d0['reset'] == 1]: axs[0].axvline(tr, color='k', lw=0.4, alpha=0.4)
axs[0].set_ylabel('Su 相対誤差 [%]'); axs[0].set_yscale('symlog', linthresh=10); axs[0].set_ylim(0, 400); axs[0].legend(fontsize=8, loc='upper left')
axs[1].plot(t0, 100 * d0['up_frac'], color=CT, lw=1, label='TR1'); axs[1].plot(t1, 100 * d1['up_frac'], color=CE, lw=1.5, label='Su 毎周期厳密')
axs[1].set_ylabel('非因果ブロックの割合 [%]\n(未来の入力→過去の状態)'); axs[1].legend(fontsize=8, loc='upper left')
axs[2].plot(t0, d0['cos_d_cur'], color=CT, lw=1, label='TR1/SR2'); axs[2].plot(t1, d1['cos_d_cur'], color=CE, lw=1.5, label='Su 毎周期厳密')
axs[2].set_ylabel('解の向きの一致度\ncos(d, d厳密)'); axs[2].set_ylim(-0.5, 1.05); axs[2].axhline(0, color='k', lw=0.5); axs[2].legend(fontsize=8, loc='lower left')
axs[2].set_xlabel('NMPC 起動からの時間 [s]（縦線 = TR1 側のフル再構築。周期 1 s ＋異常検知による臨時分）')
fig.suptitle('SIL seed0: TR1 はフル再構築のたびに誤差 0 から出直し、次の再構築まで誤差を積み上げる', fontsize=12)
save(fig, 'fig02_sawtooth_seed0.png')

# 図3: 解の向き・逆向き指令
fig, axs = plt.subplots(1, 2, figsize=(13, 4.4))
bins = np.linspace(-0.5, 1, 61)
axs[0].hist(TT['cos_d_cur'][nT], bins=bins, color=CT, alpha=0.8, label='TR1/SR2（Su・H とも差分更新）', log=True)
axs[0].hist(TT['cos_d_hold'][nT], bins=bins, histtype='step', color=CS, lw=1.5, label='同じ周期で Su だけ厳密にした場合（H は SR2 更新のまま）', log=True)
axs[0].hist(EE['cos_d_cur'][nE], bins=bins, color=CE, alpha=0.7, label='Su 毎周期厳密（今回）', log=True)
axs[0].set_xlabel('cos(制約なしの解, 厳密な Su・H での解)'); axs[0].set_ylabel('周期数（対数）'); axs[0].legend(fontsize=8, loc='upper left')
axs[0].set_title('解の向き（1 で完全一致、0 以下は直交〜逆向き）')
th = math.radians(20); labs, vals = [], []
for D, n, k, lab in ((TT, nT, 'cur', 'TR1/SR2'), (TT, nT, 'hold', 'SR2 更新の H＋厳密 Su'), (EE, nE, 'cur', 'Su 毎周期厳密')):
    w = []
    for s, mk in (('s_roll', 'mr_'), ('s_pitch', 'mp_')):
        big = n & (np.abs(D[s]) > th)
        w.append(100 * np.mean(np.sign(D[mk + k][big]) == np.sign(D[s][big])))
    labs.append(lab); vals.append(w)
vals = np.array(vals); x = np.arange(len(labs))
axs[1].bar(x - 0.18, vals[:, 0], 0.36, color=CS, label='ロール'); axs[1].bar(x + 0.18, vals[:, 1], 0.36, color=CE, label='ピッチ')
for i in range(len(labs)):
    for j, dx in ((0, -0.18), (1, 0.18)): axs[1].text(x[i] + dx, vals[i, j] + 0.1, f'{vals[i, j]:.1f}', ha='center', fontsize=9)
axs[1].set_xticks(x); axs[1].set_xticklabels(labs); axs[1].set_ylabel('逆向き指令の割合 [%]'); axs[1].legend(fontsize=8)
axs[1].set_title('先頭段の指令モーメントが姿勢を崩す向き（|s|>20 deg/s の周期）')
save(fig, 'fig03_direction_and_wrong_sign.png')

# 数値のまとめ（本文の表の元）
def row(D, n, k):
    x = D[k][n]; return f'{np.median(x):.4g} / {np.percentile(x, 95):.4g} / {x.max():.4g}'
with open(os.path.join(HERE, 'su_diag_summary.txt'), 'w') as f:
    for name, D, n in (('TR1/SR2', TT, nT), ('Su exact', EE, nE)):
        f.write(f'== {name}  normal cycles {n.sum()}  (median / p95 / max)\n')
        for k in ('e_su', 'e_stale', 'e_sx', 'up_frac', 'e_h', 'cos_c', 'dsu_rel', 'rank1_frac', 'sx_share'):
            f.write(f'  {k:11s} {row(D, n, k)}\n')
        for k in ('cos_d_cur', 'cos_d_hold'):
            x = D[k][n]; f.write(f'  {k:11s} median {np.median(x):.4g} p5 {np.percentile(x, 5):.4g} min {x.min():.4g}  <0.9: {100*np.mean(x<0.9):.2f}%  <0: {100*np.mean(x<0):.2f}%\n')
        for k in ('e_u0_cur', 'u0n_ex'):
            x = D[k][n]; f.write(f'  {k:11s} median {np.median(x):.4g} p99 {np.percentile(x, 99):.4g} max {x.max():.4g}\n')
        for lo, hi in ((1, 10), (10, 30), (30, 60), (60, 101)):
            m = n & (D['csr'] >= lo) & (D['csr'] < hi)
            f.write(f'  csr[{lo},{hi}) e_su {np.median(D["e_su"][m]):.3g} e_stale {np.median(D["e_stale"][m]):.3g} up {np.median(D["up_frac"][m]):.3g} e_h {np.median(D["e_h"][m]):.3g}\n')
        f.write(f'  wrong roll/pitch [%]: ' + ' '.join(f'{v:.2f}' for v in (vals[0] if name == 'TR1/SR2' else vals[2])) + '\n')
    f.write(f'  wrong (TR1 H + exact Su) roll/pitch [%]: {vals[1][0]:.2f} {vals[1][1]:.2f}\n')
print(open(os.path.join(HERE, 'su_diag_summary.txt')).read())
