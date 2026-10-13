"""1009_v1 の評価図（docs/20261009_v1_flight_eval.md 用）。<matplotlib のある python> figs_1009_v1.py <出力dir>"""
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
from analyze_1009_v1 import load, LOGS, metrics, MOT, segments
OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
C = {'1009_v1': '#eb6834', '0919_v8': '#8a8986', '0919_v6': '#b9b8b4', '0919_v5': '#52514e', '0911_v10': '#2a78d6'}
MC = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#4a3aa7']
R = {}
for name, path, lab in LOGS:
    c, D, _ = load(path); m, fl = metrics(c, D); R[name] = (m, c, D, fl, lab)
names = [n for n, _, _ in LOGS]
SHORT = {'1009_v1': '1009_v1\n(今回 Su厳密)', '0919_v8': '0919_v8\n(貨物・墜落)', '0919_v6': '0919_v6', '0919_v5': '0919_v5', '0911_v10': '0911_v10\n(当時最良)'}
xl = [SHORT[n] for n in names]

def save(fig, fn): fig.savefig(os.path.join(OUT, fn), dpi=130, bbox_inches='tight'); plt.close(fig)

# 図1: 1009_v1 の時系列
m, c, D, fl, _ = R['1009_v1']; t = D[:, c['t']]
fig, axs = plt.subplots(5, 1, figsize=(13, 13), sharex=True)
axs[0].plot(t, np.degrees(D[:, c['phi_ref']]), 'k:', lw=1, label='参照'); axs[0].plot(t, np.degrees(D[:, c['phi']]), color='#2a78d6', label='φ'); axs[0].set_ylabel('ロール [deg]'); axs[0].legend(fontsize=8)
axs[1].plot(t, np.degrees(D[:, c['theta_ref']]), 'k:', lw=1, label='参照'); axs[1].plot(t, np.degrees(D[:, c['theta']]), color='#eb6834', label='θ'); axs[1].set_ylabel('ピッチ [deg]'); axs[1].legend(fontsize=8)
axs[2].plot(t, np.degrees(D[:, c['psi_ref']]), 'k:', lw=1, label='ヨー率参照'); axs[2].plot(t, np.degrees(D[:, c['r']]), color='#4a3aa7', label='r'); axs[2].set_ylabel('ヨー率 [deg/s]'); axs[2].legend(fontsize=8); axs[2].set_ylim(-60, 80)
for j, k in enumerate(MOT): axs[3].plot(t, D[:, c[k]], color=MC[j], lw=0.8, label=k[2:].upper())
axs[3].plot(t, D[:, c['u_base']], 'k--', lw=0.8, label='u_base'); axs[3].set_ylabel('duty'); axs[3].legend(fontsize=7, ncol=7); axs[3].set_ylim(0, 1)
axs[4].plot(t, D[:, c['iter_us']] / 1000, color='#1baf7a', lw=0.8, label='iter_us（NMPC 本体）'); axs[4].plot(t, D[:, c['loop_us']] / 1000, color='#52514e', lw=0.8, label='loop_us（周期全体）')
axs[4].axhline(10, color='#e34948', ls=':', lw=1); axs[4].text(0.1, 10.5, '10 ms', color='#e34948', fontsize=8); axs[4].set_ylim(0, 32); axs[4].set_ylabel('[ms]'); axs[4].legend(fontsize=8); axs[4].set_xlabel('時間 [s]')
fig.suptitle('図1  1009_v1（Su 毎周期厳密化した版）: 19 s 以上を積極的な操縦で飛び、最後はスロットルを下げて着地。計算は再構築の周期（1 秒に 1 回）以外すべて 10 ms 以内', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97]); save(fig, 'fig01_timeseries.png')

# 図2: 計算時間
items = [('iter_norm_mean', 'NMPC 本体（通常周期）平均 [µs]'), ('iter_norm_max', 'NMPC 本体（通常周期）最大 [µs]'), ('loop_mean', '周期全体 平均 [µs]'),
         ('loop_over10', '周期全体が 10 ms を超えた割合'), ('rebuild_per_s', 'フル再構築の回数 [回/s]'), ('iter_rb_mean', 'フル再構築の周期の NMPC 時間 [µs]')]
fig, axs = plt.subplots(2, 3, figsize=(17, 8.5))
for ax, (k, lab) in zip(axs.flat, items):
    v = [R[n][0][k] for n in names]; ax.bar(range(len(names)), v, color=[C[n] for n in names])
    for i, x in enumerate(v): ax.text(i, x, f'{x:.3g}', ha='center', va='bottom', fontsize=8)
    if k.startswith('loop_mean'): ax.axhline(10000, color='#e34948', ls=':', lw=1)
    ax.set_xticks(range(len(names))); ax.set_xticklabels(xl, fontsize=8); ax.set_title(lab, fontsize=10)
fig.suptitle('図2  計算時間（すべて N=14・ADMM 4 反復・同じ機体）: Su 毎周期厳密化で NMPC 本体 −14〜16 %、周期全体の 10 ms 超過が 29–37 % → 1 %（再構築の周期のみ）', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.94]); save(fig, 'fig02_timing.png')

# 図3: 指令の質
items = [('du_max', '1 周期の |Δduty| 最大'), ('du_p99', '|Δduty| の 99 パーセンタイル'), ('du_rms', '|Δduty| rms'),
         ('wrong_roll', '逆向き指令の割合（ロール）'), ('wrong_pitch', '逆向き指令の割合（ピッチ）'), ('ulin_max', '線形化点 |U_lin| の最大'),
         ('box_any', 'box に張り付いた周期の割合'), ('frac_path2', 'TR1 棄却（path=2）の割合'), ('p_2_5', 'p の 2–5 Hz 帯の占有率')]
fig, axs = plt.subplots(3, 3, figsize=(17, 12))
for ax, (k, lab) in zip(axs.flat, items):
    v = [R[n][0][k] for n in names]; ax.bar(range(len(names)), v, color=[C[n] for n in names])
    for i, x in enumerate(v): ax.text(i, x, f'{x:.3g}', ha='center', va='bottom', fontsize=8)
    ax.set_xticks(range(len(names))); ax.set_xticklabels(xl, fontsize=8); ax.set_title(lab, fontsize=10)
fig.suptitle('図3  ソルバの出力の質: 指令の跳び・逆向き指令・box 張り付きがほぼ消えた（0919_v8 は貨物ありで墜落した便、0911_v10 は当時いちばん飛ばしやすかった便）', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.95]); save(fig, 'fig03_command_quality.png')

# 図4: |Δduty| の時系列（同じ縦軸で）
fig, axs = plt.subplots(len(names), 1, figsize=(13, 2.2 * len(names)), sharey=True)
for ax, n in zip(axs, names):
    m, c, D, fl, lab = R[n]; a, b = segments(fl)[0]
    U = D[a:b][:, [c[k] for k in MOT]]; tt = D[a + 1:b, c['t']] - D[a, c['t']]
    ax.plot(tt, np.abs(np.diff(U, axis=0)).max(1), color=C[n], lw=0.7); ax.set_ylim(0, 0.7)
    ax.set_title(f'{n}（{lab}）', fontsize=9, loc='left'); ax.set_ylabel('|Δduty|')
axs[-1].set_xlabel('飛行区間の開始からの時間 [s]')
fig.suptitle('図4  1 周期ごとの |Δduty| の最大（同じ縦軸）: 以前の便にあったスパイクが今回は無い', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97]); save(fig, 'fig04_du_timeseries.png')

# 図5: 追従（遅れ・帯域別の揺れ）
def band_rms(x, lo, hi, fs=100.0):
    x = np.asarray(x, float) - np.mean(x); n = len(x); w = np.hanning(n)
    X = np.fft.rfft(x * w); f = np.fft.rfftfreq(n, 1 / fs); P = np.abs(X) ** 2 / np.sum(w ** 2) * 2 / fs; df = f[1] - f[0]
    return math.sqrt(P[(f >= lo) & (f < hi)].sum() * df)
fig, axs = plt.subplots(1, 3, figsize=(17, 4.8))
bands = [(0.2, 2, '0.2–2 Hz\n(操縦)'), (2, 5, '2–5 Hz'), (5, 15, '5–15 Hz'), (15, 50.1, '15–50 Hz')]
for ax, k in zip(axs[:2], ('p', 'q')):
    w = 0.16
    for i, n in enumerate(names):
        m, c, D, fl, lab = R[n]; a, b = segments(fl)[0]
        ax.bar(np.arange(4) + (i - 2) * w, [math.degrees(band_rms(D[a:b, c[k]], lo, hi)) for lo, hi, _ in bands], w, color=C[n], label=n)
    ax.set_xticks(range(4)); ax.set_xticklabels([x[2] for x in bands]); ax.set_title(f'{k} の帯域別 rms [deg/s]', fontsize=10); ax.legend(fontsize=7)
lags = []
for n in names:
    m, c, D, fl, lab = R[n]; a, b = segments(fl)[0]; row = []
    for ax_, ref in (('phi', 'phi_ref'), ('theta', 'theta_ref')):
        x = D[a:b, c[ax_]] - D[a:b, c[ax_]].mean(); y = D[a:b, c[ref]] - D[a:b, c[ref]].mean()
        row.append(int(np.argmax([np.dot(x[k:], y[:len(y) - k]) for k in range(80)])) * 10)
    lags.append(row)
lags = np.array(lags)
axs[2].bar(np.arange(len(names)) - 0.18, lags[:, 0], 0.36, color='#2a78d6', label='ロール'); axs[2].bar(np.arange(len(names)) + 0.18, lags[:, 1], 0.36, color='#eb6834', label='ピッチ')
axs[2].set_xticks(range(len(names))); axs[2].set_xticklabels(names, fontsize=8); axs[2].set_title('参照に対する応答の遅れ（相互相関の最大）[ms]', fontsize=10); axs[2].legend(fontsize=8)
fig.suptitle('図5  揺れの帯域と応答の遅れ: 5–15 Hz の細かい揺れは最小、遅れは 100–110 ms（以前 120–310 ms）。0.2–2 Hz が大きいのは操縦入力（今回は参照 5° 超が 50 % の積極操縦）', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.9]); save(fig, 'fig05_tracking_bands.png')
print(sorted(os.listdir(OUT)))
