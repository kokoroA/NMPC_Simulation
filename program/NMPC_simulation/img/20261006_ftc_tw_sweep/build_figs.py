"""資料（docs/20261006_ftc_tw_and_sim_vs_real_report.md）用の図を作る。
入力: head_metrics.csv / head_series.npz / stress_metrics.csv / stress_series.npz / 実機ログ
出力: 引数のディレクトリ（docs/figs/20261006_report/）
"""
import sys, os, csv, math, numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
for fam in ('Hiragino Sans', 'Hiragino Maru Gothic Pro', 'AppleGothic'):
    try:
        matplotlib.font_manager.findfont(fam, fallback_to_default=False); plt.rcParams['font.family'] = fam; break
    except Exception:
        pass
plt.rcParams['axes.unicode_minus'] = False
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
C_BLUE, C_ORANGE, C_AQUA, C_YELLOW, C_MAG, C_VIOLET, C_RED = '#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#4a3aa7', '#e34948'
INK2 = '#52514e'
def rd(p): return list(csv.DictReader(open(os.path.join(HERE, p))))
def b(x): return x in ('True', True)
head = rd('head_metrics.csv'); stress = rd('stress_metrics.csv')
HS = np.load(os.path.join(HERE, 'head_series.npz')); SS = np.load(os.path.join(HERE, 'stress_series.npz'))
TF95 = 0.8389 * (-0.2265 * 0.95 ** 2 + 1.0525 * 0.95 + 0.0452)
TW_REAL = {m: 6 * TF95 / (m * 9.81) for m in (0.230, 0.285, 0.3095)}

# ---------- 図1: 推力重量比 vs 生存率 ----------
def agg(rows, key):
    g = {}
    for r in rows:
        g.setdefault(key(r), []).append(r)
    return g
fig, axs = plt.subplots(2, 1, figsize=(11, 9), sharex=True)
for ax in axs:
    for m_, lab in ((0.285, '実機 285 g'), (0.230, '実機 230 g')):
        ax.axvline(TW_REAL[m_], color=INK2, ls=':', lw=1.2)
axs[0].text(TW_REAL[0.285], 104, '実機 285 g', ha='center', fontsize=9, color=INK2)
axs[0].text(TW_REAL[0.230], 104, '実機 230 g', ha='center', fontsize=9, color=INK2)
series = [("旧 nmpc_simulation.py（元の故障処理）", head, lambda r: r['config'], C_VIOLET, 'D'),
          ("stress・元シミュ方式", [r for r in stress if r['form'] == '元シミュ方式'], lambda r: r['config'], C_AQUA, 's'),
          ("stress・実機方式・高度PID", [r for r in stress if r['form'] == '実機方式・高度PID'], lambda r: r['config'], C_BLUE, 'o'),
          ("stress・実機方式・スロットル固定", [r for r in stress if r['form'] == '実機方式・スロットル固定'], lambda r: r['config'], C_ORANGE, '^')]
for lab, rows, key, col, mk in series:
    pts = []
    for k, rs in agg(rows, key).items():
        tw = float(rs[0]['tw']); surv = 100 * np.mean([not b(r['crashed']) for r in rs])
        att = np.nanmean([float(r['att_rms']) for r in rs])
        pts.append((tw, surv, att))
    pts.sort()
    kw = dict(color=col, marker=mk, ms=8, lw=1.6, markeredgecolor='white', markeredgewidth=1.5)
    axs[0].plot([p[0] for p in pts], [p[1] for p in pts], label=lab, **kw)
    axs[1].plot([p[0] for p in pts], [p[2] for p in pts], **kw)
axs[1].axhspan(0, 10, color=C_AQUA, alpha=0.08); axs[1].text(6.5, 5, '姿勢誤差 10° 未満', fontsize=9, color=INK2)
axs[0].set_ylabel('10 s 生存した割合 [%]'); axs[0].set_ylim(-5, 112); axs[0].legend(fontsize=9, loc='center right')
axs[1].set_ylabel('故障後の姿勢誤差 RMS [deg]\n（墜落までの区間、3 シード平均）'); axs[1].set_ylim(0, 75)
axs[1].set_xscale('log'); axs[1].set_xticks([1.25, 1.5, 2, 3, 4, 6, 8]); axs[1].get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
axs[1].set_xlabel('推力重量比 T/W（6 発の最大推力 ÷ 重量、対数軸）')
for ax in axs: ax.grid(True, alpha=0.3)
fig.suptitle('1 発完全停止に耐えられるかは推力重量比で大きく変わる（ホバー中に前右モータが完全停止）', fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'fig01_survival_vs_tw.png'), dpi=140); plt.close(fig)

# ---------- 図2/3: 時系列（旧プログラム / stress スロットル固定）----------
def ts_fig(S, keys, title, fname, cols):
    fig, axs = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    for (k, lab), col in zip(keys, cols):
        for j, seed in enumerate((42, 7, 123)):
            kk = f"{k}|{seed}"
            if kk not in S.files: continue
            a = S[kk]
            axs[0].plot(a[0], np.maximum(np.abs(a[1]), np.abs(a[2])), color=col, lw=1.0, label=(lab if j == 0 else None))
            axs[1].plot(a[0], a[3], color=col, lw=1.0)
            axs[2].plot(a[0], a[4], color=col, lw=1.0)
    axs[0].set_ylabel('姿勢 |φ|,|θ| の大きい方 [deg]'); axs[0].set_ylim(0, 60)
    axs[1].set_ylabel('ヨー率 r [deg/s]'); axs[1].set_ylim(-300, 300)
    axs[2].set_ylabel('高度 [m]'); axs[2].set_ylim(0, 2.6); axs[2].set_xlabel('時刻 [s]（点線 t=5 s でモータ停止）')
    for a_ in axs: a_.axvline(5.0, color='k', ls=':', lw=1); a_.grid(True, alpha=0.3)
    axs[0].legend(fontsize=8, loc='upper left', ncol=2)
    fig.suptitle(title, fontsize=12); fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(OUT, fname), dpi=130); plt.close(fig)
hcfg = [(r['config'], r['config'], float(r['tw'])) for r in head]
hcfg = sorted({(a, b_, c) for a, b_, c in hcfg}, key=lambda x: -x[2])
ts_fig(HS, [(k, f"{k}（T/W {tw:.2f}）") for k, _, tw in hcfg],
       '旧 nmpc_simulation.py: 推力と質量を変えると FTC の成否が変わる（3 シード重ね描き）', 'fig02_old_sim_timeseries.png',
       [C_VIOLET, C_BLUE, C_AQUA, C_YELLOW, C_ORANGE, C_RED])
scfg = sorted({(r['config'], float(r['tw'])) for r in stress}, key=lambda x: -x[1])
ts_fig(SS, [(f"{k}|実機方式・高度PID", f"{k}（T/W {tw:.2f}）") for k, tw in scfg],
       'nmpc_simulation_stress.py・実機方式・高度PID: 推力と質量を変えたとき', 'fig03_stress_timeseries.png',
       [C_VIOLET, C_BLUE, C_AQUA, C_YELLOW, C_ORANGE, C_RED, C_MAG])

# ---------- 図4: 故障後の余地（解析）----------
a_, b_, c_, s_ = -0.2265, 1.0525, 0.0452, 0.8389
Tf = lambda u: s_ * (a_ * u * u + b_ * u + c_)
def inv(T):
    y = T / s_; D = b_ * b_ - 4 * a_ * (c_ - y); return (-b_ + math.sqrt(max(D, 0))) / (2 * a_)
masses = np.arange(0.180, 0.321, 0.005)
uh = [inv(m * 9.81 / 6) for m in masses]; uc = [inv(m * 9.81 / 4) for m in masses]
Iy = 1.9e-3 * 0.85
auth = [math.degrees(2 * max(Tf(0.95) - Tf(min(u, 0.95)), 0) * 0.085 / Iy) for u in uc]
fig, axs = plt.subplots(1, 2, figsize=(13, 4.8))
axs[0].plot(masses * 1000, uh, color=C_BLUE, lw=2, label='健全 6 発のホバー duty')
axs[0].plot(masses * 1000, uc, color=C_ORANGE, lw=2, label='故障後に支える 4 発の duty')
axs[0].axhline(0.95, color=C_RED, ls='--', lw=1); axs[0].text(185, 0.965, 'duty 上限 0.95', fontsize=9, color=C_RED)
for m_ in (230, 285):
    axs[0].axvline(m_, color=INK2, ls=':', lw=1)
axs[0].set_xlabel('質量 [g]'); axs[0].set_ylabel('duty'); axs[0].set_ylim(0.3, 1.05); axs[0].grid(True, alpha=0.3); axs[0].legend(fontsize=9)
axs[0].set_title('故障後に必要な duty（推力曲線は実機の飛行較正値）')
axs[1].plot(masses * 1000, auth, color=C_VIOLET, lw=2)
for m_, lab in ((230, '230 g'), (285, '285 g')):
    v = math.degrees(2 * max(Tf(0.95) - Tf(min(inv(m_ / 1000 * 9.81 / 4), 0.95)), 0) * 0.085 / Iy)
    axs[1].plot(m_, v, 'o', color=C_VIOLET, ms=8, markeredgecolor='white', markeredgewidth=1.5)
    axs[1].annotate(f"{lab}: {v:.0f} deg/s²", (m_, v), textcoords='offset points', xytext=(8, 8), fontsize=9)
axs[1].set_xlabel('質量 [g]'); axs[1].set_ylabel('故障後に出せるピッチ角加速度 [deg/s²]'); axs[1].grid(True, alpha=0.3)
axs[1].set_title('故障後の姿勢の権限（上方向の余地から計算）')
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'fig04_post_fault_margin.png'), dpi=130); plt.close(fig)

# ---------- 図8: 実機 0919_v8 の指令スパイク ----------
def load_log(path):
    cols, rows = None, []
    for line in open(path, errors='replace'):
        line = line.strip()
        if line.startswith('#cols:'): cols = [x.strip() for x in line[6:].split(',')]; continue
        if not line or line.startswith('#'): continue
        sp = line.split(',')
        if cols is None or len(sp) != len(cols): continue
        try: rows.append([float(x) for x in sp])
        except ValueError: pass
    return {n: i for i, n in enumerate(cols)}, np.array(rows)
LOG = '/Users/kokoro/Documents/research_project/program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/tools/'
c, D = load_log(LOG + '0919_v8.txt'); t = D[:, c['t']]
MOT = ['u_fr', 'u_fl', 'u_mr', 'u_ml', 'u_rr', 'u_rl']; U = D[:, [c[k] for k in MOT]]; ub = D[:, c['u_base']]
du = U - ub[:, None]; path = (D[:, c['solver_code']] // 10).astype(int)
k = (t >= 7.3) & (t <= 8.4)
fig, axs = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True)
cols6 = [C_BLUE, C_ORANGE, C_AQUA, C_YELLOW, C_MAG, C_VIOLET]
for j in range(6):
    axs[0].plot(t[k], du[k, j], color=cols6[j], lw=1.2, label=MOT[j].replace('u_', '').upper())
axs[0].set_ylabel('指令 du = u − u_base'); axs[0].legend(fontsize=8, ncol=6, loc='lower left'); axs[0].set_ylim(-0.4, 0.4)
axs[1].plot(t[k], np.degrees(D[k, c['p']]), color=C_BLUE, lw=1.2, label='p')
axs[1].plot(t[k], np.degrees(D[k, c['q']]), color=C_ORANGE, lw=1.2, label='q')
axs[1].set_ylabel('角速度 [deg/s]'); axs[1].legend(fontsize=8); axs[1].set_ylim(-40, 40)
axs[2].step(t[k], path[k], where='post', color=INK2, lw=1.2)
axs[2].set_yticks([0, 1, 2]); axs[2].set_yticklabels(['通常', 'フル再構築', 'TR1 棄却']); axs[2].set_ylabel('ソルバの経路')
for tt in t[k][path[k] == 1]:
    for a_ in axs: a_.axvline(tt, color=C_RED, lw=0.8, alpha=0.6)
i = np.where(k)[0][np.argmax(np.abs(du[k]).max(axis=1))]
axs[0].annotate(f"最大 {np.abs(du[i]).max():.3f}（通常経路・機体は静か）", (t[i], du[i, np.argmax(np.abs(du[i]))]),
                textcoords='offset points', xytext=(15, -5), fontsize=9)
axs[2].set_xlabel('ログ時刻 [s]（赤線 = フル再構築）')
fig.suptitle('実機 0919_v8 巡航中の指令スパイク: 通常経路で育ち、直後のフル再構築で消える', fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.96]); fig.savefig(os.path.join(OUT, 'fig08_v8_spike.png'), dpi=130); plt.close(fig)

# ---------- 図9: 再構築からの周期数と指令の大きさ ----------
fig, ax = plt.subplots(figsize=(11, 4.5))
bins = [(0, 10), (10, 30), (30, 60), (60, 101)]; w = 0.26
for i_, (v, a0, a1, col) in enumerate((('0919_v5', 6.0, 16.0, C_AQUA), ('0919_v6', 5.0, 14.0, C_ORANGE), ('0919_v8', 5.0, 10.3, C_BLUE))):
    c2, D2 = load_log(LOG + v + '.txt'); t2 = D2[:, c2['t']]; m2 = (t2 >= a0) & (t2 < a1)
    U2 = D2[:, [c2[kk] for kk in MOT]]; big = np.abs(U2 - D2[:, c2['u_base']][:, None]).max(axis=1); cyc = D2[:, c2['cycles']]
    vals = [np.percentile(big[m2 & (cyc >= lo) & (cyc < hi)], 95) for lo, hi in bins]
    ax.bar(np.arange(4) + (i_ - 1) * w, vals, w * 0.92, color=col, edgecolor='white', label=v)
ax.set_xticks(range(4)); ax.set_xticklabels([f"{lo}–{hi - 1} 周期" for lo, hi in bins])
ax.set_xlabel('最後のフル再構築からの周期数'); ax.set_ylabel('|du| の最大値の p95'); ax.grid(True, axis='y', alpha=0.3); ax.legend(fontsize=9)
ax.set_title('近似行列のずれの仮説: v8 では再構築から時間が経つほど指令が大きいが、v5/v6 では見えない', fontsize=11)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'fig09_cycles_since_rebuild.png'), dpi=130); plt.close(fig)
print('figs done')
