"""SIL 評価（docs/20261008_sil_nmpc_flight_eval.md）用の図。

    python3 build_figs.py <出力ディレクトリ>

入力: このディレクトリの *.npz（run_all.sh / run_ablation.sh / run_nmpc_eval.py --legacy-order）と実機ログ 0919_v5/v6/v8
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

from metrics import load_real, REAL, REAL_DIR, MOT, sil_run
from wrongsign import from_log, wrong_frac, KXM, KYM, K
from compare import runs, by_cycles

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
C_BLUE, C_ORANGE, C_AQUA, C_YELLOW, C_MAG, C_VIOLET, C_RED = '#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#4a3aa7', '#e34948'
INK2 = '#52514e'
MCOL = [C_BLUE, C_ORANGE, C_AQUA, C_YELLOW, C_MAG, C_VIOLET]
MNAME = ['FR', 'FL', 'MR', 'ML', 'RR', 'RL']


def load(path):
    d = np.load(path, allow_pickle=True)
    L = d['log']; c = {n: i for i, n in enumerate(d['log_cols'])}
    P = d['plant']; pc = {n: i for i, n in enumerate(d['plant_cols'])}
    tP = P[:, pc['t']] - P[0, pc['t']]
    return d, L, c, P, pc, tP


def shade(ax):
    ax.axvspan(7, 9, color=C_BLUE, alpha=0.06); ax.axvspan(11, 13, color=C_ORANGE, alpha=0.06)


def save(fig, name):
    fig.savefig(os.path.join(OUT, name), dpi=130, bbox_inches='tight'); plt.close(fig)


# ---------- 図1: モータ順の不具合（修正前 vs 修正後、理想条件） ----------
fig, axs = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
for col, (fn, title) in enumerate((('bug_legacy_order_ideal.npz', '修正前の SIL（モータ順が入れ替わっていた）'),
                                   ('ideal_s0.npz', '修正後（実機ファームの set_duty_* と同じ順）'))):
    d, L, c, P, pc, tP = load(os.path.join(HERE, fn))
    t = np.arange(len(L)) * 0.01
    ax = axs[0, col]
    ax.plot(t, np.degrees(L[:, c['phi_ref']]), '--', color=C_BLUE, lw=1, label='φ 参照')
    ax.plot(t, np.degrees(L[:, c['phi']]), color=C_BLUE, label='φ（ロール）')
    ax.plot(t, np.degrees(L[:, c['theta_ref']]), '--', color=C_ORANGE, lw=1, label='θ 参照')
    ax.plot(t, np.degrees(L[:, c['theta']]), color=C_ORANGE, label='θ（ピッチ）')
    ax.set_ylim(-30, 75); ax.set_ylabel('姿勢 [deg]'); ax.set_title(title, fontsize=11); shade(ax)
    if d['crash'] != '':
        ax.axvline(t[-1], color=C_RED, lw=1); ax.text(t[-1] + 0.3, 40, '墜落\n' + str(d['crash']), color=C_RED, ha='left', fontsize=9)
    ax.legend(fontsize=8, loc='upper left', ncol=2)
    ax = axs[1, col]
    for j, k in enumerate(MOT):
        ax.plot(t, L[:, c[k]], color=MCOL[j], lw=1, label=MNAME[j])
    ax.set_ylabel('firmware の duty 指令'); ax.set_ylim(0, 1); shade(ax); ax.legend(fontsize=8, ncol=6, loc='upper left')
    ax = axs[2, col]
    for j in range(6):
        ax.plot(tP, P[:, pc[f'd{j}']], color=MCOL[j], lw=1, label=MNAME[j])
    ax.set_ylabel('プラントが受け取った duty'); ax.set_ylim(0, 1); ax.set_xlabel('LOG 開始からの時間 [s]'); shade(ax)
    ax.text(0.01, 0.92, f"順序検査の不一致 {int(d['order_bad'])}/{int(d['order_checks'])} 周期", transform=ax.transAxes, fontsize=9,
            color=C_RED if int(d['order_bad']) > 50 else INK2)
for ax in axs[0]: ax.set_xlim(0, 19.5)
fig.suptitle('図1  SIL のモータ順の不具合: ホバー（6 発同値）では見えず、ロール指令を入れた瞬間にピッチへ化けて墜落する', fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.96]); save(fig, 'fig01_motor_order_bug.png')

# ---------- 図2: 理想条件での通常飛行 ----------
d, L, c, P, pc, tP = load(os.path.join(HERE, 'ideal_s0.npz'))
R = d['refs']; t = np.arange(len(L)) * 0.01
fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
axs[0].plot(R[:, 0], R[:, 1], '--', color=INK2, lw=1, label='高度目標（操縦者モデル）')
axs[0].plot(tP, P[:, pc['alt']], color=C_VIOLET, label='高度（プラント真値）'); axs[0].set_ylabel('高度 [m]'); axs[0].legend(fontsize=8)
axs[1].plot(t, np.degrees(L[:, c['phi_ref']]), '--', color=C_BLUE, lw=1); axs[1].plot(t, np.degrees(L[:, c['phi']]), color=C_BLUE, label='φ')
axs[1].plot(t, np.degrees(L[:, c['theta_ref']]), '--', color=C_ORANGE, lw=1); axs[1].plot(t, np.degrees(L[:, c['theta']]), color=C_ORANGE, label='θ')
axs[1].set_ylabel('姿勢 [deg]'); axs[1].legend(fontsize=8)
for j, k in enumerate(MOT): axs[2].plot(t, L[:, c[k]], color=MCOL[j], lw=1, label=MNAME[j])
axs[2].plot(t, L[:, c['u_base']], 'k--', lw=0.8, label='u_hover'); axs[2].set_ylabel('duty'); axs[2].legend(fontsize=8, ncol=7)
pth = L[:, c['solver_code']].astype(int) // 10
axs[3].scatter(t, pth, s=3, color=INK2); axs[3].set_yticks([0, 1, 2]); axs[3].set_yticklabels(['通常(TR1採用)', 'フル再構築', 'TR1 棄却'])
axs[3].set_xlabel('LOG 開始からの時間 [s]'); axs[3].set_title(f'ソルバ経路（雑音ゼロでは割線が小さすぎ TR1 の 88% が |c|>1e6 で棄却）', fontsize=10)
for ax in axs: shade(ax)
fig.suptitle('図2  理想条件（雑音・遅れなし）: 離陸 → ホバー → ロール +15° → ピッチ +10° を完走', fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.96]); save(fig, 'fig02_ideal_flight.png')

# ---------- 図3: 実機並み条件での 6 シード ----------
fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
for s in range(6):
    for cond, ls in (('real', '-'),):
        dd, L, c, P, pc, tP = load(os.path.join(HERE, f'{cond}_s{s}.npz'))
        t = np.arange(len(L)) * 0.01
        axs[0].plot(t, np.degrees(L[:, c['phi']]), ls, lw=0.8, color=MCOL[s], label=f'seed {s}')
        axs[1].plot(t, np.degrees(L[:, c['theta']]), ls, lw=0.8, color=MCOL[s])
        U = L[:, [c[k] for k in MOT]]
        axs[2].plot(t[1:], np.abs(np.diff(U, axis=0)).max(1), lw=0.6, color=MCOL[s])
dd, L, c, *_ = load(os.path.join(HERE, 'noise_s2.npz')); t = np.arange(len(L)) * 0.01
axs[0].plot(t, np.degrees(L[:, c['phi']]), color='k', lw=1.4, label='noise seed2（墜落）')
axs[1].plot(t, np.degrees(L[:, c['theta']]), color='k', lw=1.4)
axs[0].plot(t, np.degrees(L[:, c['phi_ref']]), '--', color=INK2, lw=1)
axs[0].set_ylabel('φ [deg]'); axs[0].set_ylim(-45, 45); axs[0].legend(fontsize=8, ncol=7)
axs[1].set_ylabel('θ [deg]'); axs[1].set_ylim(-45, 60)
axs[2].axhline(0.544, color=C_RED, lw=1, ls=':'); axs[2].text(19.3, 0.56, '実機 0919_v8 巡航区間の最大 0.544（同じ定義）', ha='right', color=C_RED, fontsize=9)
axs[2].set_ylabel('1 周期の |Δduty| 最大'); axs[2].set_xlabel('LOG 開始からの時間 [s]'); axs[2].set_ylim(0, 1)
for ax in axs: shade(ax)
fig.suptitle('図3  実機並み条件（ジャイロ雑音・機体外乱・加速度雑音・出力遅延 10 ms・再構築中 30 ms 据え置き）: 実機ファームそのまま', fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.96]); save(fig, 'fig03_realistic_flights.png')

# ---------- 図4: noise seed2 の墜落の解剖 ----------
dd, L, c, P, pc, tP = load(os.path.join(HERE, 'noise_s2.npz')); t = np.arange(len(L)) * 0.01
m = (t >= 7.5)
U = L[:, [c[k] for k in MOT]]
Mp = (KXM * U).sum(1); s_p = K * (L[:, c['theta']] - L[:, c['theta_ref']]) + L[:, c['q']]
fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
axs[0].plot(t[m], np.degrees(L[m, c['theta']]), color=C_ORANGE, label='θ'); axs[0].plot(t[m], np.degrees(L[m, c['q']]) / 5, color=C_ORANGE, lw=0.8, ls='--', label='q / 5 [deg/s]')
axs[0].plot(t[m], np.degrees(L[m, c['r']]) / 5, color=C_VIOLET, lw=0.8, label='r / 5 [deg/s]')
axs[0].axhline(0, color=INK2, lw=0.5); axs[0].legend(fontsize=8); axs[0].set_ylabel('[deg]')
axs[1].plot(t[m], s_p[m], color=C_ORANGE, label='戻すべき量 s = 4(θ−θref) + q'); axs[1].plot(t[m], Mp[m] * 40, color=C_BLUE, label='指令ピッチモーメント Σ kXm·u（×40）')
wrong = m & (np.abs(s_p) > math.radians(20)) & (np.sign(Mp) == np.sign(s_p))
axs[1].scatter(t[wrong], Mp[wrong] * 40, color=C_RED, s=12, zorder=5, label='逆向き（崩す向き）の指令')
axs[1].axhline(0, color=INK2, lw=0.5); axs[1].legend(fontsize=8); axs[1].set_ylabel('[rad/s] / [N·m×40]')
for j, k in enumerate(MOT): axs[2].plot(t[m], L[m, c[k]], color=MCOL[j], lw=1, label=MNAME[j])
axs[2].legend(fontsize=8, ncol=6); axs[2].set_ylabel('duty')
axs[3].plot(t[m], L[m, c['cycles']], color=INK2, label='最後のフル再構築からの周期数'); axs[3].set_ylabel('周期')
pth = L[:, c['solver_code']].astype(int) // 10; box = L[:, c['solver_code']].astype(int) % 10
ax2 = axs[3].twinx(); ax2.step(t[m], box[m], color=C_RED, lw=0.8, where='post', label='box 張り付き本数'); ax2.set_ylabel('box 本数', color=C_RED)
axs[3].set_xlabel('LOG 開始からの時間 [s]')
for ax in axs: ax.axvspan(8.45, 8.9, color=C_RED, alpha=0.06)
fig.suptitle('図4  墜落した 1 本の解剖: 通常経路（TR1 採用・制約非有効）のまま、ソルバがピッチ・ヨーを崩す向きの指令を出し続けた', fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.96]); save(fig, 'fig04_crash_anatomy.png')

# ---------- 図5: 再構築周期のアブレーション ----------
BUILDS = [('rp1', 'ablation_rp1/', '毎周期'), ('rp10', 'ablation_rp10/', '10 周期'), ('rp30', 'ablation_rp30/', '30 周期'), ('rp100', '', '100 周期\n(実機ファーム)')]
CONDS = [('noise', '雑音・外乱のみ', C_AQUA), ('real_nh', '+ 出力遅延 10 ms', C_BLUE), ('real', '+ 再構築据え置き 30 ms', C_ORANGE)]
cache = {}
for bl, pre, _ in BUILDS:
    for cd, *_ in CONDS:
        cache[(bl, cd)] = runs(pre, cd)
metrics = [('du_max', '1 周期の |Δduty| 最大（6 本平均）', 0.544, '実機 v8'), ('du_p99', '|Δduty| の 99 パーセンタイル', 0.194, '実機 v8'),
           ('wrong', '戻す向きと逆の指令の割合', 0.04, '実機 v5 最大'), ('late_max', '後半ホバーの姿勢誤差 rms 最悪 [deg]', None, None)]
fig, axs = plt.subplots(2, 2, figsize=(13, 9))
x = np.arange(len(BUILDS)); w = 0.26
for ax, (key, lab, ref, reflab) in zip(axs.flat, metrics):
    for i, (cd, cl, col) in enumerate(CONDS):
        vals = []
        for bl, *_ in BUILDS:
            R = cache[(bl, cd)]
            if not R: vals.append(np.nan); continue
            if key == 'wrong':
                vals.append(np.nanmean([0.5 * (r['roll_wrong'] + r['pitch_wrong']) for r in R]))
            elif key == 'late_max':
                vals.append(np.nanmax([r['late_hover_rms'] for r in R]))
            else:
                vals.append(np.nanmean([r[key] for r in R]))
        ax.bar(x + (i - 1) * w, vals, w, color=col, label=cl)
        for xi, v in zip(x, vals):
            if np.isnan(v): ax.text(xi + (i - 1) * w, 0, '実機で\n成立せず', ha='center', va='bottom', fontsize=7, color=INK2)
    if ref is not None:
        ax.axhline(ref, color=C_RED, ls=':', lw=1); ax.text(-0.45, ref, reflab, color=C_RED, fontsize=8, va='bottom', ha='left')
    ax.set_xticks(x); ax.set_xticklabels([b[2] for b in BUILDS]); ax.set_title(lab, fontsize=10)
    ax.set_xlabel('フル再構築の間隔（reset_period）')
axs[0, 0].legend(fontsize=8)
crash_txt = '  '.join(f"{b[2].splitlines()[0]}: 墜落 {sum(sum(bool(r['crash']) for r in cache[(b[0], cd)]) for cd, *_ in CONDS)}/{sum(len(cache[(b[0], cd)]) for cd, *_ in CONDS)}" for b in BUILDS)
fig.suptitle('図5  再構築の間隔だけを変えたファーム（他は同一）: 低ランク更新で走る周期が長いほど指令が荒れ、逆向き指令が増える\n' + crash_txt, fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.93]); save(fig, 'fig05_rebuild_period_ablation.png')

# ---------- 図6: 再構築からの周期数 ----------
std = cache[('rp100', 'noise')] + cache[('rp100', 'real_nh')] + cache[('rp100', 'real')]
bc = by_cycles(std)
fig, axs = plt.subplots(1, 2, figsize=(13, 4.8))
lbl = [f'{a}–{b - 1}' for a, b, *_ in bc]
axs[0].plot(lbl, [x[3] * 100 for x in bc], 'o-', color=C_RED, label='SIL 実機ファーム（18 本）')
axs[1].plot(lbl, [x[2] for x in bc], 'o-', color=C_RED, label='SIL 実機ファーム（18 本）')
for v, a, b in REAL:
    cc, D = load_real(REAL_DIR + v + '.txt'); tt = D[:, cc['t']]; mm = (tt >= a) & (tt < b)
    br = by_cycles([dict(L=D[mm], c=cc)])
    axs[0].plot(lbl, [np.nan_to_num(x[3]) * 100 for x in br], 's--', lw=1, label=f'実機 {v}')
    axs[1].plot(lbl, [x[2] for x in br], 's--', lw=1, label=f'実機 {v}')
axs[0].set_ylabel('逆向き指令の割合 [%]'); axs[1].set_ylabel('|Δduty| の 95 パーセンタイル')
for ax in axs: ax.set_xlabel('最後のフル再構築からの周期数'); ax.legend(fontsize=8)
fig.suptitle('図6  再構築からの周期数で区切ると: SIL では逆向き指令が周期数とともに増える。実機 v8 は 80 周期以降に指令が荒れる', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.92]); save(fig, 'fig06_cycles_since_rebuild.png')

# ---------- 図7: SIL と実機の数値比較 ----------
rows = []
for v, a, b in REAL:
    cc, D = load_real(REAL_DIR + v + '.txt'); tt = D[:, cc['t']]; mm = (tt >= a) & (tt < b)
    from metrics import log_metrics
    r = log_metrics(D, cc, mm); r.update(from_log(D, cc, mm)); rows.append((f'実機\n{v[5:]}', r))
for lab, bl, cd in (('SIL\n実機ファーム', 'rp100', 'real'), ('SIL\n10周期再構築', 'rp10', 'real'),
                    ('SIL\n毎周期再構築*', 'rp1', 'real_nh')):
    R = cache[(bl, cd)]
    agg = {k: float(np.nanmean([r[k] for r in R])) for k in ('p_rms', 'q_rms', 'r_rms', 'du_rms', 'du_max', 'path2')}
    agg['wrong'] = float(np.nanmean([0.5 * (r['roll_wrong'] + r['pitch_wrong']) for r in R]))
    rows.append((lab, agg))
for r in rows[:3]: r[1]['wrong'] = 0.5 * (np.nan_to_num(r[1]['roll_wrong']) + np.nan_to_num(r[1]['pitch_wrong']))
items = [('p_rms', 'p rms [deg/s]'), ('r_rms', 'r rms [deg/s]'), ('du_rms', '|Δduty| rms'), ('du_max', '|Δduty| 最大'),
         ('wrong', '逆向き指令の割合'), ('path2', 'TR1 棄却の割合')]
fig, axs = plt.subplots(2, 3, figsize=(17, 8))
cols = [INK2, '#8a8986', '#b9b8b4', C_RED, C_ORANGE, C_AQUA]
for ax, (k, lab) in zip(axs.flat, items):
    vals = [r[1][k] for r in rows]
    ax.bar(range(len(rows)), vals, color=cols)
    ax.set_xticks(range(len(rows))); ax.set_xticklabels([r[0] for r in rows], fontsize=8, rotation=0)
    ax.set_title(lab, fontsize=10)
fig.suptitle('図7  実機ログ（巡航区間）と SIL（実機並み条件、各 6 本平均）の比較\n揺れの大きさは揃うが、実機ファームの SIL は指令の荒れ・逆向き指令が実機より多く、逆に TR1 棄却は実機の 1/8 しかない'
             '\n（* 毎周期再構築は再構築据え置きなし。実機の計算時間では成立しない参考値）', fontsize=10)
fig.tight_layout(rect=[0, 0, 1, 0.90]); save(fig, 'fig07_sil_vs_real.png')
print('done', sorted(os.listdir(OUT)))
