"""健全飛行で「実機にあってシミュに無い要素」を 1 つずつ足し、どれで崩れるかを見る。
使い方: program/NMPC_simulation で
    python img/20261006_healthy_realism/realism.py img/20261006_healthy_realism
共通: NX=5/N=14, 遅れ補償 DC6（実機）, R1（計算遅延 1 周期 + 再構築保持 3 周期 + ジャイロバイアス）, 20 s, 高度 2 m から
"""
import sys, os, math, csv, importlib.util, numpy as np
os.environ['MPLBACKEND'] = 'Agg'
spec = importlib.util.spec_from_file_location('st', 'nmpc_simulation_stress.py')
M = importlib.util.module_from_spec(spec); sys.modules['st'] = M; spec.loader.exec_module(M)
plt = M.plt
R1 = [c for c in M.CONDITIONS if c[0] == 'R1'][0]
D2R = math.radians

def ref_schedule(pilot: bool):
    z = np.zeros(5)
    if not pilot:
        return [(0.0, z)]
    def v(phi=0.0, th=0.0):
        x = np.zeros(5); x[0] = D2R(phi); x[1] = D2R(th); return x
    # 0919_v8 の操縦入力（±13〜20°）を模したステップ列
    return [(0.0, v()), (4.0, v(phi=15)), (6.0, v(phi=-15)), (8.0, v()),
            (10.0, v(th=10)), (12.0, v(th=-10)), (14.0, v()), (16.0, v(phi=20)), (17.0, v())]

# 実測由来の定常外乱トルク
TAU_ROLL_LEFT3 = -0.0028     # 左側 3 発の推力 3 % 不足（0910_v8 の左傾きトリム）≈ −74 deg/s²
TAU_YAW_RESID = 5.4e-4       # ヨー残留トルク ≈ 10 deg/s²（20260914 Stage1/2）
def dist(on: bool):
    if not on:
        return M.DisturbanceManager()
    return M.DisturbanceManager([
        {'t_start': 0.0, 't_end': 1e9, 'kind': 'torque_body', 'axis': 0, 'magnitude': TAU_ROLL_LEFT3},
        {'t_start': 0.0, 't_end': 1e9, 'kind': 'torque_body', 'axis': 2, 'magnitude': TAU_YAW_RESID}])

# (名前, 操縦入力, 雑音倍率, 外乱トルク, 質量[kg], 効き倍率, DC[, 機体外乱倍率])
PROC_K = float(os.environ.get('PROC_K', '1.0'))   # 機体外乱（プロセス雑音）の倍率。10 で実機ホバーの揺れ（p rms 7〜8.6 deg/s）に一致
CONFIGS = [
    ("H0_シミュ現状(R1)",        False, 1.0, False, 0.285, 1.0, 6),
    ("H1_+操縦入力",             True,  1.0, False, 0.285, 1.0, 6),
    ("H2_+雑音3倍",              False, 3.0, False, 0.285, 1.0, 6),
    ("H3_+外乱トルク",           False, 1.0, True,  0.285, 1.0, 6),
    ("H4_+貨物310g",             False, 1.0, False, 0.310, 1.0, 6),
    ("H5_全部",                  True,  3.0, True,  0.310, 1.0, 6),
    ("H5d0_全部・遅れ補償なし",  True,  3.0, True,  0.310, 1.0, 0),
    ("H6_全部+効き2倍",          True,  3.0, True,  0.310, 2.0, 6),
]
SEEDS = [42, 7, 123]
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
rows, runs = [], {}
for name, pilot, noise_k, dist_on, mass, gain, dc in CONFIGS:
    for seed in SEEDS:
        p = M.make_case_params(R1, dc, seed)
        p.mass = mass; p.plant_moment_gain = gain
        p.sigma_gyro_meas *= noise_k; p.sigma_phi_meas *= noise_k; p.sigma_theta_meas *= noise_k
        p.sigma_p_proc *= PROC_K; p.sigma_q_proc *= PROC_K; p.sigma_r_proc *= PROC_K
        x0 = np.zeros(p.NX_plant); x0[M.PI_Z] = p.z_ref_default
        lg, m = M.run_scenario(p, M.FaultManager(schedule=[]), ref_schedule(pilot), fdi_known=True,
                               t_end=20.0, label=name, savepath=None, x0=x0, dist_mgr=dist(dist_on),
                               z_ref_default=p.z_ref_default, plan_open_loop_ascent=False)
        plt.close('all')
        d = lg.arrays(); runs[(name, seed)] = d
        alt = -d['x'][:, M.PI_Z]
        rows.append(dict(config=name, seed=seed, diverged=m['diverged'], t_fail=m['t_fail'], why=m['why'],
                         rms_phi=m['rms_phi_ctrl_deg'], rms_theta=m['rms_theta_ctrl_deg'],
                         peak_pq=m['peak_pq_ctrl_dps'], p_2_5=m['p_band_2_5'], du_hf5=m['du_hf5'],
                         sat=m['sat_box'], alt_min=float(np.min(alt[d['t'] >= 1.0]))))
with open(os.path.join(out, 'realism_metrics.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

print(f"{'config':<26} {'墜落':>4} {'RMSφ°':>11} {'RMSθ°':>11} {'角速度峰':>13} {'p2-5Hz%':>11} {'張付%':>11} {'最低高度m':>10}")
for name, *_ in CONFIGS:
    rs = [r for r in rows if r['config'] == name]
    nf = sum(r['diverged'] for r in rs)
    def rng(k, s=1.0):
        v = [r[k] * s for r in rs]; return f"{np.nanmin(v):.1f}–{np.nanmax(v):.1f}"
    print(f"{name:<26} {nf}/3 {rng('rms_phi'):>11} {rng('rms_theta'):>11} {rng('peak_pq'):>13} "
          f"{rng('p_2_5',100):>11} {rng('sat',100):>11} {min(r['alt_min'] for r in rs):>10.2f}")

# ---- 図 1: 条件ごとのロール・ピッチ（参照つき）----
fig, axs = plt.subplots(len(CONFIGS), 2, figsize=(14, 2.1 * len(CONFIGS)), sharex=True)
for i, (name, *_ ) in enumerate(CONFIGS):
    for j, seed in enumerate(SEEDS):
        d = runs[(name, seed)]
        for c, idx in ((0, 0), (1, 1)):
            ax = axs[i, c]
            ax.plot(d['t'], np.degrees(d['x'][:, idx]), lw=0.9, color=plt.cm.tab10(j), label=f"seed {seed}")
            if j == 0:
                ax.plot(d['t'], np.degrees(d['x_ref'][:, idx]), 'k--', lw=0.8, label='参照')
    rs = [r for r in rows if r['config'] == name]
    nf = sum(r['diverged'] for r in rs)
    axs[i, 0].set_ylabel(f"{name}\n墜落 {nf}/3", fontsize=8)
    for c in range(2):
        axs[i, c].set_ylim(-45, 45); axs[i, c].grid(True, alpha=0.3)
axs[0, 0].set_title('ロール φ [deg]'); axs[0, 1].set_title('ピッチ θ [deg]')
axs[0, 1].legend(fontsize=7, ncol=4, loc='upper right')
axs[-1, 0].set_xlabel('時刻 [s]'); axs[-1, 1].set_xlabel('時刻 [s]')
fig.suptitle(f'健全飛行: 実機にある要素を 1 つずつ足したときの姿勢（NX=5/N=14, DC6, 計算遅延 1 + 再構築保持 3 + バイアス, 機体外乱 ×{PROC_K:g}）', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97]); fig.savefig(os.path.join(out, 'realism_attitude.png'), dpi=120, bbox_inches='tight')

# ---- 図 2: 指標の棒グラフ ----
names = [c[0] for c in CONFIGS]
mets = [("墜落率 [%]", lambda r: 100.0 * float(r['diverged'])),
        ("ロール追従誤差 RMS [deg]", lambda r: r['rms_phi']),
        ("ロール/ピッチ角速度の最大 [deg/s]", lambda r: r['peak_pq']),
        ("p の 2–5 Hz 成分 [%]", lambda r: 100.0 * r['p_2_5']),
        ("duty の 0/0.95 張り付き [%]", lambda r: 100.0 * r['sat'])]
fig, axs = plt.subplots(len(mets), 1, figsize=(13, 2.3 * len(mets)), sharex=True)
x = np.arange(len(names))
for ax, (lab, fn) in zip(axs, mets):
    vals = [[fn(r) for r in rows if r['config'] == n] for n in names]
    mean = np.array([np.nanmean(v) for v in vals])
    err = [mean - np.array([np.nanmin(v) for v in vals]), np.array([np.nanmax(v) for v in vals]) - mean]
    ax.bar(x, mean, 0.7, color='#2a78d6', edgecolor='white', yerr=err, capsize=3, error_kw=dict(lw=0.8, ecolor='#52514e'))
    for xi, v in zip(x, mean):
        ax.text(xi, v, f"{v:.3g}", ha='center', va='bottom', fontsize=7)
    ax.set_ylabel(lab, fontsize=8); ax.grid(True, axis='y', alpha=0.3)
axs[-1].set_xticks(x); axs[-1].set_xticklabels([n.replace('_', '\n', 1) for n in names], fontsize=8)
fig.suptitle(f'健全飛行の指標（機体外乱 ×{PROC_K:g}、棒 = 3 シード平均、ひげ = 最小〜最大）', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.96]); fig.savefig(os.path.join(out, 'realism_summary.png'), dpi=120, bbox_inches='tight')
print('saved figures')
