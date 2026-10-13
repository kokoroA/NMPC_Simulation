import sys, os, math, importlib.util, numpy as np, csv, time
os.environ['MPLBACKEND'] = 'Agg'
spec = importlib.util.spec_from_file_location('st', 'nmpc_simulation_stress.py')
M = importlib.util.module_from_spec(spec); sys.modules['st'] = M; spec.loader.exec_module(M)
plt = M.plt
ang = np.deg2rad([90.0, -90.0, -30.0, 150.0, 30.0, -150.0])
HEAD_GEOM = dict(motor_x=tuple(0.05*np.cos(ang)), motor_y=tuple(0.05*np.sin(ang)),
                 motor_s=(-1.0, 1.0, -1.0, 1.0, 1.0, -1.0))
AIR_HEAD = dict(T_max=2.725, mass=0.200, plant_thrust_curve=False, plant_mass_ratio=1.05, plant_T_max_ratio=0.95,
                U_max=1.0, **HEAD_GEOM)
INER_HEAD = dict(Ix=6.10e-3, Iy=6.53e-3, Iz=1.16e-2, Cp=0.01, Cq=0.025, C_QT=0.03614)
CR_HEAD = dict(Cr=0.005)
TUNE_HEAD = dict(N=30, Q_phi=100.0, Q_theta=100.0, Q_p=10.0, Q_q=10.0, Q_r=5.0, R_u=14.0, admm_max_iter=1)
def cur(): return {}
CONFIGS = [
 ("A_旧シミュそのもの",            {**AIR_HEAD, **INER_HEAD, **CR_HEAD, **TUNE_HEAD}, 4),
 ("B_A+重みだけ現行",               {**AIR_HEAD, **INER_HEAD, **CR_HEAD}, 4),
 ("C_A+ヨー減衰Crだけ現行",         {**AIR_HEAD, **INER_HEAD, **TUNE_HEAD}, 4),
 ("D_A+慣性・腕長・CQTだけ現行",    {**{k:v for k,v in AIR_HEAD.items() if not k.startswith('motor_')}, **CR_HEAD, **TUNE_HEAD}, -1),
 ("E_A+推力・質量だけ現行",         {**{k:v for k,v in AIR_HEAD.items() if k.startswith('motor_')}, **INER_HEAD, **CR_HEAD, **TUNE_HEAD}, 4),
 ("F_機体は現行・重みだけ旧",       {**TUNE_HEAD}, 0),
 ("G_現行そのもの",                 {}, 0),
]
seeds = [42, 7, 123]
rows = []
img = sys.argv[1]; os.makedirs(img, exist_ok=True)
runs = {}
for name, over, fidx in CONFIGS:
    for seed in seeds:
        p = M.make_case_params([c for c in M.CONDITIONS if c[0] == 'base'][0], 0, seed)
        p.mass = 0.230; p.ftc_formulation = 'sim'
        for k, v in over.items(): setattr(p, k, v)
        if 'motor_x' in over: fault = 4          # 旧幾何の M5 = 前右
        elif fidx == -1: fault = 0
        else: fault = 0                          # 現行幾何の FR
        x0 = np.zeros(p.NX_plant); x0[M.PI_Z] = p.z_ref_default
        lg, m = M.run_scenario(p, M.FaultManager(schedule=[(5.0, fault, 0.0)]), M.phi_step_schedule([(0, 0)]),
                               fdi_known=True, t_end=15.0, label=name, savepath=None, x0=x0,
                               z_ref_default=p.z_ref_default, plan_open_loop_ascent=False)
        plt.close('all')
        d = lg.arrays(); runs[(name, seed)] = d
        uh = p.u_hover_nom
        rows.append(dict(config=name, seed=seed, diverged=m['diverged'], t_fail=m['t_fail'], why=m['why'],
                         alt_drop=m['alt_drop'], att_rms=m['att_rms_after_deg'], att_peak=m['att_peak_after_deg'],
                         r_peak=m['r_peak_after_dps'], u_hover=uh))
with open(os.path.join(img, 'ablation_metrics.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
print(f"{'config':<28} {'墜落':>4} {'姿勢RMS°':>9} {'姿勢峰°':>8} {'r峰°/s':>8} {'高度低下m':>9} {'ホバーduty':>9}")
for name, _, _ in CONFIGS:
    rs = [r for r in rows if r['config'] == name]
    nf = sum(r['diverged'] for r in rs)
    f = lambda k: f"{np.nanmin([r[k] for r in rs]):.2f}–{np.nanmax([r[k] for r in rs]):.2f}"
    print(f"{name:<28} {nf}/3  {f('att_rms'):>11} {f('att_peak'):>12} {f('r_peak'):>13} {f('alt_drop'):>11} {rs[0]['u_hover']:.3f}")
# 図: 構成ごとの高度と姿勢
fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
for i, (name, _, _) in enumerate(CONFIGS):
    col = plt.cm.tab10(i)
    for j, seed in enumerate(seeds):
        d = runs[(name, seed)]
        lab = name if j == 0 else None
        axs[0].plot(d['t'], np.degrees(np.max(np.abs(d['x'][:, 0:2]), axis=1)), color=col, lw=0.9, label=lab)
        axs[1].plot(d['t'], np.degrees(d['x'][:, 4]), color=col, lw=0.9)
        axs[2].plot(d['t'], -d['x'][:, M.PI_Z], color=col, lw=0.9)
axs[0].set_ylabel('姿勢 |φ|,|θ| の大きい方 [deg]'); axs[0].set_ylim(0, 60)
axs[1].set_ylabel('ヨー率 r [deg/s]'); axs[1].set_ylim(-300, 300)
axs[2].set_ylabel('高度 [m]'); axs[2].set_ylim(0, 2.6); axs[2].set_xlabel('時刻 [s]（t=5 s で前右モータ停止）')
for a in axs: a.axvline(5.0, color='k', ls=':', lw=1); a.grid(True, alpha=0.3)
axs[0].legend(fontsize=8, ncol=2, loc='upper left')
fig.suptitle('切り分け: 旧シミュ（成功）から 1 項目ずつ現行へ（元シミュ方式 FTC・230 g 相当・3 シード）', fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.96]); fig.savefig(os.path.join(img, 'ablation_overlay.png'), dpi=130, bbox_inches='tight')
print('saved', os.path.join(img, 'ablation_overlay.png'))
