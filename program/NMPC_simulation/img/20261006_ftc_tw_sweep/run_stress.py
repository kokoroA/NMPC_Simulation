"""nmpc_simulation_stress.py（実機パラメータ化）の FTC を、質量と推力を振って回す。
条件: NX=5/N=14、遅れ補償 DC6、悪条件なし、高度 2 m ホバーから、t=5 s に FR 完全停止、15 s、3 シード。
推力は飛行較正曲線 Tf(u) を scale 倍し、NMPC モデルの T_max も同じ倍率にする（強いモータを積んだら同定し直す想定）。
方式: 実機方式（高度 PID / スロットル固定）と元シミュ方式。
"""
import sys, os, math, csv, importlib.util, numpy as np
os.environ['MPLBACKEND'] = 'Agg'
HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('st', os.path.join(HERE, '..', '..', 'nmpc_simulation_stress.py'))
M = importlib.util.module_from_spec(spec); sys.modules['st'] = M; spec.loader.exec_module(M)
BASE = [c for c in M.CONDITIONS if c[0] == 'base'][0]
CONFIGS = [  # (名前, 推力倍率, 質量 [kg])
    ("x1.0_m200", 1.0, 0.200), ("x1.0_m230", 1.0, 0.230), ("x1.0_m250", 1.0, 0.250), ("x1.0_m285", 1.0, 0.285),
    ("x1.5_m230", 1.5, 0.230), ("x2.0_m230", 2.0, 0.230), ("x3.0_m230", 3.0, 0.230),
]
FORMS = [("実機方式・スロットル固定", 'firmware', True), ("実機方式・高度PID", 'firmware', False), ("元シミュ方式", 'sim', False)]
SEEDS = [42, 7, 123]
rows, series = [], {}
for name, sc, mass in CONFIGS:
    for flab, form, hold in FORMS:
        for seed in SEEDS:
            p = M.make_case_params(BASE, 6, seed)
            p.mass = mass; p.thrust_curve_scale *= sc; p.T_max *= sc
            p.ftc_formulation = form; p.ftc_throttle_hold = hold
            tw = 6 * float(M.thrust_curve(p.U_max, p)) / (p.mass * p.gravity)
            x0 = np.zeros(p.NX_plant); x0[M.PI_Z] = p.z_ref_default
            lg, m = M.run_scenario(p, M.FaultManager(schedule=[(5.0, 0, 0.0)]), [(0.0, np.zeros(5))], fdi_known=True,
                                   t_end=15.0, label=f"{name} {flab}", savepath=None, x0=x0,
                                   z_ref_default=p.z_ref_default, plan_open_loop_ascent=False)
            M.plt.close('all')
            d = lg.arrays(); t = d['t']; x = d['x']; alt = -x[:, M.PI_Z]
            rows.append(dict(program='stress', config=name, form=flab, scale=sc, mass=mass, tw=tw, seed=seed,
                             crashed=m['diverged'], t_fail=m['t_fail'], why=m['why'], att_rms=m['att_rms_after_deg'],
                             r_peak=m['r_peak_after_dps'], alt_min=m['alt_min_after'], carry_sat=m['carry_sat_after'],
                             u_carry=m['u_carry_mean_after']))
            series[f"{name}|{flab}|{seed}"] = np.vstack([t[::10], np.degrees(x[::10, 0]), np.degrees(x[::10, 1]),
                                                        np.degrees(x[::10, 4]), alt[::10]])
            print(f"RESULT {name} {flab} seed={seed} T/W={tw:.2f} crashed={m['diverged']} t_fail={m['t_fail']:.2f}", flush=True)
with open(os.path.join(HERE, 'stress_metrics.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
np.savez_compressed(os.path.join(HERE, 'stress_series.npz'), **series)
print("DONE stress")
