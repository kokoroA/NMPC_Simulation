"""旧 nmpc_simulation.py（git 46a71bc = 2026-06-17 版、S1〜S8 検証済み）そのものに FTC を適用し、推力と質量を振る。
条件: 高度 2 m ホバーから開始、t=5 s に M5（前右）完全停止、FDI 既知（元の故障処理）、15 s、3 シード。
旧版には打ち切りが無いので、高度 0 m 未満 / 姿勢 90° 超 / 角速度 360 deg/s 超を事後判定で「墜落」とする。
"""
import sys, os, math, csv, importlib.util, numpy as np
os.environ['MPLBACKEND'] = 'Agg'
HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('head', os.path.join(HERE, 'nmpc_simulation_HEAD_46a71bc.py'))
H = importlib.util.module_from_spec(spec); sys.modules['head'] = H; spec.loader.exec_module(H)
CONFIGS = [  # (名前, T_max [N], 質量 [kg])
    ("元の値 T2.725_m200", 2.725, 0.200),
    ("T1.50_m230", 1.50, 0.230),
    ("T1.00_m230", 1.00, 0.230),
    ("T0.75_m200", 0.75, 0.200),
    ("T0.75_m230", 0.75, 0.230),
    ("T0.75_m285", 0.75, 0.285),
]
SEEDS = [42, 7, 123]
rows, series = [], {}
for name, tmax, mass in CONFIGS:
    for seed in SEEDS:
        p = H.Params(); p.T_max = tmax; p.mass = mass; p.noise_seed = seed
        tw = 6 * p.T_max * p.U_max * p.plant_T_max_ratio / (p.mass * p.plant_mass_ratio * p.gravity)
        x0 = np.zeros(p.NX_plant); x0[H.PI_Z] = p.z_ref_default
        try:
            lg, m = H.run_scenario(p, H.FaultManager(schedule=[(5.0, 4, 0.0)]), H.phi_step_schedule([(0.0, 0.0)]),
                                   fdi_known=True, t_end=15.0, label=name, savepath=None, x0=x0,
                                   z_ref_default=p.z_ref_default, plan_open_loop_ascent=False)
            d = lg.arrays(); err = ""
        except Exception as e:   # Cholesky 失敗など
            d = None; err = type(e).__name__
        H.plt.close('all')
        if d is None:
            rows.append(dict(program='旧', config=name, T_max=tmax, mass=mass, tw=tw, seed=seed, crashed=True,
                             t_fail=float('nan'), why=err, att_rms=float('nan'), r_peak=float('nan'), alt_min=float('nan')))
            continue
        t = d['t']; x = d['x']; alt = -x[:, H.PI_Z]
        ang = np.degrees(np.max(np.abs(x[:, 0:2]), axis=1)); rate = np.degrees(np.max(np.abs(x[:, 2:5]), axis=1))
        bad = (alt < 0) | (ang > 90) | (rate > 360) | ~np.isfinite(ang)
        i_fail = int(np.argmax(bad)) if np.any(bad) else None
        after = (t >= 5.0) & ((t < t[i_fail]) if i_fail is not None else True)
        why = "" if i_fail is None else ("altitude<0" if alt[i_fail] < 0 else ("|angle|>90" if ang[i_fail] > 90 else "|rate|>360"))
        rows.append(dict(program='旧', config=name, T_max=tmax, mass=mass, tw=tw, seed=seed, crashed=i_fail is not None,
                         t_fail=(float(t[i_fail]) if i_fail is not None else float('nan')), why=why,
                         att_rms=float(np.sqrt(np.mean(np.degrees(x[after, 0]) ** 2 + np.degrees(x[after, 1]) ** 2))), r_peak=float(np.max(np.abs(np.degrees(x[after, 4])))),
                         alt_min=float(np.min(alt[after]))))
        k = slice(None, i_fail + 1 if i_fail is not None else None, 10)
        series[f"{name}|{seed}"] = np.vstack([t[k], np.degrees(x[k, 0]), np.degrees(x[k, 1]), np.degrees(x[k, 4]), alt[k]])
        print(f"RESULT {name} seed={seed} T/W={tw:.2f} crashed={rows[-1]['crashed']} t_fail={rows[-1]['t_fail']:.2f} {why}", flush=True)
with open(os.path.join(HERE, 'head_metrics.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
np.savez_compressed(os.path.join(HERE, 'head_series.npz'), **series)
print("DONE head")
