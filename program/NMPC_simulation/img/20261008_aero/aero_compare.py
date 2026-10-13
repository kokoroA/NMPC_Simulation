"""Python シミュ（nmpc_simulation_stress.py, R1・DC6）に PKG-E10 空力を入れたときの比較（2026-10-08）。

    cd program/NMPC_simulation && <venv python> img/20261008_aero/aero_compare.py

SIL と同じ飛行: 2 m ホバーから t=5–7 s ロール 15°、t=9–11 s ピッチ 10°、14 s まで。
⚠ Python シミュの NMPC は状態をほぼ直接観測する（EKF なし）ので、加速度計の比力による「偽の傾き」は原理的に出ない。
"""
import os, sys, math, importlib.util
import numpy as np
os.environ['MPLBACKEND'] = 'Agg'
HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('st', os.path.join(HERE, '..', '..', 'nmpc_simulation_stress.py'))
M = importlib.util.module_from_spec(spec); sys.modules['st'] = M; spec.loader.exec_module(M)

R1 = [c for c in M.CONDITIONS if c[0] == 'R1'][0]
VARIANTS = [('off', 'off', 1.0), ('full_bf0', 'full', 0.0), ('full_bf05', 'full', 0.5), ('full', 'full', 1.0)]
SEEDS = [42, 7, 123]


def ref_schedule():
    def x(phi, th): return np.array([math.radians(phi), math.radians(th), 0.0, 0.0, 0.0])
    return [(0.0, x(0, 0)), (5.0, x(15, 0)), (7.0, x(0, 0)), (9.0, x(0, 10)), (11.0, x(0, 0))]


out = {}
for name, mode, k in VARIANTS:
    M.AERO_ARGS = (mode, k)
    for seed in SEEDS:
        p = M.make_case_params(R1, 6, seed)
        x0 = np.zeros(p.NX_plant); x0[M.PI_Z] = p.z_ref_default
        lg, m = M.run_scenario(p, M.FaultManager(schedule=[]), ref_schedule(), fdi_known=True, t_end=14.0,
                               label=f'{name}_s{seed}', savepath=None, x0=x0, z_ref_default=p.z_ref_default,
                               plan_open_loop_ascent=False)
        X = np.array(lg.x); t = np.array(lg.t)
        out[f'{name}_s{seed}'] = dict(t=t, phi=X[:, M.PI_PHI], theta=X[:, M.PI_THETA], xp=X[:, M.PI_X], yp=X[:, M.PI_Y],
                                      vx=X[:, M.PI_UVEL], vy=X[:, M.PI_VVEL], z=X[:, M.PI_Z],
                                      u=np.array(lg.u_app), diverged=bool(m.get('diverged', False)) if isinstance(m, dict) else False)
        o = out[f'{name}_s{seed}']
        def mean(x, a, b): mm = (t >= a) & (t < b); return float(np.degrees(x[mm]).mean())
        print(f"{name:10s} s{seed:<3d} roll 0.7-1.0s {mean(o['phi'],5.7,6.0)/15:5.2f}  1.7-2.0s {mean(o['phi'],6.7,7.0)/15:5.2f} | "
              f"pitch 0.7-1.0s {mean(o['theta'],9.7,10.0)/10:5.2f} 1.7-2.0s {mean(o['theta'],10.7,11.0)/10:5.2f} | "
              f"vh_max {np.hypot(o['vx'],o['vy']).max():5.2f} m/s drift {math.hypot(o['xp'][-1],o['yp'][-1]):5.1f} m", flush=True)
np.savez_compressed(os.path.join(HERE, 'aero_compare.npz'), **{f'{k}__{f}': v for k, d in out.items() for f, v in d.items()})
