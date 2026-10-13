"""シミュの機体外乱（プロセス雑音 σ_proc）を k 倍して、静かなホバーの p rms を実機（7〜8.6 deg/s）に合わせる。"""
import sys, os, math, importlib.util, numpy as np
os.environ['MPLBACKEND'] = 'Agg'
spec = importlib.util.spec_from_file_location('st', 'nmpc_simulation_stress.py')
M = importlib.util.module_from_spec(spec); sys.modules['st'] = M; spec.loader.exec_module(M)
R1 = [c for c in M.CONDITIONS if c[0] == 'R1'][0]
for k in [float(a) for a in sys.argv[1:]]:
    p = M.make_case_params(R1, 6, 42)
    p.sigma_p_proc *= k; p.sigma_q_proc *= k; p.sigma_r_proc *= k
    p.sigma_gyro_meas *= 3; p.sigma_phi_meas *= 3; p.sigma_theta_meas *= 3
    x0 = np.zeros(p.NX_plant); x0[M.PI_Z] = p.z_ref_default
    lg, m = M.run_scenario(p, M.FaultManager(schedule=[]), [(0.0, np.zeros(5))], fdi_known=True, t_end=15.0,
                           label=f'k={k}', savepath=None, x0=x0, z_ref_default=p.z_ref_default, plan_open_loop_ascent=False)
    M.plt.close('all')
    d = lg.arrays(); s = d['t'] >= 3.0
    pm = np.degrees(d['x_meas'][s][::10, 2]); qm = np.degrees(d['x_meas'][s][::10, 3])
    print(f"CALIB k={k:5.1f}: p rms={np.std(pm):6.2f}  q rms={np.std(qm):6.2f} deg/s  sat={m['sat_box']*100:.2f}%  diverged={m['diverged']}")
