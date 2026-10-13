"""実機の巡航ログとシミュ（H0 / H5）の、角速度 p,q と指令 du の帯域別 rms を比べる。"""
import sys, os, math, importlib.util, numpy as np
os.environ['MPLBACKEND'] = 'Agg'
spec = importlib.util.spec_from_file_location('st', 'nmpc_simulation_stress.py')
M = importlib.util.module_from_spec(spec); sys.modules['st'] = M; spec.loader.exec_module(M)
plt = M.plt
def load_log(path):
    cols, rows = None, []
    for line in open(path, errors='replace'):
        line = line.strip()
        if line.startswith('#cols:'):
            cols = [x.strip() for x in line[6:].split(',')]; continue
        if not line or line.startswith('#'): continue
        sp = line.split(',')
        if cols is None or len(sp) != len(cols): continue
        try: rows.append([float(x) for x in sp])
        except ValueError: continue
    return {n: i for i, n in enumerate(cols)}, np.array(rows)
LOGDIR = '/Users/kokoro/Documents/research_project/program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/tools/'
BANDS = [(0.2, 2), (2, 5), (5, 15), (15, 50.1)]
def band_rms(x, fs=100.0):
    x = np.asarray(x, float) - np.mean(x); n = len(x)
    X = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs)
    P = (np.abs(X) ** 2) / n ** 2 * 2
    return [math.sqrt(P[(f >= lo) & (f < hi)].sum()) for lo, hi in BANDS], f, P
MOT = ['u_fr', 'u_fl', 'u_mr', 'u_ml', 'u_rr', 'u_rl']
series = {}
for v, a, b in [('0919_v6', 5.0, 14.0), ('0919_v8', 5.0, 10.3), ('0919_v5', 6.0, 16.0)]:
    c, D = load_log(LOGDIR + v + '.txt'); t = D[:, c['t']]; m = (t >= a) & (t < b)
    U = D[np.ix_(m, [c[k] for k in MOT])]
    series['実機 ' + v] = dict(p=np.degrees(D[m, c['p']]), q=np.degrees(D[m, c['q']]),
                              du=U - U.mean(axis=1, keepdims=True))
R1 = [c for c in M.CONDITIONS if c[0] == 'R1'][0]
for lab, mass, noise_k, proc_k in [('シミュ H0', 0.285, 1.0, 1.0), ('シミュ 雑音3倍', 0.285, 3.0, 1.0),
                                   ('シミュ 雑音3倍+機体外乱10倍', 0.285, 3.0, 10.0)]:
    p = M.make_case_params(R1, 6, 42); p.mass = mass
    p.sigma_p_proc *= proc_k; p.sigma_q_proc *= proc_k; p.sigma_r_proc *= proc_k
    p.sigma_gyro_meas *= noise_k; p.sigma_phi_meas *= noise_k; p.sigma_theta_meas *= noise_k
    x0 = np.zeros(p.NX_plant); x0[M.PI_Z] = p.z_ref_default
    lg, _ = M.run_scenario(p, M.FaultManager(schedule=[]), [(0.0, np.zeros(5))], fdi_known=True, t_end=15.0,
                           label=lab, savepath=None, x0=x0, z_ref_default=p.z_ref_default, plan_open_loop_ascent=False)
    plt.close('all')
    d = lg.arrays(); k = d['t'] >= 3.0
    xm = d['x_meas'][k][::10]; u = d['u_cmd'][k][::10]
    series[lab] = dict(p=np.degrees(xm[:, 2]), q=np.degrees(xm[:, 3]), du=u - u.mean(axis=1, keepdims=True))
print(f"{'':<26}{'p 全体':>8} | p 帯域 rms [deg/s]: {'0.2-2':>6} {'2-5':>6} {'5-15':>6} {'15-50':>6} | du rms {'0.2-2':>6} {'2-5':>6} {'5-15':>6} {'15-50':>6}")
res = {}
for lab, s in series.items():
    bp, f, Pp = band_rms(s['p'])
    bd = np.mean([band_rms(s['du'][:, j])[0] for j in range(6)], axis=0)
    res[lab] = (bp, bd, f, Pp)
    du_all = float(np.mean(np.std(s['du'], axis=0))); du_max = float(np.max(np.abs(s['du'])))
    print(f"{lab:<26} du rms={du_all:.4f} max={du_max:.3f} ", end='')
    print(f"{np.std(s['p']):8.2f} |                     {bp[0]:6.2f} {bp[1]:6.2f} {bp[2]:6.2f} {bp[3]:6.2f} |        {bd[0]:6.3f} {bd[1]:6.3f} {bd[2]:6.3f} {bd[3]:6.3f}")
# 図
fig, axs = plt.subplots(1, 2, figsize=(14, 5))
labs = list(res.keys()); x = np.arange(len(BANDS)); w = 0.8 / len(labs)
cols = ['#e34948', '#eb6834', '#eda100', '#2a78d6', '#1baf7a', '#4a3aa7']
for i, lab in enumerate(labs):
    axs[0].bar(x + (i - (len(labs) - 1) / 2) * w, res[lab][0], w * 0.95, color=cols[i], label=lab)
    axs[1].bar(x + (i - (len(labs) - 1) / 2) * w, res[lab][1], w * 0.95, color=cols[i], label=lab)
for ax, t_ in zip(axs, ['ロール角速度 p の帯域別 rms [deg/s]', '指令 du（差分）の帯域別 rms [duty]']):
    ax.set_xticks(x); ax.set_xticklabels([f"{lo:g}–{hi:g} Hz" for lo, hi in [(0.2, 2), (2, 5), (5, 15), (15, 50)]])
    ax.set_title(t_); ax.grid(True, axis='y', alpha=0.3)
axs[0].legend(fontsize=8)
fig.suptitle('静かなホバー中の揺れ: 実機（赤系）とシミュ（青緑）', fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.94]); fig.savefig(os.path.join(sys.argv[1], 'real_vs_sim_bands.png'), dpi=130, bbox_inches='tight')
print('saved')
