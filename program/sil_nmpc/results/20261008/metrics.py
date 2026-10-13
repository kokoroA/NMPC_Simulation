"""SIL 評価の指標（firmware の 100Hz ログ＋プラント真値）。実機 0919_v5/v6/v8 と同じ定義で数える。"""
import glob, math, os, sys
import numpy as np

MOT = ['u_fr', 'u_fl', 'u_mr', 'u_ml', 'u_rr', 'u_rl']
REAL_DIR = '/Users/kokoro/Documents/research_project/program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/tools/'
REAL = [('0919_v6', 5.0, 14.0), ('0919_v8', 5.0, 10.3), ('0919_v5', 6.0, 16.0)]


def load_real(path):
    cols, rows = None, []
    for line in open(path, errors='replace'):
        line = line.strip()
        if line.startswith('#cols:'):
            cols = [x.strip() for x in line[6:].split(',')]; continue
        if not line or line.startswith('#'): continue
        sp = line.split(',')
        if cols is None or len(sp) != len(cols): continue
        try: rows.append([float(x) for x in sp])
        except ValueError: pass
    return {n: i for i, n in enumerate(cols)}, np.array(rows)


def log_metrics(L, c, m):
    """L: firmware ログ、m: 対象行のマスク（飛行中）"""
    U = L[np.ix_(m, [c[k] for k in MOT])]
    dU = np.diff(U, axis=0)
    du = np.abs(dU).max(1)
    path = L[m, c['solver_code']].astype(int) // 10
    box = L[m, c['solver_code']].astype(int) % 10
    skip = L[m, c['tr1_skip_run']] if 'tr1_skip_run' in c else np.zeros(m.sum())
    return dict(
        p_rms=math.degrees(L[m, c['p']].std()), q_rms=math.degrees(L[m, c['q']].std()),
        r_rms=math.degrees(L[m, c['r']].std()),
        du_rms=float(np.sqrt((dU ** 2).mean())), du_p99=float(np.percentile(du, 99)), du_max=float(du.max()),
        path0=float((path == 0).mean()), path1=float((path == 1).mean()), path2=float((path == 2).mean()),
        box_any=float((box > 0).mean()), skip_max=float(skip.max()),
        sat0=float((U <= 1e-3).mean()), sat95=float((U >= 0.95 - 1e-3).mean()),
    )


def sil_run(path):
    d = np.load(path, allow_pickle=True)
    L = d['log']; c = {n: i for i, n in enumerate(d['log_cols'])}
    t = np.arange(len(L)) * 0.01
    out = dict(name=os.path.basename(path)[:-4], cond=str(d['cond']), seed=int(d['seed']), crash=str(d['crash']),
               order_bad=int(d['order_bad']), order_checks=int(d['order_checks']), holds=int(d['n_holds']))
    # 区間別の姿勢誤差（firmware 推定値 − 参照）
    e_phi = np.degrees(L[:, c['phi']] - L[:, c['phi_ref']])
    e_th = np.degrees(L[:, c['theta']] - L[:, c['theta_ref']])
    def rms(x, a, b):
        mm = (t >= a) & (t < b)
        return float(np.sqrt((x[mm] ** 2).mean())) if mm.any() else float('nan')
    out['hover_att_rms'] = rms(np.hypot(e_phi, e_th), 4.0, 7.0)
    out['roll_step_rms'] = rms(e_phi, 7.5, 9.0)       # 立ち上がり 0.5 s を除く
    out['pitch_step_rms'] = rms(e_th, 11.5, 13.0)
    out['late_hover_rms'] = rms(np.hypot(e_phi, e_th), 15.5, 19.5)
    # ステップ応答: 参照の 90% に達するまで
    for nm, col, t0, ref in (('roll_t90', 'phi', 7.0, 15.0), ('pitch_t90', 'theta', 11.0, 10.0)):
        mm = (t >= t0) & (t < t0 + 2.0)
        x = np.degrees(L[mm, c[col]]); tt = t[mm]
        hit = np.where(x >= 0.9 * ref)[0]
        out[nm] = float(tt[hit[0]] - t0) if hit.size else float('nan')
        out[nm.replace('t90', 'ovs')] = float(x.max() - ref) if mm.any() else float('nan')
    m = (t >= 4.0) & (t < min(19.5, t[-1] + 0.01))
    if m.sum() > 10:
        out.update(log_metrics(L, c, m))
    return out


def real_runs():
    res = []
    for v, a, b in REAL:
        c, D = load_real(REAL_DIR + v + '.txt')
        t = D[:, c['t']]; m = (t >= a) & (t < b)
        r = dict(name=v, cond='real_aircraft', crash='')
        r.update(log_metrics(D, c, m))
        res.append(r)
    return res


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    rows = [sil_run(p) for p in sorted(glob.glob(os.path.join(here, '*_s[0-9].npz')))] + real_runs()
    keys = ['name', 'crash', 'hover_att_rms', 'roll_t90', 'roll_ovs', 'roll_step_rms', 'pitch_t90', 'pitch_ovs',
            'pitch_step_rms', 'late_hover_rms', 'p_rms', 'q_rms', 'r_rms', 'du_rms', 'du_p99', 'du_max',
            'path0', 'path1', 'path2', 'box_any', 'skip_max', 'sat0', 'sat95', 'holds', 'order_bad', 'order_checks']
    import csv
    with open(os.path.join(here, 'metrics.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore'); w.writeheader()
        for r in rows: w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
    for r in rows:
        print(' '.join(f"{k}={r[k]:.3g}" if isinstance(r.get(k), float) else f"{k}={r.get(k, '')}" for k in keys))
