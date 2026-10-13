"""プラントの段階（ladder）ごとの指標。firmware の 100Hz ログとプラント真値の両方を使う。

    python3 metrics_plant.py            → plant_metrics.csv と条件別の平均表を出す
"""
import csv, glob, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '20261008'))
from metrics import load_real, REAL, REAL_DIR, MOT        # noqa: E402
from wrongsign import from_log                             # noqa: E402

LADDER = ['old', 'legacy', 'sf_nodrag', 'sf', 'bf', 'motor', 'full', 'wind']


def load(path):
    d = np.load(path, allow_pickle=True)
    L = d['log']; c = {n: i for i, n in enumerate(d['log_cols'])}
    P = d['plant']; pc = {n: i for i, n in enumerate(d['plant_cols'])}
    tP = P[:, pc['t']] - P[0, pc['t']]
    t = np.arange(len(L)) * 0.01
    idx = np.clip(np.searchsorted(tP, t + 1e-6), 0, len(tP) - 1)   # 100 Hz の各行に対応するプラント時刻
    return d, L, c, P, pc, tP, t, idx


def run_metrics(path):
    d, L, c, P, pc, tP, t, idx = load(path)
    phi_t = P[idx, pc['phi']]; th_t = P[idx, pc['theta']]
    out = dict(name=os.path.basename(path)[:-4], plant=str(d['plant_kind']), cond=str(d['cond']),
               seed=int(d['seed']), set=str(d['plant_set']), crash=str(d['crash']))
    end = t[-1] - (0.5 if out['crash'] else 0.0)
    fl = (t >= 4.0) & (t <= end)

    def mean(x, a, b):
        m = (t >= a) & (t < b) & (t <= end)
        return float(np.mean(x[m])) if m.any() else float('nan')

    # 真の姿勢で見た追従（参照に対する比）。ステップ開始から 0.7–1.0 s と 1.7–2.0 s
    for ax, x, t0, ref in (('roll', phi_t, 7.0, 15.0), ('pitch', th_t, 11.0, 10.0)):
        out[f'{ax}_true_07'] = mean(np.degrees(x), t0 + 0.7, t0 + 1.0) / ref
        out[f'{ax}_true_17'] = mean(np.degrees(x), t0 + 1.7, t0 + 2.0) / ref
        est = L[:, c['phi' if ax == 'roll' else 'theta']]
        out[f'{ax}_est_17'] = mean(np.degrees(est), t0 + 1.7, t0 + 2.0) / ref
    # 推定（EKF）− 真値
    e = np.hypot(np.degrees(L[:, c['phi']] - phi_t), np.degrees(L[:, c['theta']] - th_t))
    out['est_err_rms'] = float(np.sqrt(np.mean(e[fl] ** 2))) if fl.any() else float('nan')
    out['est_err_max'] = float(e[fl].max()) if fl.any() else float('nan')
    # 真の姿勢誤差（参照との差）: ホバー区間
    et = np.hypot(np.degrees(phi_t - L[:, c['phi_ref']]), np.degrees(th_t - L[:, c['theta_ref']]))
    hv = (((t >= 4) & (t < 7)) | ((t >= 15.5) & (t < 19.5))) & (t <= end)
    out['hover_true_rms'] = float(np.sqrt(np.mean(et[hv] ** 2))) if hv.any() else float('nan')
    out['true_err_max'] = float(et[fl].max()) if fl.any() else float('nan')
    # 並進
    if 'vn' in pc:
        sp = np.hypot(P[:, pc['vn']], P[:, pc['ve']])
        out['vh_max'] = float(sp.max())
        out['drift_m'] = float(np.hypot(P[-1, pc['pn']], P[-1, pc['pe']]))
    else:
        out['vh_max'] = out['drift_m'] = float('nan')
    # firmware 側（実機ログと同じ定義）
    U = L[np.ix_(fl, [c[k] for k in MOT])]
    dU = np.abs(np.diff(U, axis=0)).max(1) if U.shape[0] > 2 else np.array([np.nan])
    out['du_max'] = float(np.nanmax(dU)); out['du_p99'] = float(np.nanpercentile(dU, 99))
    out['du_rms'] = float(np.sqrt(np.nanmean(np.diff(U, axis=0) ** 2))) if U.shape[0] > 2 else float('nan')
    out['p_rms'] = math.degrees(L[fl, c['p']].std()); out['q_rms'] = math.degrees(L[fl, c['q']].std())
    out['acc_std'] = float(L[fl, c['acc_norm']].std())
    path = L[fl, c['solver_code']].astype(int) // 10
    out['path2'] = float((path == 2).mean()); out['path1'] = float((path == 1).mean())
    w = from_log(L, c, fl)
    out['wrong'] = 0.5 * (np.nan_to_num(w['roll_wrong']) + np.nan_to_num(w['pitch_wrong']))
    return out


def real_metrics():
    rows = []
    for v, a, b in REAL:
        c, D = load_real(REAL_DIR + v + '.txt'); tt = D[:, c['t']]; m = (tt >= a) & (tt < b)
        U = D[np.ix_(m, [c[k] for k in MOT])]
        path = D[m, c['solver_code']].astype(int) // 10
        w = from_log(D, c, m)
        rows.append(dict(name=v, p_rms=math.degrees(D[m, c['p']].std()), q_rms=math.degrees(D[m, c['q']].std()),
                         acc_std=float(D[m, c['acc_norm']].std()), path2=float((path == 2).mean()),
                         du_max=float(np.abs(np.diff(U, axis=0)).max()),
                         du_rms=float(np.sqrt(np.mean(np.diff(U, axis=0) ** 2))),
                         wrong=0.5 * (np.nan_to_num(w['roll_wrong']) + np.nan_to_num(w['pitch_wrong']))))
    return rows


def all_runs():
    return [run_metrics(p) for p in sorted(glob.glob(os.path.join(HERE, '*.npz')))]


KEYS = ['crash', 'roll_true_07', 'roll_true_17', 'roll_est_17', 'pitch_true_07', 'pitch_true_17', 'est_err_rms',
        'est_err_max', 'hover_true_rms', 'true_err_max', 'vh_max', 'drift_m', 'du_max', 'du_p99', 'du_rms', 'p_rms',
        'acc_std', 'path2', 'wrong']


def group(rows, prefix_filter=None):
    g = {}
    for r in rows:
        key = r['name'].rsplit('_s', 1)[0]
        g.setdefault(key, []).append(r)
    return g


if __name__ == '__main__':
    rows = all_runs()
    with open(os.path.join(HERE, 'plant_metrics.csv'), 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=['name', 'plant', 'cond', 'seed', 'set'] + KEYS); wr.writeheader()
        for r in rows: wr.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items() if k in wr.fieldnames})
    g = group(rows)
    order = [f'{p}_{c}' for c in ('ideal', 'real') for p in LADDER] + sorted(k for k in g if k.startswith('bfx'))
    print('group n crash | ' + ' '.join(KEYS[1:]))
    for k in order:
        if k not in g: continue
        R = g[k]
        vals = [np.nanmean([r[x] for r in R]) for x in KEYS[1:]]
        print(f"{k:16s} {len(R)} {sum(bool(r['crash']) for r in R)} | " + ' '.join(f'{v:.3g}' for v in vals))
    print('実機:')
    for r in real_metrics():
        print(f"  {r['name']}: p_rms {r['p_rms']:.1f} acc_std {r['acc_std']:.2f} path2 {r['path2']:.3f} du_max {r['du_max']:.3f} du_rms {r['du_rms']:.4f} wrong {r['wrong']:.3f}")
