"""ビルド別（再構築周期 1/10/30/100）× 条件（noise / real_nh）の集計と、再構築からの周期数に対する劣化。"""
import csv, glob, os
import numpy as np
from metrics import sil_run, load_real, REAL, REAL_DIR, MOT
from wrongsign import from_log, wrong_frac

HERE = os.path.dirname(os.path.abspath(__file__))
BUILDS = [('rp1', 'ablation_rp1/'), ('rp10', 'ablation_rp10/'), ('rp30', 'ablation_rp30/'), ('rp100(実機)', '')]
CONDS = ['noise', 'real_nh', 'real']


def runs(prefix, cond):
    out = []
    for p in sorted(glob.glob(os.path.join(HERE, f'{prefix}{cond}_s[0-5].npz'))):
        r = sil_run(p)
        d = np.load(p, allow_pickle=True); L = d['log']; c = {n: i for i, n in enumerate(d['log_cols'])}
        t = np.arange(len(L)) * 0.01; m = t >= 4.0
        if r['crash']: m &= t < t[-1] - 0.5
        r.update(from_log(L, c, m)); r['L'] = L[m]; r['c'] = c
        out.append(r)
    return out


def by_cycles(rows, edges=(0, 10, 20, 40, 60, 80, 101)):
    """再構築からの周期数で区切った |du| p95 と逆向き率"""
    res = []
    for a, b in zip(edges[:-1], edges[1:]):
        du_all, wr = [], []
        for r in rows:
            L, c = r['L'], r['c']
            cyc = L[:, c['cycles']]
            U = L[:, [c[k] for k in MOT]]
            du = np.r_[0, np.abs(np.diff(U, axis=0)).max(1)]
            m = (cyc >= a) & (cyc < b)
            du_all.append(du[m])
            w = wrong_frac(L[m, c['phi']], L[m, c['theta']], L[m, c['p']], L[m, c['q']],
                           L[m, c['phi_ref']], L[m, c['theta_ref']], U[m])
            wr.append((w['roll_n'] * np.nan_to_num(w['roll_wrong']) + w['pitch_n'] * np.nan_to_num(w['pitch_wrong']),
                       w['roll_n'] + w['pitch_n']))
        du_all = np.concatenate(du_all)
        nw = sum(x for x, _ in wr); nn = sum(y for _, y in wr)
        res.append((a, b, float(np.percentile(du_all, 95)) if du_all.size else np.nan,
                    nw / nn if nn else np.nan, int(du_all.size)))
    return res


if __name__ == '__main__':
    table = []
    for lab, pre in BUILDS:
        for cond in CONDS:
            R = runs(pre, cond)
            if not R:   # 毎周期再構築 × 据え置きありは実機で成立しないので走らせていない
                continue
            k = lambda key: float(np.nanmean([r[key] for r in R]))
            mx = lambda key: float(np.nanmax([r[key] for r in R]))
            row = dict(build=lab, cond=cond, n=len(R), crash=sum(bool(r['crash']) for r in R),
                       hover=k('hover_att_rms'), late=k('late_hover_rms'), late_max=mx('late_hover_rms'),
                       roll_step=k('roll_step_rms'), roll_step_max=mx('roll_step_rms'),
                       p_rms=k('p_rms'), q_rms=k('q_rms'), r_rms=k('r_rms'), du_rms=k('du_rms'), du_p99=k('du_p99'),
                       du_max=k('du_max'), wrong_roll=k('roll_wrong'), wrong_pitch=k('pitch_wrong'),
                       path1=k('path1'), path2=k('path2'), box=k('box_any'))
            table.append(row)
            print(' '.join(f'{a}={v:.3g}' if isinstance(v, float) else f'{a}={v}' for a, v in row.items()))
    with open(os.path.join(HERE, 'ablation_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(table[0])); w.writeheader()
        for r in table: w.writerow({a: (round(v, 4) if isinstance(v, float) else v) for a, v in r.items()})
    print('\n再構築からの周期数別（標準ビルド noise+real_nh 12 本）: [a,b) du_p95 wrong n')
    std = runs('', 'noise') + runs('', 'real_nh')
    for x in by_cycles(std): print('  ', x)
    print('実機 0919_v5/v6/v8:')
    for v, a, b in REAL:
        c, D = load_real(REAL_DIR + v + '.txt'); t = D[:, c['t']]; m = (t >= a) & (t < b)
        for x in by_cycles([dict(L=D[m], c=c)]): print('  ', v, x)
