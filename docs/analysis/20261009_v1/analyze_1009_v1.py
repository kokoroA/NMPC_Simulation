"""1009_v1（Su 毎周期厳密化した版の初飛行）を、それ以前の実機ログと同じ指標で比べる。

    <matplotlib のある python> analyze_1009_v1.py [図の出力ディレクトリ]

飛行区間 = NMPC が動いている周期（iter_us > 0）かつ u_base > 0.3。
"""
import csv, math, os, sys
import numpy as np

FW = '/Users/kokoro/Documents/research_project/program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/'
LOGS = [('1009_v1', FW + 'log/1009_v1.csv', 'Su 毎周期厳密（今回）'),
        ('0919_v8', FW + 'tools/0919_v8.txt', 'TR1/SR2（貨物、墜落）'),
        ('0919_v6', FW + 'tools/0919_v6.txt', 'TR1/SR2'),
        ('0919_v5', FW + 'tools/0919_v5.txt', 'TR1/SR2'),
        ('0911_v10', FW + 'tools/0911_100Hz_v10.csv', 'TR1/SR2（当時最良の体感）')]
MOT = ['u_fr', 'u_fl', 'u_mr', 'u_ml', 'u_rr', 'u_rl']
KXM = np.array([0.085, 0.085, 0.0, 0.0, -0.085, -0.085])
KYM = np.array([0.050, -0.050, 0.10, -0.10, 0.050, -0.050])


def load(path):
    cols, rows, hdr = None, [], {}
    for line in open(path, errors='replace'):
        line = line.strip()
        if line.startswith('#cols:'):
            cols = [x.strip() for x in line[6:].split(',')]; continue
        if line.startswith('#params:'):
            for kv in line[8:].split():
                if '=' in kv: k, v = kv.split('=', 1); hdr[k] = v
        if not line or line.startswith('#'): continue
        sp = line.split(',')
        if cols is None or len(sp) != len(cols): continue
        try: rows.append([float(x) for x in sp])
        except ValueError: pass
    return {n: i for i, n in enumerate(cols)}, np.array(rows), hdr


def band_share(x, lo, hi, fs=100.0):
    x = np.asarray(x, float) - np.mean(x)
    if len(x) < 64: return float('nan')
    X = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2; f = np.fft.rfftfreq(len(x), 1 / fs)
    tot = X[(f >= 0.2)].sum()
    return float(X[(f >= lo) & (f < hi)].sum() / tot) if tot > 0 else float('nan')


def segments(mask, min_len=50):
    idx = np.where(np.diff(np.r_[0, mask.astype(int), 0]))[0]
    return [(a, b) for a, b in zip(idx[::2], idx[1::2]) if b - a >= min_len]


def metrics(c, D):
    fl = (D[:, c['iter_us']] > 0) & (D[:, c['u_base']] > 0.3)
    segs = segments(fl)
    out = dict(flight_s=fl.sum() / 100.0, n_segs=len(segs))
    it = D[fl, c['iter_us']]; lu = D[fl, c['loop_us']]
    path = D[fl, c['solver_code']].astype(int) // 10; box = D[fl, c['solver_code']].astype(int) % 10
    n = path == 0; r = path == 1; s = path == 2
    out.update(iter_mean=it.mean(), iter_norm_mean=it[n | s].mean(), iter_norm_p95=np.percentile(it[n | s], 95),
               iter_norm_max=it[n | s].max(), iter_rb_mean=it[r].mean() if r.any() else np.nan,
               loop_mean=lu.mean(), loop_norm_p95=np.percentile(lu[n | s], 95), loop_max=lu.max(),
               loop_over10=float((lu > 10000).mean()), rebuild_per_s=r.sum() / (fl.sum() / 100.0),
               frac_path0=n.mean(), frac_path1=r.mean(), frac_path2=s.mean(),
               box_any=float((box > 0).mean()), lam_pos=float((D[fl, c['lam_norm']] > 0).mean()),
               ulin_p99=float(np.percentile(D[fl, c['ulin_norm']], 99)), ulin_max=float(D[fl, c['ulin_norm']].max()))
    # 指令
    du_all, hf = [], []
    for a, b in segs:
        U = D[a:b][:, [c[k] for k in MOT]]
        du_all.append(np.abs(np.diff(U, axis=0)).max(1))
        hf.append(np.mean([band_share(np.diff(U[:, j]), 5, 50.1) for j in range(6)]))
    du = np.concatenate(du_all)
    U = D[fl][:, [c[k] for k in MOT]]
    out.update(du_max=du.max(), du_p99=np.percentile(du, 99), du_rms=float(np.sqrt(np.mean(du ** 2))),
               du_hf5=float(np.nanmean(hf)), sat=float(((U >= 0.949) | (U <= 0.001)).mean()))
    # 逆向き指令
    def wrong(ang, rate, ref, M):
        sv = 4 * (D[fl, c[ang]] - D[fl, c[ref]]) + D[fl, c[rate]]
        big = np.abs(sv) > math.radians(20)
        return float((np.sign(M[big]) == np.sign(sv[big])).mean()) if big.any() else float('nan'), int(big.sum())
    wr, nr = wrong('phi', 'p', 'phi_ref', (-KYM * U).sum(1)); wp, npp = wrong('theta', 'q', 'theta_ref', (KXM * U).sum(1))
    out.update(wrong_roll=wr, wrong_pitch=wp, wrong_n=nr + npp)
    # 揺れ・追従
    for k in ('p', 'q', 'r'):
        out[f'{k}_rms'] = math.degrees(float(np.sqrt(np.mean(np.concatenate([D[a:b, c[k]] - D[a:b, c[k]].mean() for a, b in segs]) ** 2))))
    out['p_2_5'] = float(np.nanmean([band_share(D[a:b, c['p']], 2, 5) for a, b in segs]))
    out['q_2_5'] = float(np.nanmean([band_share(D[a:b, c['q']], 2, 5) for a, b in segs]))
    for ax, ref in (('phi', 'phi_ref'), ('theta', 'theta_ref')):
        x = D[fl, c[ax]]; y = D[fl, c[ref]]
        out[f'{ax}_corr'] = float(np.corrcoef(x, y)[0, 1]) if y.std() > 1e-6 else float('nan')
        out[f'{ax}_amp'] = float(x.std() / y.std()) if y.std() > 1e-6 else float('nan')
        out[f'{ax}_err_rms'] = math.degrees(float(np.sqrt(np.mean((x - y) ** 2))))
    lv = fl & (np.abs(D[:, c['phi_ref']]) < math.radians(2)) & (np.abs(D[:, c['theta_ref']]) < math.radians(2))
    out['level_s'] = lv.sum() / 100.0
    out['level_phi_rms'] = math.degrees(float(np.sqrt(np.mean(D[lv, c['phi']] ** 2)))) if lv.any() else float('nan')
    out['level_theta_rms'] = math.degrees(float(np.sqrt(np.mean(D[lv, c['theta']] ** 2)))) if lv.any() else float('nan')
    out['att_max'] = math.degrees(float(np.max(np.abs(D[fl][:, [c['phi'], c['theta']]]))))
    out['ref_max'] = math.degrees(float(np.max(np.abs(D[fl][:, [c['phi_ref'], c['theta_ref']]]))))
    out['ekf_drift_rms'] = math.degrees(float(np.sqrt(np.mean(D[fl, c['ekf_dphi']] ** 2 + D[fl, c['ekf_dtheta']] ** 2))))
    out['failsafe'] = int(np.max(D[:, c['failsafe']]))
    out['u_base_mean'] = float(D[fl, c['u_base']].mean())
    return out, fl


KEYS = ['flight_s', 'n_segs', 'u_base_mean', 'iter_norm_mean', 'iter_norm_p95', 'iter_norm_max', 'iter_rb_mean',
        'loop_mean', 'loop_norm_p95', 'loop_max', 'loop_over10', 'rebuild_per_s', 'frac_path1', 'frac_path2',
        'box_any', 'lam_pos', 'ulin_p99', 'ulin_max', 'du_max', 'du_p99', 'du_rms', 'du_hf5', 'sat',
        'wrong_roll', 'wrong_pitch', 'wrong_n', 'p_rms', 'q_rms', 'r_rms', 'p_2_5', 'q_2_5',
        'phi_corr', 'phi_amp', 'phi_err_rms', 'theta_corr', 'theta_amp', 'theta_err_rms',
        'level_s', 'level_phi_rms', 'level_theta_rms', 'att_max', 'ref_max', 'ekf_drift_rms', 'failsafe']

if __name__ == '__main__':
    HERE = os.path.dirname(os.path.abspath(__file__))
    res = {}
    for name, path, lab in LOGS:
        c, D, hdr = load(path)
        m, fl = metrics(c, D); m['N'] = hdr.get('N', '?'); res[name] = (m, c, D, fl, lab)
    with open(os.path.join(HERE, 'metrics.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['metric'] + [n for n, _, _ in LOGS])
        for k in ['N'] + KEYS: w.writerow([k] + [res[n][0][k] if isinstance(res[n][0][k], str) else round(float(res[n][0][k]), 4) for n, _, _ in LOGS])
    print(f"{'':16s}" + ''.join(f'{n:>11s}' for n, _, _ in LOGS))
    for k in ['N'] + KEYS:
        print(f'{k:16s}' + ''.join(f"{res[n][0][k]:>11}" if isinstance(res[n][0][k], str) else f"{float(res[n][0][k]):>11.4g}" for n, _, _ in LOGS))
