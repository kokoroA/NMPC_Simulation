"""揺れの指標（実機ログと SIL で同じ定義）。静かな区間 = |φ_ref|,|θ_ref| < QUIET_DEG（既定 5°）。
p/q/r の帯域別 rms（0.2–2, 2–5, 5–15, 15–50 Hz）、加速度ノルムの std、EKF の観測補正量 ekf_dphi/dtheta の rms。"""
import glob, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '20261008'))
from metrics import load_real, REAL, REAL_DIR   # noqa
BANDS = [(0.2, 2), (2, 5), (5, 15), (15, 50.1)]
QUIET_DEG = float(os.environ.get('QUIET_DEG', '5'))   # 静かな区間のしきい値（参照の絶対値）


def band_rms(x, fs=100.0):
    x = np.asarray(x, float) - np.mean(x); n = len(x)
    if n < 64: return [np.nan] * len(BANDS)
    X = np.fft.rfft(x * np.hanning(n)); f = np.fft.rfftfreq(n, 1 / fs)
    P = (np.abs(X) ** 2) / np.sum(np.hanning(n) ** 2) * 2 / fs
    df = f[1] - f[0]
    return [math.sqrt(P[(f >= lo) & (f < hi)].sum() * df) for lo, hi in BANDS]


def segments(mask, min_len=100):
    """連続した True 区間（min_len 行以上）"""
    idx = np.where(np.diff(np.r_[0, mask.astype(int), 0]))[0]
    return [(a, b) for a, b in zip(idx[::2], idx[1::2]) if b - a >= min_len]


def stats(L, c, m):
    quiet = m & (np.abs(L[:, c['phi_ref']]) < math.radians(QUIET_DEG)) & (np.abs(L[:, c['theta_ref']]) < math.radians(QUIET_DEG))
    segs = segments(quiet)
    out = {}
    for k in ('p', 'q', 'r'):
        x = np.concatenate([L[a:b, c[k]] - L[a:b, c[k]].mean() for a, b in segs]) if segs else np.array([np.nan])
        out[f'{k}_rms'] = math.degrees(float(np.sqrt(np.mean(x ** 2))))
        bs = np.array([band_rms(L[a:b, c[k]]) for a, b in segs]) if segs else np.full((1, 4), np.nan)
        w = np.array([b - a for a, b in segs], float) if segs else np.ones(1)
        bb = np.sqrt((bs ** 2 * w[:, None]).sum(0) / w.sum())
        for (lo, hi), v in zip(BANDS, bb):
            out[f'{k}_{lo:g}_{int(hi)}'] = math.degrees(v)
    seg_all = np.concatenate([np.arange(a, b) for a, b in segs]) if segs else np.array([], int)
    out['acc_std'] = float(L[seg_all, c['acc_norm']].std()) if seg_all.size else np.nan
    out['ekf_dphi_rms'] = math.degrees(float(np.sqrt(np.mean(L[seg_all, c['ekf_dphi']] ** 2)))) if seg_all.size else np.nan
    out['ekf_dth_rms'] = math.degrees(float(np.sqrt(np.mean(L[seg_all, c['ekf_dtheta']] ** 2)))) if seg_all.size else np.nan
    out['quiet_s'] = seg_all.size / 100.0
    return out


def real_stats():
    res = {}
    for v, a, b in REAL:
        c, D = load_real(REAL_DIR + v + '.txt'); t = D[:, c['t']]
        res[v] = stats(D, c, (t >= a) & (t < b))
    return res


def sil_stats(path):
    d = np.load(path, allow_pickle=True); L = d['log']; c = {n: i for i, n in enumerate(d['log_cols'])}
    t = np.arange(len(L)) * 0.01
    end = t[-1] - (0.5 if str(d['crash']) else 0)
    return stats(L, c, (t >= 4.0) & (t <= end))


KEYS = ['p_rms', 'q_rms', 'r_rms', 'p_0.2_2', 'p_2_5', 'p_5_15', 'p_15_50', 'acc_std', 'ekf_dphi_rms', 'ekf_dth_rms']

if __name__ == '__main__':
    R = real_stats()
    print('name ' + ' '.join(KEYS))
    for k, v in R.items(): print(k, ' '.join(f'{v[x]:.2f}' for x in KEYS), f"(quiet {v['quiet_s']:.1f}s)")
    for pat in sys.argv[1:]:
        S = [sil_stats(p) for p in sorted(glob.glob(pat))]
        print(os.path.basename(pat), ' '.join(f'{np.nanmean([s[x] for s in S]):.2f}' for x in KEYS), f'n={len(S)}')
