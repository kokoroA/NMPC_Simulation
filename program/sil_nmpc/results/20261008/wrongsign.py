"""指令モーメントの向きが「姿勢を戻す向き」と逆になっている周期の割合（SIL と実機ログで同じ定義）。

s = k·(角度 − 参照) + 角速度（k = 4 1/s）。|s| が大きいのに指令モーメントが s と同符号なら、
その周期の指令は機体をさらに崩す向き。遅れ補償・モータ遅れで一時的に起こり得るので、
理想条件（雑音なし）の SIL を基準線にして比べる。
"""
import glob, math, os
import numpy as np
from metrics import load_real, REAL, REAL_DIR, MOT

KXM = np.array([0.085, 0.085, 0.0, 0.0, -0.085, -0.085])
KYM = np.array([0.050, -0.050, 0.10, -0.10, 0.050, -0.050])
K = 4.0
S_TH = math.radians(20.0)


def wrong_frac(phi, th, p, q, phr, thr, U):
    s_r = K * (phi - phr) + p
    s_p = K * (th - thr) + q
    Lr = (-KYM * U).sum(1)
    Mp = (KXM * U).sum(1)
    out = {}
    for nm, s, m in (('roll', s_r, Lr), ('pitch', s_p, Mp)):
        big = np.abs(s) > S_TH
        out[nm + '_n'] = int(big.sum())
        out[nm + '_wrong'] = float((np.sign(m[big]) == np.sign(s[big])).mean()) if big.any() else float('nan')
    return out


def from_log(L, c, m):
    U = L[np.ix_(m, [c[k] for k in MOT])]
    return wrong_frac(L[m, c['phi']], L[m, c['theta']], L[m, c['p']], L[m, c['q']],
                      L[m, c['phi_ref']], L[m, c['theta_ref']], U)


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    for path in sorted(glob.glob(os.path.join(here, '*_s[0-9].npz'))):
        d = np.load(path, allow_pickle=True)
        L = d['log']; c = {n: i for i, n in enumerate(d['log_cols'])}
        t = np.arange(len(L)) * 0.01
        m = t >= 4.0
        crash = str(d['crash'])
        if crash:   # 墜落の最後 0.5 s は除く（崩壊後の挙動を数えない）
            m &= t < t[-1] - 0.5
        print(os.path.basename(path)[:-4], from_log(L, c, m))
    for v, a, b in REAL:
        c, D = load_real(REAL_DIR + v + '.txt')
        t = D[:, c['t']]; m = (t >= a) & (t < b)
        print(v, from_log(D, c, m))
