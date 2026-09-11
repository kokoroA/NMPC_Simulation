"""FR停止 relaxed hover 平衡探索（ベクトル化版）。

decision 20260904_relaxed_hover_equilibrium_design.md 2026-09-05 追記の方法に従う:
  外側で (phi, theta, Omega) を探索し、固定候補に対する推力 f は線形問題として解く。

高速化の鍵: 等式行列 M は「どのモータが死んでいるか」だけで決まり、(phi,theta,Omega) に依存しない。
よって擬似逆行列とヌル空間は 1 回だけ計算し、右辺 b だけをグリッド全体でベクトル化できる。
未知数 5 発・等式 4 本なのでヌル空間は 1 次元。余裕最大化は 10 本の直線の下側包絡線の頂点
（45 通りの交点）を全部評価すれば厳密に解ける。
"""
import numpy as np, math

Ix, Iy, Iz = 1.8e-3, 1.9e-3, 3.00e-3
C_QT = 5.8e-9 / 5.21e-7
G = 9.81
kX = np.array([0.085, 0.085, 0.0, 0.0, -0.085, -0.085])
kY = np.array([0.050, -0.050, 0.10, -0.10, 0.050, -0.050])
kS = np.array([-1., 1, 1, -1, -1, 1])
NAMES = ['FR', 'FL', 'MR', 'ML', 'RR', 'RL']
D90 = lambda u: 0.0904 * u**2 + 0.5431 * u + 0.0243
GEM = lambda u: -0.2265 * u**2 + 1.0525 * u + 0.0452


def solve_grid(PH, TH, OM, m, fmax, Cp, Cq, Cr, dead=(0,)):
    p = -OM * np.sin(TH)
    q = OM * np.sin(PH) * np.cos(TH)
    r = OM * np.cos(PH) * np.cos(TH)
    b = np.stack([
        (Iz - Iy) * q * r + Cp * p * np.abs(p),
        (Ix - Iz) * r * p + Cq * q * np.abs(q),
        (Iy - Ix) * p * q + Cr * r * np.abs(r),
        m * G / (np.cos(PH) * np.cos(TH)),
    ], axis=1)                                        # (Ng, 4)
    idx = [i for i in range(6) if i not in dead]
    M = np.vstack([-kY[idx], kX[idx], kS[idx] * C_QT, np.ones(len(idx))])   # (4, 5)
    fp = b @ np.linalg.pinv(M).T                      # (Ng, 5) 最小ノルム解
    _, s_, vt = np.linalg.svd(M)
    n = vt[np.sum(s_ > 1e-12):][0]                    # (5,) ヌル空間基底

    # 10 本の直線 g_j(s) = a_j + c_j s :  f_i >= 0  と  fmax - f_i >= 0
    a = np.concatenate([fp, fmax - fp], axis=1)       # (Ng, 10)
    c = np.concatenate([np.tile(n, (1, 1)), np.tile(-n, (1, 1))], axis=1)[0]  # (10,)
    ii, jj = np.triu_indices(10, k=1)
    den = c[ii] - c[jj]
    keep = np.abs(den) > 1e-12
    ii, jj, den = ii[keep], jj[keep], den[keep]

    best_m = np.full(len(b), -np.inf); best_s = np.zeros(len(b))
    CH = 4000
    for st in range(0, len(b), CH):
        aa = a[st:st + CH]                            # (nb, 10)
        s = (aa[:, jj] - aa[:, ii]) / den             # (nb, ncand)
        vals = aa[:, None, :] + s[:, :, None] * c[None, None, :]   # (nb, ncand, 10)
        mg = vals.min(axis=2)                         # (nb, ncand)
        k = np.argmax(mg, axis=1)
        rows = np.arange(len(aa))
        best_m[st:st + CH] = mg[rows, k]; best_s[st:st + CH] = s[rows, k]
    f5 = fp + best_s[:, None] * n[None, :]
    F = np.zeros((len(b), 6)); F[:, idx] = f5
    return best_m / fmax, F, np.stack([p, q, r], axis=1)


def scan(m, fmax, Cp, Cq, Cr, tag, dead=(0,), verbose=True):
    ph = np.deg2rad(np.arange(-40, 40.01, 2.0))
    th = np.deg2rad(np.arange(-40, 40.01, 2.0))
    om = np.deg2rad(np.arange(0, 800.01, 10.0))
    PH, TH, OM = [x.ravel() for x in np.meshgrid(ph, th, om, indexing='ij')]
    rho, F, PQR = solve_grid(PH, TH, OM, m, fmax, Cp, Cq, Cr, dead)
    k = int(np.argmax(rho)); d = 180 / math.pi
    if verbose:
        print(f"  {tag}")
        print(f"    最良余裕 rho = {rho[k]*100:+6.1f} %   (負なら平衡解なし)")
        print(f"    phi={PH[k]*d:+5.1f} deg  theta={TH[k]*d:+5.1f} deg  Omega={OM[k]*d:5.0f} deg/s"
              f"   p,q,r = {PQR[k,0]*d:+6.0f}, {PQR[k,1]*d:+6.0f}, {PQR[k,2]*d:+6.0f} deg/s")
        print("    f[N] " + " ".join(f"{nm}={v:.3f}" for nm, v in zip(NAMES, F[k])) +
              f"  合計 {F[k].sum():.3f} N ({F[k].sum()/G*1000:.0f} g)")
    return rho[k], PH[k], TH[k], OM[k], F[k]


if __name__ == "__main__":
    # 使い方: <venv>/bin/python relaxed_hover_equilibrium.py
    # 出典: docs/ai/decisions/20260905_relaxed_hover_eval_cr_decides.md
    print("=== FR停止 relaxed hover 平衡探索 ===")
    print("rho = fmax 正規化した最小余裕。rho>=0 で平衡解あり\n")
    for pname, fn in [("D90s-3", D90), ("GEMFAN-3630-3", GEM)]:
        for duty in (0.95, 1.00):
            for m in (0.350, 0.383):
                scan(m, fn(duty), 0.025, 0.025, 0.005,
                     tag=f"{pname} / duty{duty:.2f} / fmax={fn(duty):.3f} N / m={m*1000:.0f} g")
    print("\n=== Cr 感度 (GEMFAN 0.95, m=350 g, Cp=Cq=0.025) ===")
    print("Cr が未同定であり、可否はここだけで決まる")
    for Cr in (1e-4, 5e-4, 1e-3, 2e-3, 5e-3):
        rho, ph, th, om, F = scan(0.350, GEM(0.95), 0.025, 0.025, Cr, tag="", verbose=False)
        print(f"  Cr={Cr:.1e}  rho={rho*100:+6.1f} %  Omega={om*180/math.pi:5.0f} deg/s  RL={F[5]:.3f} N")
