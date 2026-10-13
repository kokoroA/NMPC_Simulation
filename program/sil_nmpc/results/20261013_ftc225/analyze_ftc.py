"""FTC（1 発停止）SIL の集計（2026-10-13）。

    python3 analyze_ftc.py [runs_dir]     → ftc_summary.csv と画面表

1 本ごとの指標（プラント真値。時刻は LOG を入れた時刻が 0）:
  ok        墜落（姿勢 70° 超・接地）もフェイルセーフも無く 19.5 s を飛び切った
  t_end     墜落・フェイルセーフの時刻（飛び切ったら 19.5）
  tilt_max  故障後の最大傾き [deg]（acos(cosφ cosθ)）
  tilt_rms  故障 2 s 後〜12 s の傾き rms [deg]（回復後の静かなホバー）
  alt_drop  故障時の高度 − 故障後の最低高度 [m]（ground は 1 m − 最低高度、4 s 以降）
  r_mean / r_max  故障後のヨー率の平均 / 最大 |r| [deg/s]
  carry_max / carry_sat  担当 4 発の duty 最大 / 0.949 以上の割合
  diag_mean 対角発の duty 平均
  est_err   EKF の傾き推定と真値の差の最大 [deg]
  roll_trk / pitch_trk  5° 指令（12–14 s / 16–18 s）の後半 1 s の平均 [deg]
"""
import glob
import math
import os
import re
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MOTORS = ["FR", "FL", "MR", "ML", "RR", "RL"]
DIAG = [5, 4, 3, 2, 1, 0]
T_FAULT = 7.0


def one(path):
    d = np.load(path, allow_pickle=True)
    P = d['plant']; c = {k: i for i, k in enumerate(d['plant_cols'])}
    L = d['log']; l = {k: i for i, k in enumerate(d['log_cols'])}
    t = P[:, c['t']] - P[0, c['t']]
    mode = str(d['mode']); mi = MOTORS.index(str(d['motor'])); di = DIAG[mi]
    crash = str(d['crash']); fs = int(d['status'][2]) if len(d['status']) > 2 else 0
    fsl = L[:, l['failsafe']] if len(L) else np.zeros(1)
    fs_any = int(np.max(fsl)) if len(fsl) else 0
    t_end = 19.5
    m = re.search(r't=([\d.]+)', crash)
    if m: t_end = float(m.group(1))
    if fs_any:
        t_end = min(t_end, float(np.argmax(fsl > 0)) * 0.01)
    tf = {"air": T_FAULT, "ground": 4.0, "healthy": T_FAULT}[mode]
    phi, th = P[:, c['phi']], P[:, c['theta']]
    tilt = np.degrees(np.arccos(np.clip(np.cos(phi) * np.cos(th), -1, 1)))
    post = t >= tf
    out = dict(ok=int(crash == '' and fs_any == 0), t_end=t_end, fs=fs_any,
               tilt_max=float(tilt[post].max()) if post.any() else np.nan)
    w = (t >= tf + 2.0) & (t < 12.0)
    out['tilt_rms'] = float(np.sqrt(np.mean(tilt[w] ** 2))) if w.sum() > 100 and t_end >= 12.0 else np.nan
    alt = P[:, c['alt']]
    a0 = alt[np.searchsorted(t, tf)] if mode != 'ground' else 1.0
    out['alt_drop'] = float(a0 - alt[post].min()) if post.any() else np.nan
    r = np.degrees(P[:, c['r']])
    out['r_mean'] = float(np.mean(r[post])); out['r_max'] = float(np.max(np.abs(r[post])))
    D = np.array([P[:, c[f'd{i}']] for i in range(6)]).T
    carry = [j for j in range(6) if j not in (mi, di)] if mode != 'healthy' else list(range(6))
    out['carry_max'] = float(D[post][:, carry].max())
    out['carry_sat'] = float((D[post][:, carry] >= 0.949).mean())
    out['diag_mean'] = float(D[post][:, di].mean())
    # EKF の推定誤差（100 Hz ログとプラントを時刻で合わせる）
    if len(L):
        tl = np.arange(len(L)) * 0.01
        ip = np.clip(np.searchsorted(t, tl), 0, len(t) - 1)
        e = np.maximum(np.abs(np.degrees(L[:, l['phi']] - phi[ip])), np.abs(np.degrees(L[:, l['theta']] - th[ip])))
        out['est_err'] = float(e[tl >= tf].max()) if (tl >= tf).any() else np.nan
    else:
        out['est_err'] = np.nan
    def trk(a, b, col):
        ww = (t >= a) & (t < b)
        return float(np.degrees(np.mean(P[ww, c[col]]))) if ww.sum() > 100 and t_end >= b else np.nan
    out['roll_trk'] = trk(13.0, 14.0, 'phi'); out['pitch_trk'] = trk(17.0, 18.0, 'theta')
    return out


def group_key(fn):
    return re.sub(r'_s\d+\.npz$', '', os.path.basename(fn))


def main():
    rd = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'runs')
    G = defaultdict(list)
    for f in sorted(glob.glob(os.path.join(rd, '*.npz'))):
        G[group_key(f)].append(one(f))
    keys = ['ok', 't_end', 'tilt_max', 'tilt_rms', 'alt_drop', 'r_mean', 'r_max', 'carry_max', 'carry_sat', 'diag_mean', 'est_err', 'roll_trk', 'pitch_trk']
    rows = []
    for g, lst in G.items():
        A = {k: np.array([x[k] for x in lst], float) for k in keys}
        rows.append((g, len(lst), A))
    with open(os.path.join(HERE, 'ftc_summary.csv'), 'w') as f:
        f.write('group,n,' + ','.join(f'{k}_med,{k}_min,{k}_max' if k != 'ok' else 'ok_count' for k in keys) + '\n')
        for g, n, A in rows:
            vals = []
            for k in keys:
                if k == 'ok':
                    vals.append(f'{int(np.nansum(A[k]))}')
                else:
                    x = A[k]
                    vals += [f'{np.nanmedian(x):.4g}' if np.isfinite(x).any() else 'nan',
                             f'{np.nanmin(x):.4g}' if np.isfinite(x).any() else 'nan',
                             f'{np.nanmax(x):.4g}' if np.isfinite(x).any() else 'nan']
            f.write(f'{g},{n},' + ','.join(vals) + '\n')
    hdr = f"{'group':34s} {'ok':>5s} {'t_end':>6s} {'tiltmax':>8s} {'tiltrms':>8s} {'altdrop':>8s} {'r_mean':>7s} {'r_max':>6s} {'cmax':>5s} {'csat%':>6s} {'diag':>5s} {'esterr':>6s} {'rollT':>6s} {'pitT':>6s}"
    print(hdr)
    for g, n, A in rows:
        med = lambda k: np.nanmedian(A[k]) if np.isfinite(A[k]).any() else float('nan')
        print(f"{g:34s} {int(np.nansum(A['ok'])):>2d}/{n:<2d} {np.nanmin(A['t_end']):6.2f} {med('tilt_max'):8.1f} {med('tilt_rms'):8.2f} {med('alt_drop'):8.2f} "
              f"{med('r_mean'):7.0f} {med('r_max'):6.0f} {med('carry_max'):5.2f} {100*med('carry_sat'):6.1f} {med('diag_mean'):5.2f} {med('est_err'):6.1f} {med('roll_trk'):6.1f} {med('pitch_trk'):6.1f}")


if __name__ == '__main__':
    main()


# ---- 指令の向き（ログ = ファームの推定値と指令 duty）----
KX = np.array([0.085, 0.085, 0, 0, -0.085, -0.085])
KY = np.array([0.05, -0.05, 0.10, -0.10, 0.05, -0.05])
KS = np.array([-1, 1, 1, -1, -1, 1.0])          # nmpc_model.cpp kS
LMOT = ['u_fr', 'u_fl', 'u_mr', 'u_ml', 'u_rr', 'u_rl']


def cmd_stats(path, t0=T_FAULT, t1=12.0):
    """故障後 [t0, t1) の 100 Hz ログから:
    wrong_roll / wrong_pitch: s = 4(角度−参照)+角速度 が 20 deg/s を超える周期のうち、指令モーメントが s と同符号（姿勢を崩す向き）の割合
    yaw_oppose: |r| > 20 deg/s の周期のうち、指令ヨートルク Σ kS·u が r と逆符号（自転を止める向き）の割合"""
    d = np.load(path, allow_pickle=True)
    L = d['log']; l = {k: i for i, k in enumerate(d['log_cols'])}
    tl = np.arange(len(L)) * 0.01
    m = (tl >= t0) & (tl < min(t1, tl[-1] - 0.3)) if len(L) else np.zeros(0, bool)
    if m.sum() < 20:
        return dict(wrong_roll=np.nan, wrong_pitch=np.nan, yaw_oppose=np.nan)
    U = L[m][:, [l[k] for k in LMOT]]
    sr = 4 * (L[m, l['phi']] - L[m, l['phi_ref']]) + L[m, l['p']]
    sp = 4 * (L[m, l['theta']] - L[m, l['theta_ref']]) + L[m, l['q']]
    Mr, Mp, Mz = (-KY * U).sum(1), (KX * U).sum(1), (KS * U).sum(1)
    th = math.radians(20)
    def frac(s, M):
        b = np.abs(s) > th
        return float(np.mean(np.sign(M[b]) == np.sign(s[b]))) if b.sum() > 5 else np.nan
    r = L[m, l['r']]; b = np.abs(r) > th
    return dict(wrong_roll=frac(sr, Mr), wrong_pitch=frac(sp, Mp),
                yaw_oppose=float(np.mean(np.sign(Mz[b]) == -np.sign(r[b]))) if b.sum() > 5 else np.nan)
