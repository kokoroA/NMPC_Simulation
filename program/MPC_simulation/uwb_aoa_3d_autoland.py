#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AoA-only 3D Localization + EKF + Outlier Rejection (Anchors=3)
- Start: (0,0,0) -> takeoff to 2m -> straight cruise to (10,0,0) -> land to 0m -> stop
- Measurements: AoA (azimuth + elevation) per anchor, with noise and occasional outliers
- Estimation: Constant-velocity EKF; bearing-only updates with gating + robust weighting
- Plot: legend/info outside; 3D animation (blit=False)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ---------------- WORLD / PATH ----------------
START_POS   = np.array([0.0, 0.0, 0.0], dtype=float)
GOAL_POS    = np.array([20.0, 0.0, 0.0], dtype=float)
CRUISE_ALT  = 2.0

ANCHORS = np.array([
    [ 5.0, 2.0, 0.0],
    [ 10.0, -2.0, 0.0],
    [ 15.0, 2.0, 0.0],
], dtype=float)

DT      = 0.05
T_TOTAL = 180.0

# ---------------- AUTO TAKEOFF / LAND ----------------
TAKEOFF_ALT   = 2.0
V_TAKEOFF     = 0.8
V_LAND        = 0.6
ALT_TOL       = 0.03
GOAL_TOL_XY   = 0.25
GOAL_TOL_STOP = 0.10

# ---------------- STRAIGHT CRUISE ----------------
CRUISE_SPEED = 1.0
WIND_MEAN   = np.array([0.0, 0.0, 0.0])
WIND_TAU    = 10.0
WIND_SIGMA  = 0.02

# ---------------- SIMPLE PHYSICS ----------------
MASS      = 1.2
DRAG_COEF = 0.6
TAU_V     = 0.5

# ---------------- AOA MODEL ----------------
AOA_AZ_STD   = np.deg2rad(2.0)   # 1σ
AOA_EL_STD   = np.deg2rad(2.0)   # 1σ
AOA_OUTLIER_PROB    = 0.02
AOA_OUTLIER_BIAS_AZ = np.deg2rad(8.0)
AOA_OUTLIER_BIAS_EL = np.deg2rad(6.0)

# --------------- EKF (CV model) ----------------
Q_POS = 1e-4
Q_VEL = 2e-2
Q = np.diag([Q_POS, Q_POS, Q_POS, Q_VEL, Q_VEL, Q_VEL])

# ゲーティング/ロバスト化
GATE_DEG      = 5.0      # 角度残差の許容（度）
HUBER_DELTA_D = 3.0      # Huberロスのデルタ（度）

# ---------------- INIT ----------------
X0_TRUE = np.array([*START_POS, 0.0, 0.0, 0.0], dtype=float)
X0_EKF  = np.array([START_POS[0]+0.3, START_POS[1]-0.3, 0.2, 0.0, 0.0, 0.0], dtype=float)
P0_EKF  = np.diag([0.5,0.5,0.5, 0.5,0.5,0.5])**2

# ---------------- UTILS ----------------
def unit(v, eps=1e-12):
    n = np.linalg.norm(v)
    return v / (n + eps)

def step_wind(w, rng):
    drift = -(w - WIND_MEAN) / WIND_TAU
    diffusion = WIND_SIGMA * rng.normal(0.0, 1.0, size=3)
    return w + DT * drift + np.sqrt(DT) * diffusion

def guidance_velocity(now_pos):
    path_vec = GOAL_POS - START_POS
    d_xy = unit(path_vec[:2])
    v_forward_xy = d_xy * CRUISE_SPEED
    pos_rel_xy = now_pos[:2] - START_POS[:2]
    d = d_xy.reshape(2,1)
    e_perp_xy = ((np.eye(2) - d @ d.T) @ pos_rel_xy.reshape(2,1)).ravel()
    k_line = 0.9
    v_corr_xy = np.clip(-k_line * e_perp_xy, -0.6, 0.6)
    v_z = np.clip(0.8 * (CRUISE_ALT - now_pos[2]), -0.8, 0.8)
    return np.array([v_forward_xy[0] + v_corr_xy[0],
                     v_forward_xy[1] + v_corr_xy[1],
                     v_z])

def simulate_motion(x, wind, rng, phase):
    if phase == 0:      # TAKEOFF
        v_cmd = np.array([0.0, 0.0, V_TAKEOFF])
    elif phase == 1:    # CRUISE
        v_cmd = guidance_velocity(x[:3])
    elif phase == 2:    # LAND
        dx, dy = (GOAL_POS[0] - x[0]), (GOAL_POS[1] - x[1])
        vxy = np.clip(0.8 * np.array([dx, dy]), -0.6, 0.6)
        v_cmd = np.array([vxy[0], vxy[1], -V_LAND])
    else:
        v_cmd = np.zeros(3)

    a_ctrl = (v_cmd - x[3:]) / TAU_V
    a_drag = -(DRAG_COEF / MASS) * (x[3:] - wind)
    x[3:] = x[3:] + DT * (a_ctrl + a_drag)
    x[:3] = x[:3] + DT * x[3:]
    if x[2] < 0.0:
        x[2] = 0.0; x[5] = 0.0
    x[:3] += rng.normal(0.0, 1e-3, size=3)
    return x

# ------- AoA helpers -------
def vec_to_azel(v):
    vx, vy, vz = v
    az = np.arctan2(vy, vx)
    el = np.arctan2(vz, np.hypot(vx, vy))
    return az, el

def measure_aoa(p, anchors, rng):
    azs = []; els = []
    for ai in anchors:
        v = p - ai
        az, el = vec_to_azel(v)
        az += rng.normal(0.0, AOA_AZ_STD)
        el += rng.normal(0.0, AOA_EL_STD)
        if rng.random() < AOA_OUTLIER_PROB:
            az += rng.normal(np.sign(rng.normal())*AOA_OUTLIER_BIAS_AZ, np.deg2rad(2.0))
            el += rng.normal(np.sign(rng.normal())*AOA_OUTLIER_BIAS_EL, np.deg2rad(2.0))
        az = (az + np.pi) % (2*np.pi) - np.pi
        el = np.clip(el, -np.pi/2, np.pi/2)
        azs.append(az); els.append(el)
    return np.array(azs), np.array(els)

def ang_wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi

# ------- EKF -------
def ekf_predict(x, P):
    F = np.eye(6); F[0,3]=DT; F[1,4]=DT; F[2,5]=DT
    x = F @ x
    P = F @ P @ F.T + Q
    return x, P

def jacobian_az_el_wrt_pos(p_rel):
    """ H_pos: 2x3 Jacobian of [az, el] w.r.t. position (x,y,z) """
    x, y, z = p_rel
    r_xy2 = x*x + y*y
    r = np.sqrt(r_xy2 + z*z) + 1e-12
    r_xy = np.sqrt(r_xy2) + 1e-12

    # az = atan2(y, x)
    d_az_dx = -y / r_xy2
    d_az_dy =  x / r_xy2
    d_az_dz =  0.0

    # el = atan2(z, sqrt(x^2+y^2))
    d_el_dx = -x*z / (r_xy * r*r)
    d_el_dy = -y*z / (r_xy * r*r)
    d_el_dz =  r_xy / (r*r)

    Hpos = np.array([[d_az_dx, d_az_dy, d_az_dz],
                     [d_el_dx, d_el_dy, d_el_dz]], dtype=float)
    return Hpos

def huber_weight(res_deg, delta_deg):
    """Huber重み：|r|<=δ→1、|r|>δ→δ/|r|"""
    a = np.abs(res_deg)
    w = np.ones_like(a)
    mask = a > delta_deg
    w[mask] = delta_deg / a[mask]
    return w

def ekf_update_aoa(x, P, anchors, azs, els):
    used = 0
    res_list = []
    for ai, z_az, z_el in zip(anchors, azs, els):
        p_rel = x[:3] - ai
        # 予測角
        h_az, h_el = vec_to_azel(p_rel)
        # 角度残差（wrap）
        y_az = ang_wrap(z_az - h_az)
        y_el = ang_wrap(z_el - h_el)
        # ゲーティング
        if (np.degrees(np.abs(y_az)) > GATE_DEG) or (np.degrees(np.abs(y_el)) > GATE_DEG):
            continue
        used += 1
        res_list.extend([np.degrees(y_az), np.degrees(y_el)])

        # ヤコビアン（角→位置）, 角は速度に依存しない
        Hpos = jacobian_az_el_wrt_pos(p_rel)
        H = np.zeros((2,6)); H[:, :3] = Hpos

        # 測定ノイズ共分散（ロバスト重みをRに反映）
        R = np.diag([AOA_AZ_STD**2, AOA_EL_STD**2])
        # Huber重み
        w = huber_weight(np.array([np.degrees(y_az), np.degrees(y_el)]), HUBER_DELTA_D)
        W = np.diag(w)  # 2x2
        R_eff = np.linalg.inv(W) @ R  # = R / w_i（大きい残差ほどRを大きく）

        # EKF更新
        S = H @ P @ H.T + R_eff
        K = P @ H.T @ np.linalg.inv(S)
        y = np.array([y_az, y_el])
        x = x + K @ y
        P = (np.eye(6) - K @ H) @ P

    res_rms = np.sqrt(np.mean(np.square(res_list))) if res_list else 0.0
    return x, P, used, res_rms

# ---------------- 3D ANIMATION ----------------
PHASE_TAKEOFF, PHASE_CRUISE, PHASE_LAND, PHASE_DONE = range(4)

class AOAEKFSim3D:
    def __init__(self):
        self.rng = np.random.default_rng()
        self.wind = WIND_MEAN.copy()
        self.x_true = X0_TRUE.copy()
        self.x_est  = X0_EKF.copy()
        self.P      = P0_EKF.copy()
        self.t = 0.0
        self.phase = PHASE_TAKEOFF
        self.max_steps = int(T_TOTAL / DT)
        self.reached = False

        self.hist_true = []
        self.hist_est  = []

        # Figure (right margin)
        self.fig = plt.figure(figsize=(10.0, 6.6))
        self.fig.subplots_adjust(right=0.72)
        self.ax  = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("AoA-only Localization + EKF (3D)")

        self.ax.scatter(ANCHORS[:,0], ANCHORS[:,1], ANCHORS[:,2], marker='^', s=80, label="Anchors")
        self.ax.scatter([START_POS[0]],[START_POS[1]],[START_POS[2]], marker='o', s=60, label="Start@0m")
        self.ax.scatter([GOAL_POS[0]],[GOAL_POS[1]],[GOAL_POS[2]],   marker='s', s=70, label="Goal@0m")

        self.true_line, = self.ax.plot([], [], [], lw=2, label="True path")
        self.est_line,  = self.ax.plot([], [], [], lw=2, label="EKF path")
        self.true_pt,   = self.ax.plot([], [], [], 'o', label="True")
        self.est_pt,    = self.ax.plot([], [], [], 'x', label="Estimate")

        xs = np.array([ANCHORS[:,0].min(), ANCHORS[:,0].max(), START_POS[0], GOAL_POS[0]])
        ys = np.array([ANCHORS[:,1].min(), ANCHORS[:,1].max(), START_POS[1], GOAL_POS[1]])
        pad = 2.0
        self.ax.set_xlim(xs.min()-pad, xs.max()+pad)
        self.ax.set_ylim(ys.min()-pad, ys.max()+pad)
        self.ax.set_zlim(0.0, CRUISE_ALT + pad)
        self.ax.set_xlabel("X [m]"); self.ax.set_ylabel("Y [m]"); self.ax.set_zlabel("Z [m]")

        self.ax.legend(loc='upper left', bbox_to_anchor=(1.02, 0.78), borderaxespad=0.)
        self.text = self.fig.text(0.73, 0.98, "", va='top', ha='left', family='monospace')

    def _update_phase(self):
        if self.phase == PHASE_TAKEOFF:
            if self.x_true[2] >= TAKEOFF_ALT - ALT_TOL:
                self.phase = PHASE_CRUISE
        elif self.phase == PHASE_CRUISE:
            d_xy = np.linalg.norm(self.x_true[:2] - GOAL_POS[:2])
            if d_xy < GOAL_TOL_XY:
                self.phase = PHASE_LAND
        elif self.phase == PHASE_LAND:
            d_xy = np.linalg.norm(self.x_true[:2] - GOAL_POS[:2])
            if (self.x_true[2] <= ALT_TOL) and (d_xy < GOAL_TOL_STOP):
                self.phase = PHASE_DONE
                self.reached = True

    def step(self, frame):
        if not plt.fignum_exists(self.fig.number):
            return tuple()
        if self.reached or frame >= self.max_steps:
            return (self.true_line, self.est_line, self.true_pt, self.est_pt, self.text)

        # Phase update & true dynamics
        self._update_phase()
        self.wind   = step_wind(self.wind, self.rng)
        self.x_true = simulate_motion(self.x_true, self.wind, self.rng, self.phase)
        self.t     += DT

        # Measurements
        azs, els = measure_aoa(self.x_true[:3], ANCHORS, self.rng)

        # EKF
        self.x_est, self.P = ekf_predict(self.x_est, self.P)
        self.x_est, self.P, used, res_rms = ekf_update_aoa(self.x_est, self.P, ANCHORS, azs, els)

        # Draw
        self.hist_true.append(self.x_true[:3].copy())
        self.hist_est.append(self.x_est[:3].copy())
        T = np.array(self.hist_true); E = np.array(self.hist_est)

        self.true_line.set_data(T[:,0], T[:,1]); self.true_line.set_3d_properties(T[:,2])
        self.est_line.set_data(E[:,0], E[:,1]);  self.est_line.set_3d_properties(E[:,2])
        self.true_pt.set_data([self.x_true[0]], [self.x_true[1]]); self.true_pt.set_3d_properties([self.x_true[2]])
        self.est_pt.set_data([self.x_est[0]],   [self.x_est[1]]);   self.est_pt.set_3d_properties([self.x_est[2]])

        err3d = np.linalg.norm(self.x_true[:3] - self.x_est[:3])
        dxy   = np.linalg.norm(self.x_true[:2] - GOAL_POS[:2])
        self.text.set_text(
            "phase={ph}  t={t:5.2f}s\n"
            "true=({tx:5.2f},{ty:5.2f},{tz:4.2f}) m\n"
            "est =({ex:5.2f},{ey:5.2f},{ez:4.2f}) m\n"
            "err3D={e:.2f} m  distXY→goal={dxy:.2f} m\n"
            "AoA used={u}/3  RMSres={r:.1f} deg".format(
                ph=["TO","CR","LD","DN"][self.phase], t=self.t,
                tx=self.x_true[0], ty=self.x_true[1], tz=self.x_true[2],
                ex=self.x_est[0],  ey=self.x_est[1],  ez=self.x_est[2],
                e=err3d, dxy=dxy, u=used, r=res_rms
            )
        )

        return (self.true_line, self.est_line, self.true_pt, self.est_pt, self.text)

def main():
    sim = AOAEKFSim3D()
    anim = FuncAnimation(sim.fig, sim.step, frames=int(T_TOTAL/DT), interval=DT*1000, blit=False)
    plt.show()

if __name__ == "__main__":
    main()
