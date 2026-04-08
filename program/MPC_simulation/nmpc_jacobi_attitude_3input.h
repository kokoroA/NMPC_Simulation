#pragma once

/* 姿勢制御用 NMPC ヤコビアン (5次元サブシステム, 3入力版)
   状態: [phi, theta, p, q, r]
   入力: [tau_phi, tau_theta, tau_r] (roll/pitch/yaw 仮想トルク)
   4入力版との違い: モーター推力 T_fr/fl/rr/rl を仮想トルクに置換済み
     tau_phi   = 0.5*l*(T_fl+T_rl-T_fr-T_rr)
     tau_theta = 0.5*l*(T_fr+T_fl-T_rr-T_rl)
     tau_r     = C_Q_T*(T_fr+T_rl-T_fl-T_rr) */

#define NMPC_ATT_NX 5
#define NMPC_ATT_NU 3

/* --- 連続時間 状態ヤコビアン A (5x5) --- */
static inline void calc_jacobian_A_att(double *A, double phi, double theta, double p, double q, double r, double tau_phi, double tau_theta, double tau_r, double Cp, double Cq, double Cr, double Ix, double Iy, double Iz) {
    A[0 * 5 + 0] = q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta);
    A[0 * 5 + 1] = q*(pow(tan(theta), 2) + 1)*sin(phi) + r*(pow(tan(theta), 2) + 1)*cos(phi);
    A[0 * 5 + 2] = 1;
    A[0 * 5 + 3] = sin(phi)*tan(theta);
    A[0 * 5 + 4] = cos(phi)*tan(theta);
    A[1 * 5 + 0] = -q*sin(phi) - r*cos(phi);
    A[1 * 5 + 3] = cos(phi);
    A[1 * 5 + 4] = -sin(phi);
    A[2 * 5 + 2] = -Cp*p*(((p) > 0) - ((p) < 0))/Ix - Cp*fabs(p)/Ix;
    A[2 * 5 + 3] = -r*(-Iy + Iz)/Ix;
    A[2 * 5 + 4] = -q*(-Iy + Iz)/Ix;
    A[3 * 5 + 2] = -r*(Ix - Iz)/Iy;
    A[3 * 5 + 3] = -Cq*q*(((q) > 0) - ((q) < 0))/Iy - Cq*fabs(q)/Iy;
    A[3 * 5 + 4] = -p*(Ix - Iz)/Iy;
    A[4 * 5 + 2] = -q*(-Ix + Iy)/Iz;
    A[4 * 5 + 3] = -p*(-Ix + Iy)/Iz;
    A[4 * 5 + 4] = -Cr*r*(((r) > 0) - ((r) < 0))/Iz - Cr*fabs(r)/Iz;
}

/* --- 連続時間 入力ヤコビアン B (5x3) --- */
/* 注: B は状態・入力に依存しない定数行列になります
       B = diag(1/Ix, 1/Iy, 1/Iz) (上 3x3) + 零行列 (下 2x3) */
static inline void calc_jacobian_B_att(double *B, double phi, double theta, double p, double q, double r, double tau_phi, double tau_theta, double tau_r, double Cp, double Cq, double Cr, double Ix, double Iy, double Iz) {
    B[2 * 3 + 0] = 1.0/Ix;
    B[3 * 3 + 1] = 1.0/Iy;
    B[4 * 3 + 2] = 1.0/Iz;
}
