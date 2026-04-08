#pragma once

/* 姿勢制御用 NMPC ヤコビアン (5次元サブシステム)
   状態: [p, q, r, phi, theta]
   入力: [T_fr, T_fl, T_rr, T_rl] */

#define NMPC_ATT_NX 5
#define NMPC_ATT_NU 4

/* --- 連続時間 状態ヤコビアン A (5x5) --- */
static inline void calc_jacobian_A_att(double *A, double p, double q, double r, double phi, double theta, double T_fr, double T_fl, double T_rr, double T_rl, double Cp, double Cq, double Cr, double Ix, double Iy, double Iz, double l, double C_Q_T) {
    A[0 * 5 + 0] = -Cp*p*(((p) > 0) - ((p) < 0))/Ix - Cp*fabs(p)/Ix;
    A[0 * 5 + 1] = -r*(-Iy + Iz)/Ix;
    A[0 * 5 + 2] = -q*(-Iy + Iz)/Ix;
    A[1 * 5 + 0] = -r*(Ix - Iz)/Iy;
    A[1 * 5 + 1] = -Cq*q*(((q) > 0) - ((q) < 0))/Iy - Cq*fabs(q)/Iy;
    A[1 * 5 + 2] = -p*(Ix - Iz)/Iy;
    A[2 * 5 + 0] = -q*(-Ix + Iy)/Iz;
    A[2 * 5 + 1] = -p*(-Ix + Iy)/Iz;
    A[2 * 5 + 2] = -Cr*r*(((r) > 0) - ((r) < 0))/Iz - Cr*fabs(r)/Iz;
    A[3 * 5 + 0] = 1;
    A[3 * 5 + 1] = sin(phi)*tan(theta);
    A[3 * 5 + 2] = cos(phi)*tan(theta);
    A[3 * 5 + 3] = q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta);
    A[3 * 5 + 4] = q*(pow(tan(theta), 2) + 1)*sin(phi) + r*(pow(tan(theta), 2) + 1)*cos(phi);
    A[4 * 5 + 1] = cos(phi);
    A[4 * 5 + 2] = -sin(phi);
    A[4 * 5 + 3] = -q*sin(phi) - r*cos(phi);
}

/* --- 連続時間 入力ヤコビアン B (5x4) --- */
static inline void calc_jacobian_B_att(double *B, double p, double q, double r, double phi, double theta, double T_fr, double T_fl, double T_rr, double T_rl, double Cp, double Cq, double Cr, double Ix, double Iy, double Iz, double l, double C_Q_T) {
    B[0 * 4 + 0] = -0.5*l/Ix;
    B[0 * 4 + 1] = 0.5*l/Ix;
    B[0 * 4 + 2] = -0.5*l/Ix;
    B[0 * 4 + 3] = 0.5*l/Ix;
    B[1 * 4 + 0] = 0.5*l/Iy;
    B[1 * 4 + 1] = 0.5*l/Iy;
    B[1 * 4 + 2] = -0.5*l/Iy;
    B[1 * 4 + 3] = -0.5*l/Iy;
    B[2 * 4 + 0] = C_Q_T/Iz;
    B[2 * 4 + 1] = -C_Q_T/Iz;
    B[2 * 4 + 2] = -C_Q_T/Iz;
    B[2 * 4 + 3] = C_Q_T/Iz;
}
