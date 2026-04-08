#pragma once

#define NMPC_NX 9
#define NMPC_NU 4

/* --- 連続時間 状態ヤコビアン A (9x9) --- */
static inline void calc_jacobian_A(double *A, double u, v, w, p, q, r, phi, theta, Ze, double T_fr, T_fl, T_rr, T_rl, double g, double m, double Cu, double Cv, double Cw, double Cp, double Cq, double Cr, double Ix, double Iy, double Iz, double l, double C_Q_T) {
    A[0 * 9 + 0] = -Cu*u*(((u) > 0) - ((u) < 0))/m - Cu*fabs(u)/m;
    A[0 * 9 + 1] = r;
    A[0 * 9 + 2] = -q;
    A[0 * 9 + 4] = -w;
    A[0 * 9 + 5] = v;
    A[0 * 9 + 7] = -g*cos(theta);
    A[1 * 9 + 0] = -r;
    A[1 * 9 + 1] = -Cv*v*(((v) > 0) - ((v) < 0))/m - Cv*fabs(v)/m;
    A[1 * 9 + 2] = q;
    A[1 * 9 + 4] = w;
    A[1 * 9 + 5] = -u;
    A[1 * 9 + 6] = g*cos(phi)*cos(theta);
    A[1 * 9 + 7] = -g*sin(phi)*sin(theta);
    A[2 * 9 + 0] = q;
    A[2 * 9 + 1] = -q;
    A[2 * 9 + 2] = -Cw*w*(((w) > 0) - ((w) < 0))/m - Cw*fabs(w)/m;
    A[2 * 9 + 4] = u - v;
    A[2 * 9 + 6] = -g*sin(phi)*cos(theta);
    A[2 * 9 + 7] = -g*sin(theta)*cos(phi);
    A[3 * 9 + 3] = -Cp*p*(((p) > 0) - ((p) < 0))/Ix - Cp*fabs(p)/Ix;
    A[3 * 9 + 4] = -r*(-Iy + Iz)/Ix;
    A[3 * 9 + 5] = -q*(-Iy + Iz)/Ix;
    A[4 * 9 + 3] = -r*(Ix - Iz)/Iy;
    A[4 * 9 + 4] = -Cq*q*(((q) > 0) - ((q) < 0))/Iy - Cq*fabs(q)/Iy;
    A[4 * 9 + 5] = -p*(Ix - Iz)/Iy;
    A[5 * 9 + 3] = -q*(-Ix + Iy)/Iz;
    A[5 * 9 + 4] = -p*(-Ix + Iy)/Iz;
    A[5 * 9 + 5] = -Cr*r*(((r) > 0) - ((r) < 0))/Iz - Cr*fabs(r)/Iz;
    A[6 * 9 + 3] = 1;
    A[6 * 9 + 4] = sin(phi)*tan(theta);
    A[6 * 9 + 5] = cos(phi)*tan(theta);
    A[6 * 9 + 6] = q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta);
    A[6 * 9 + 7] = q*(pow(tan(theta), 2) + 1)*sin(phi) + r*(pow(tan(theta), 2) + 1)*cos(phi);
    A[7 * 9 + 4] = cos(phi);
    A[7 * 9 + 5] = -sin(phi);
    A[7 * 9 + 6] = -q*sin(phi) - r*cos(phi);
    A[8 * 9 + 0] = -sin(theta);
    A[8 * 9 + 1] = sin(phi)*cos(theta);
    A[8 * 9 + 2] = cos(phi)*cos(theta);
    A[8 * 9 + 6] = v*cos(phi)*cos(theta) - w*sin(phi)*cos(theta);
    A[8 * 9 + 7] = -u*cos(theta) - v*sin(phi)*sin(theta) - w*sin(theta)*cos(phi);
}

/* --- 連続時間 入力ヤコビアン B (9x4) --- */
static inline void calc_jacobian_B(double *B, double u, v, w, p, q, r, phi, theta, Ze, double T_fr, T_fl, T_rr, T_rl, double g, double m, double Cu, double Cv, double Cw, double Cp, double Cq, double Cr, double Ix, double Iy, double Iz, double l, double C_Q_T) {
    B[2 * 4 + 0] = -1/m;
    B[2 * 4 + 1] = -1/m;
    B[2 * 4 + 2] = -1/m;
    B[2 * 4 + 3] = -1/m;
    B[3 * 4 + 0] = -0.5*l/Ix;
    B[3 * 4 + 1] = 0.5*l/Ix;
    B[3 * 4 + 2] = -0.5*l/Ix;
    B[3 * 4 + 3] = 0.5*l/Ix;
    B[4 * 4 + 0] = 0.5*l/Iy;
    B[4 * 4 + 1] = 0.5*l/Iy;
    B[4 * 4 + 2] = -0.5*l/Iy;
    B[4 * 4 + 3] = -0.5*l/Iy;
    B[5 * 4 + 0] = C_Q_T/Iz;
    B[5 * 4 + 1] = -C_Q_T/Iz;
    B[5 * 4 + 2] = -C_Q_T/Iz;
    B[5 * 4 + 3] = C_Q_T/Iz;
}
