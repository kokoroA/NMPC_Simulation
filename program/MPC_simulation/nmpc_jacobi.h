#ifndef NMPC_JACOBI_H
#define NMPC_JACOBI_H

#include <math.h>
#include <string.h>

// SymPyが出力する sign() 関数をC++で処理するための実装
inline double sign(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

#define NMPC_NX 16
#define NMPC_NU 4

/* 連続時間ヤコビアン A(NX×NX), B(NX×NU) を埋める。A, B は呼び出し側で確保すること。 */
static inline void nmpc_jacobi_AB(
    double *A, double *B,
    double u, double v, double w, double w_fr, double w_fl, double w_rr, double w_rl,
    double p, double q, double r, double phi, double theta, double psi,
    double Xe, double Ye, double Ze,
    double g, double m, double Cu, double Cv, double Cw, double Cp, double Cq, double Cr,
    double Ix, double Iy, double Iz, double l, double K, double Reg, double Iz_m,
    double Qf, double CQ, double D, double CT, double CQ_aero)
{
    memset(A, 0, NMPC_NX * NMPC_NX * sizeof(double));
    memset(B, 0, NMPC_NX * NMPC_NU * sizeof(double));

    A[0 * NMPC_NX + 0] = -Cu*u*(((u) > 0) - ((u) < 0))/m - Cu*fabs(u)/m;
    A[0 * NMPC_NX + 1] = r;
    A[0 * NMPC_NX + 2] = -q;
    A[0 * NMPC_NX + 8] = -w;
    A[0 * NMPC_NX + 9] = v;
    A[0 * NMPC_NX + 11] = -g*cos(theta);
    A[1 * NMPC_NX + 0] = -r;
    A[1 * NMPC_NX + 1] = -Cv*v*(((v) > 0) - ((v) < 0))/m - Cv*fabs(v)/m;
    A[1 * NMPC_NX + 2] = q;
    A[1 * NMPC_NX + 8] = w;
    A[1 * NMPC_NX + 9] = -u;
    A[1 * NMPC_NX + 10] = g*cos(phi)*cos(theta);
    A[1 * NMPC_NX + 11] = -g*sin(phi)*sin(theta);
    A[2 * NMPC_NX + 0] = q;
    A[2 * NMPC_NX + 1] = -q;
    A[2 * NMPC_NX + 2] = -Cw*w*(((w) > 0) - ((w) < 0))/m - Cw*fabs(w)/m;
    A[2 * NMPC_NX + 3] = -2*CT*w_fr/m;
    A[2 * NMPC_NX + 4] = -2*CT*w_fl/m;
    A[2 * NMPC_NX + 5] = -2*CT*w_rr/m;
    A[2 * NMPC_NX + 6] = -2*CT*w_rl/m;
    A[2 * NMPC_NX + 8] = u - v;
    A[2 * NMPC_NX + 10] = -g*sin(phi)*cos(theta);
    A[2 * NMPC_NX + 11] = -g*sin(theta)*cos(phi);
    A[3 * NMPC_NX + 3] = -2*CQ*w_fr/Iz_m - (D + pow(K, 2))/(Iz_m*Reg);
    A[4 * NMPC_NX + 4] = -2*CQ*w_fl/Iz_m - (D + pow(K, 2))/(Iz_m*Reg);
    A[5 * NMPC_NX + 5] = -2*CQ*w_rr/Iz_m - (D + pow(K, 2))/(Iz_m*Reg);
    A[6 * NMPC_NX + 6] = -2*CQ*w_rl/Iz_m - (D + pow(K, 2))/(Iz_m*Reg);
    A[7 * NMPC_NX + 3] = -1.0*CT*l*w_fr/Ix;
    A[7 * NMPC_NX + 4] = 1.0*CT*l*w_fl/Ix;
    A[7 * NMPC_NX + 5] = -1.0*CT*l*w_rr/Ix;
    A[7 * NMPC_NX + 6] = 1.0*CT*l*w_rl/Ix;
    A[7 * NMPC_NX + 7] = -Cp*p*(((p) > 0) - ((p) < 0))/Ix - Cp*fabs(p)/Ix;
    A[7 * NMPC_NX + 8] = -r*(-Iy + Iz)/Ix;
    A[7 * NMPC_NX + 9] = -q*(-Iy + Iz)/Ix;
    A[8 * NMPC_NX + 3] = 1.0*CT*l*w_fr/Iy;
    A[8 * NMPC_NX + 4] = 1.0*CT*l*w_fl/Iy;
    A[8 * NMPC_NX + 5] = -1.0*CT*l*w_rr/Iy;
    A[8 * NMPC_NX + 6] = -1.0*CT*l*w_rl/Iy;
    A[8 * NMPC_NX + 7] = -r*(Ix - Iz)/Iy;
    A[8 * NMPC_NX + 8] = -Cq*q*(((q) > 0) - ((q) < 0))/Iy - Cq*fabs(q)/Iy;
    A[8 * NMPC_NX + 9] = -p*(Ix - Iz)/Iy;
    A[9 * NMPC_NX + 3] = 2*CQ_aero*w_fr/Iz;
    A[9 * NMPC_NX + 4] = -2*CQ_aero*w_fl/Iz;
    A[9 * NMPC_NX + 5] = -2*CQ_aero*w_rr/Iz;
    A[9 * NMPC_NX + 6] = 2*CQ_aero*w_rl/Iz;
    A[9 * NMPC_NX + 7] = -q*(-Ix + Iy)/Iz;
    A[9 * NMPC_NX + 8] = -p*(-Ix + Iy)/Iz;
    A[9 * NMPC_NX + 9] = -Cr*r*(((r) > 0) - ((r) < 0))/Iz - Cr*fabs(r)/Iz;
    A[10 * NMPC_NX + 7] = 1;
    A[10 * NMPC_NX + 8] = sin(phi)*tan(theta);
    A[10 * NMPC_NX + 9] = cos(phi)*tan(theta);
    A[10 * NMPC_NX + 10] = q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta);
    A[10 * NMPC_NX + 11] = q*(pow(tan(theta), 2) + 1)*sin(phi) + r*(pow(tan(theta), 2) + 1)*cos(phi);
    A[11 * NMPC_NX + 8] = cos(phi);
    A[11 * NMPC_NX + 9] = -sin(phi);
    A[11 * NMPC_NX + 10] = -q*sin(phi) - r*cos(phi);
    A[12 * NMPC_NX + 8] = sin(phi)/cos(theta);
    A[12 * NMPC_NX + 9] = cos(phi)/cos(theta);
    A[12 * NMPC_NX + 10] = (q*cos(phi) - r*sin(phi))/cos(theta);
    A[12 * NMPC_NX + 11] = (q*sin(phi) + r*cos(phi))*sin(theta)/pow(cos(theta), 2);
    A[13 * NMPC_NX + 0] = cos(psi)*cos(theta);
    A[13 * NMPC_NX + 1] = sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi);
    A[13 * NMPC_NX + 2] = (sin(phi)*sin(psi) + cos(psi))*sin(theta)*cos(phi);
    A[13 * NMPC_NX + 10] = v*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi)) - w*(sin(phi)*sin(psi) + cos(psi))*sin(phi)*sin(theta) + w*sin(psi)*sin(theta)*pow(cos(phi), 2);
    A[13 * NMPC_NX + 11] = -u*sin(theta)*cos(psi) + v*sin(phi)*cos(psi)*cos(theta) + w*(sin(phi)*sin(psi) + cos(psi))*cos(phi)*cos(theta);
    A[13 * NMPC_NX + 12] = -u*sin(psi)*cos(theta) + v*(-sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi)) + w*(sin(phi)*cos(psi) - sin(psi))*sin(theta)*cos(phi);
    A[14 * NMPC_NX + 0] = sin(psi)*cos(theta);
    A[14 * NMPC_NX + 1] = sin(phi)*sin(psi)*sin(theta) + cos(phi)*cos(psi);
    A[14 * NMPC_NX + 2] = -sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi);
    A[14 * NMPC_NX + 10] = v*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi)) + w*(-sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi));
    A[14 * NMPC_NX + 11] = -u*sin(psi)*sin(theta) + v*sin(phi)*sin(psi)*cos(theta) + w*sin(psi)*cos(phi)*cos(theta);
    A[14 * NMPC_NX + 12] = u*cos(psi)*cos(theta) + v*(sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi)) + w*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi));
    A[15 * NMPC_NX + 0] = -sin(theta);
    A[15 * NMPC_NX + 1] = sin(phi)*cos(theta);
    A[15 * NMPC_NX + 2] = cos(phi)*cos(theta);
    A[15 * NMPC_NX + 10] = v*cos(phi)*cos(theta) - w*sin(phi)*cos(theta);
    A[15 * NMPC_NX + 11] = -u*cos(theta) - v*sin(phi)*sin(theta) - w*sin(theta)*cos(phi);

    B[3 * NMPC_NU + 0] = K/(Iz_m*Reg);
    B[4 * NMPC_NU + 3] = K/(Iz_m*Reg);
    B[5 * NMPC_NU + 1] = K/(Iz_m*Reg);
    B[6 * NMPC_NU + 2] = K/(Iz_m*Reg);
}

#endif /* NMPC_JACOBI_H */
