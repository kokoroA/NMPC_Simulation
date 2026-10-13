"""物理状態 -> センサ生値 の変換。

SIL の最大の落とし穴はここ。実機ドライバが出す「生値」と単位・符号を
厳密に一致させないと、EKF が静かに破綻する。

根拠（control.cpp の sensor_read()）:
    Ax = -acceleration_mg[0] * GRAV * 0.001      X 反転, milli-g
    Ay = -acceleration_mg[1] * GRAV * 0.001      Y 反転, milli-g
    Az =  acceleration_mg[2] * GRAV * 0.001      そのまま
    Wp =  angular_rate_mdps[0] * pi/180/1000     milli-deg/s
    Wq =  angular_rate_mdps[1] * pi/180/1000
    Wr = -angular_rate_mdps[2] * pi/180/1000     Z 反転
    Mx0 = -magnetic_field_mgauss[0]
    My0 =  magnetic_field_mgauss[1]
    Mz0 = -magnetic_field_mgauss[2]

さらに地磁気は control.cpp 内でハードアイアン校正を通る:
    v = zoom * (R @ m0 - center)      (要素積)
    m = normalize(R.T @ v)
合成値を作るには**この校正を逆算**する必要がある。
"""
from __future__ import annotations

import numpy as np

GRAV = 9.80665  # ekf.hpp: #define GRAV (9.80665)

# --- control.cpp の sensor_read() に埋め込まれた校正定数（有効な方） -------
_ROT = np.array([
    [-0.78435472, -0.62015392, -0.01402787],
    [0.61753358, -0.78277935, 0.07686857],
    [-0.05865107, 0.05162955, 0.99694255],
])
_CENTER = np.array([-109.32529343620176, 72.76584808916506, 759.2285249891385])
_ZOOM = np.array([0.002034773458122364, 0.002173892202021849, 0.0021819494099235273])

_MDPS_PER_RADS = 180.0 / np.pi * 1000.0


def quat_from_euler(phi, theta, psi):
    """ZYX(yaw-pitch-roll) オイラー角 -> クォータニオン [q0,q1,q2,q3]."""
    cr, sr = np.cos(phi / 2), np.sin(phi / 2)
    cp, sp = np.cos(theta / 2), np.sin(theta / 2)
    cy, sy = np.cos(psi / 2), np.sin(psi / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def dcm_body_from_ned(q):
    """ekf.cpp observation_equation と同一の方向余弦行列。"""
    q0, q1, q2, q3 = q
    return np.array([
        [q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2)],
        [2*(q1*q2 - q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 + q0*q1)],
        [2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3],
    ])


def _mag_calibration_forward(m0):
    """control.cpp と同じ校正（検証用）。正規化まで行う。"""
    v = _ZOOM * (_ROT @ m0 - _CENTER)
    m = _ROT.T @ v
    return m / np.linalg.norm(m)


def _mag_calibration_inverse(m_target, scale=1.0):
    """校正後に m_target（方向）になる生値 m0 を求める。"""
    v = _ROT @ (np.asarray(m_target, dtype=float) * scale)
    return np.linalg.solve(_ROT, v / _ZOOM + _CENTER)


class SensorModel:
    """機体の物理状態から LSM9DS1 の生値を合成する。"""

    def __init__(self, mag_ned=None):
        if mag_ned is None:
            inc = np.deg2rad(49.0)          # 日本付近の伏角
            mag_ned = np.array([np.cos(inc), 0.0, np.sin(inc)])
        m = np.asarray(mag_ned, dtype=float)
        self.mag_ned = m / np.linalg.norm(m)

    def raw_from_state(self, phi, theta, psi, p=0.0, q=0.0, r=0.0,
                       accel_body_extra=None):
        """(acc_mg[3], gyr_mdps[3], mag_mgauss[3]) を返す。"""
        C = dcm_body_from_ned(quat_from_euler(phi, theta, psi))

        a_body = C @ np.array([0.0, 0.0, GRAV])
        if accel_body_extra is not None:
            a_body = a_body + np.asarray(accel_body_extra, dtype=float)

        acc_mg = [-a_body[0] / GRAV * 1000.0,
                  -a_body[1] / GRAV * 1000.0,
                  +a_body[2] / GRAV * 1000.0]
        gyr_mdps = [+p * _MDPS_PER_RADS,
                    +q * _MDPS_PER_RADS,
                    -r * _MDPS_PER_RADS]

        m0 = _mag_calibration_inverse(C @ self.mag_ned)
        mag_mgauss = [-m0[0], +m0[1], -m0[2]]

        return ([float(x) for x in acc_mg],
                [float(x) for x in gyr_mdps],
                [float(x) for x in mag_mgauss])


def euler_rates_to_body_rates(phi, theta, phi_dot, theta_dot, psi_dot):
    """オイラー角速度 -> 機体角速度 [p, q, r]。

        p = phi_dot - psi_dot*sin(theta)
        q = theta_dot*cos(phi) + psi_dot*cos(theta)*sin(phi)
        r = -theta_dot*sin(phi) + psi_dot*cos(theta)*cos(phi)

    姿勢を動かすときは必ずこれで角速度を作ること。
    角速度 0 のまま姿勢だけ動かすと EKF には「ジャイロバイアス」に見え、
    収束が極端に遅くなる（M1 で実際に踏んだ）。
    """
    sp, cp = np.sin(phi), np.cos(phi)
    st, ct = np.sin(theta), np.cos(theta)
    p = phi_dot - psi_dot * st
    q = theta_dot * cp + psi_dot * ct * sp
    r = -theta_dot * sp + psi_dot * ct * cp
    return p, q, r


def smoothstep(t):
    """0->1 の滑らかな補間（両端で微分 0）。"""
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
