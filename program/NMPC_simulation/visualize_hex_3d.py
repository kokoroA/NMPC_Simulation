"""
visualize_hex_3d.py
====================
Hexa-X シミュレーション結果の 3D 姿勢・軌跡アニメーション (matplotlib).

左: 全体軌跡ビュー  /  右: 機体追尾・拡大姿勢ビュー (方針 A)

使い方:
    python visualize_hex_3d.py              # S3a を実行して 3D 表示
    python visualize_hex_3d.py --scenario S4
"""

from __future__ import annotations

import argparse
from typing import List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from nmpc_simulation import (
    PI_PSI, PI_X, PI_Y, PI_Z,
    FaultManager,
    HexaModel,
    Logger,
    Params,
    phi_step_schedule,
    run_scenario,
)

# 機体追尾ビューの半幅 [m]（中心 ±BODY_HALF の立方体にクリップ）
DEFAULT_BODY_HALF = 0.15
# 機体座標軸の表示長 [m]
BODY_AXIS_LEN = 0.10


def rotation_body_to_ned(phi: float, theta: float, psi: float) -> np.ndarray:
    """Body → NED 回転行列 R = Rz(ψ) @ Ry(θ) @ Rx(φ)."""
    cφ, sφ = np.cos(phi), np.sin(phi)
    cθ, sθ = np.cos(theta), np.sin(theta)
    cψ, sψ = np.cos(psi), np.sin(psi)
    return np.array([
        [ cθ * cψ,  sφ * sθ * cψ - cφ * sψ,  cφ * sθ * cψ + sφ * sψ],
        [ cθ * sψ,  sφ * sθ * sψ + cφ * cψ,  cφ * sθ * sψ - sφ * cψ],
        [-sθ,       cφ * cθ,                  -sφ * cθ                ],
    ])


def ned_to_enu(p_ned: np.ndarray) -> np.ndarray:
    """NED (z 下向き正) → ENU 表示座標 (z 上向き正)."""
    p = np.asarray(p_ned, dtype=np.float64).reshape(3)
    return np.array([p[0], p[1], -p[2]])


def hex_body_points(model: HexaModel) -> np.ndarray:
    """6 モータ位置 (body frame, z=0 平面). shape (6, 3)."""
    motors = np.zeros((6, 3), dtype=np.float64)
    for k in range(6):
        motors[k] = [model.x_motor[k], model.y_motor[k], 0.0]
    return motors


def frame_geometry(xi: np.ndarray,
                   motors_b: np.ndarray,
                   R: np.ndarray,
                   arm_length: float) -> Tuple[np.ndarray, List[np.ndarray],
                                                 np.ndarray, np.ndarray,
                                                 np.ndarray, np.ndarray,
                                                 np.ndarray]:
    """1 フレーム分の機体幾何 (ENU)."""
    z_ned = float(xi[PI_Z])
    center_ned = np.array([float(xi[PI_X]), float(xi[PI_Y]), z_ned])
    center_enu = ned_to_enu(center_ned)
    motors_enu = [ned_to_enu(center_ned + R @ motors_b[k]) for k in range(6)]
    nose_enu = ned_to_enu(center_ned + R @ np.array([arm_length * 1.4, 0.0, 0.0]))
    # body 軸端点 (NED body: x前, y右, z下 → 上向きは -z)
    axis_len = BODY_AXIS_LEN
    axis_x = ned_to_enu(center_ned + R @ np.array([axis_len, 0.0, 0.0]))
    axis_y = ned_to_enu(center_ned + R @ np.array([0.0, axis_len, 0.0]))
    axis_z_up = ned_to_enu(center_ned + R @ np.array([0.0, 0.0, -axis_len]))
    return center_enu, motors_enu, nose_enu, axis_x, axis_y, axis_z_up, center_ned


def _set_line3d(line, p0: np.ndarray, p1: np.ndarray) -> None:
    line.set_data([p0[0], p1[0]], [p0[1], p1[1]])
    line.set_3d_properties([p0[2], p1[2]])


def _update_hex_on_ax(center_enu: np.ndarray,
                      motors_enu: List[np.ndarray],
                      nose_enu: np.ndarray,
                      arm_lines: List,
                      motor_scatters: List,
                      center_pt,
                      nose_line,
                      u_frame: np.ndarray) -> None:
    cx, cy, cz = center_enu
    center_pt.set_data([cx], [cy])
    center_pt.set_3d_properties([cz])
    _set_line3d(nose_line, center_enu, nose_enu)
    for k in range(6):
        mx, my, mz = motors_enu[k]
        _set_line3d(arm_lines[k], center_enu, motors_enu[k])
        motor_scatters[k]._offsets3d = ([mx], [my], [mz])
        alpha = 0.25 + 0.75 * float(np.clip(u_frame[k], 0.0, 1.0))
        motor_scatters[k].set_alpha(alpha)


def _make_hex_artists(ax, arm_lw: float = 2.0, motor_s: int = 40):
    """ヘキサ機体の Line3D / scatter を生成."""
    arm_colors = plt.cm.tab10(np.linspace(0, 1, 6))
    arm_lines, motor_scatters = [], []
    for k in range(6):
        ln, = ax.plot([], [], [], color=arm_colors[k], lw=arm_lw)
        arm_lines.append(ln)
        sc = ax.scatter([], [], [], s=motor_s, color=arm_colors[k], depthshade=True)
        motor_scatters.append(sc)
    center_pt, = ax.plot([], [], [], 'ko', ms=6, zorder=10)
    nose_line, = ax.plot([], [], [], 'r-', lw=2.5, label='body +x')
    return arm_lines, motor_scatters, center_pt, nose_line


def animate_hex(logger: Logger,
                params: Params,
                *,
                decimate: int = 10,
                title: str = "",
                interval_ms: int = 50,
                body_half: float = DEFAULT_BODY_HALF) -> FuncAnimation:
    """Logger データから 2 パネル 3D アニメーションを生成."""
    d = logger.arrays()
    model = HexaModel(params)
    motors_b = hex_body_points(model)

    t_all = d['t']
    x_all = d['x']
    u_all = d['u_app']
    idx = np.arange(0, len(t_all), max(decimate, 1))
    t = t_all[idx]
    x = x_all[idx]
    u_app = u_all[idx]

    traj = np.array([ned_to_enu([xi[PI_X], xi[PI_Y], xi[PI_Z]]) for xi in x_all])

    fig = plt.figure(figsize=(14, 7))
    if title:
        fig.suptitle(title, fontsize=12)
    ax_traj = fig.add_subplot(121, projection='3d')
    ax_body = fig.add_subplot(122, projection='3d')

    # ---- 左: 全体軌跡 ----
    ax_traj.set_xlabel('North [m]')
    ax_traj.set_ylabel('East [m]')
    ax_traj.set_zlabel('Up [m]')
    ax_traj.set_title('Trajectory (overview)')

    ax_traj.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                 'k--', alpha=0.35, lw=0.8, label='trajectory')

    span = max(0.5, float(np.max(np.abs(traj[:, :2]))) + 0.3)
    alt_max = max(0.5, float(np.max(traj[:, 2])) + 0.3)
    gx = np.linspace(-span, span, 2)
    gy = np.linspace(-span, span, 2)
    gxx, gyy = np.meshgrid(gx, gy)
    gzz = np.zeros_like(gxx)
    ax_traj.plot_surface(gxx, gyy, gzz, alpha=0.12, color='0.7', linewidth=0)

    traj_arm, traj_motors, traj_center, traj_nose = _make_hex_artists(ax_traj)
    ax_traj.set_xlim(-span, span)
    ax_traj.set_ylim(-span, span)
    ax_traj.set_zlim(0.0, alt_max)
    ax_traj.view_init(elev=25, azim=-60)
    ax_traj.legend(loc='upper left', fontsize=7)

    # ---- 右: 機体追尾・拡大姿勢 ----
    ax_body.set_xlabel('North [m]')
    ax_body.set_ylabel('East [m]')
    ax_body.set_zlabel('Up [m]')
    ax_body.set_title(f'Attitude (zoom ±{body_half:.2f} m, body-follow)')

    body_arm, body_motors, body_center, body_nose = _make_hex_artists(
        ax_body, arm_lw=3.0, motor_s=80)

    # body 座標軸 (RGB)
    axis_x_line, = ax_body.plot([], [], [], 'r-', lw=3.0, label='body x')
    axis_y_line, = ax_body.plot([], [], [], 'g-', lw=3.0, label='body y')
    axis_z_line, = ax_body.plot([], [], [], 'b-', lw=3.0, label='body up (-z)')

    # 機体高さの局所水平参照面 (薄いグリッド)
    _local_grid_lines: List = []
    grid_n = 5
    for _ in range(grid_n * grid_n * 2):
        ln, = ax_body.plot([], [], [], color='0.75', lw=0.5, alpha=0.5)
        _local_grid_lines.append(ln)

    ax_body.view_init(elev=20, azim=-70)
    ax_body.legend(loc='upper left', fontsize=7)

    status_text = fig.text(0.02, 0.02, '', fontsize=9, family='monospace')

    def _local_grid_at(center_enu: np.ndarray, half: float) -> None:
        """機体中心高さに水平グリッドを配置."""
        cx, cy, cz = center_enu
        xs = np.linspace(cx - half, cx + half, grid_n)
        ys = np.linspace(cy - half, cy + half, grid_n)
        li = 0
        for xg in xs:
            p0 = np.array([xg, ys[0], cz])
            p1 = np.array([xg, ys[-1], cz])
            _set_line3d(_local_grid_lines[li], p0, p1)
            li += 1
        for yg in ys:
            p0 = np.array([xs[0], yg, cz])
            p1 = np.array([xs[-1], yg, cz])
            _set_line3d(_local_grid_lines[li], p0, p1)
            li += 1
        for k in range(li, len(_local_grid_lines)):
            _local_grid_lines[k].set_data([], [])
            _local_grid_lines[k].set_3d_properties([])

    def update(i: int):
        phi, theta, psi = float(x[i, 0]), float(x[i, 1]), float(x[i, PI_PSI])
        R = rotation_body_to_ned(phi, theta, psi)
        geom = frame_geometry(x[i], motors_b, R, params.arm_length)
        (center_enu, motors_enu, nose_enu,
         axis_x, axis_y, axis_z_up, _center_ned) = geom
        cx, cy, cz = center_enu

        _update_hex_on_ax(center_enu, motors_enu, nose_enu,
                          traj_arm, traj_motors, traj_center, traj_nose,
                          u_app[i])
        _update_hex_on_ax(center_enu, motors_enu, nose_enu,
                          body_arm, body_motors, body_center, body_nose,
                          u_app[i])

        _set_line3d(axis_x_line, center_enu, axis_x)
        _set_line3d(axis_y_line, center_enu, axis_y)
        _set_line3d(axis_z_line, center_enu, axis_z_up)

        ax_body.set_xlim(cx - body_half, cx + body_half)
        ax_body.set_ylim(cy - body_half, cy + body_half)
        ax_body.set_zlim(cz - body_half, cz + body_half)
        _local_grid_at(center_enu, body_half)

        alt = -float(x[i, PI_Z])
        status_text.set_text(
            f"t={t[i]:.2f}s  alt={alt:.2f}m  "
            f"pos=({x[i, PI_X]:+.2f}, {x[i, PI_Y]:+.2f}) m\n"
            f"φ={np.degrees(phi):+.2f}°  "
            f"θ={np.degrees(theta):+.2f}°  "
            f"ψ={np.degrees(psi):+.2f}°"
        )

        artists = (traj_arm + traj_motors + body_arm + body_motors
                   + [traj_center, traj_nose, body_center, body_nose,
                      axis_x_line, axis_y_line, axis_z_line, status_text])
        return artists

    anim = FuncAnimation(fig, update, frames=len(t), interval=interval_ms,
                         blit=False, repeat=True)
    fig.tight_layout(rect=[0, 0.04, 1, 0.96] if title else [0, 0.04, 1, 1])
    return anim


def run_demo_scenario(scenario: str,
                      params: Optional[Params] = None,
                      t_end: float = 15.0) -> Tuple[Logger, Params, str]:
    """デモ用に 1 シナリオを実行して Logger を返す."""
    params = params or Params()
    phi_ref = phi_step_schedule([(0.0, 0.0)])
    plan_phi_step = (5.0, 0.0)
    fault_delay = 2.0
    z_target = params.z_ref_default
    dist_mgr = None
    faults: Sequence[Tuple[int, float]] = []
    fdi_known = True
    label = scenario

    if scenario == 'S1':
        label = 'S1: healthy'
    elif scenario == 'S2a':
        faults = [(4, 0.5)]; label = 'S2a: M5 partial η=0.5'
    elif scenario == 'S3a':
        faults = [(4, 0.0)]; label = 'S3a: M5 complete fault'
    elif scenario == 'S4':
        faults = [(4, 0.0), (5, 0.0)]; label = 'S4: M5+M6 fail'
    elif scenario == 'S6':
        from nmpc_simulation import DisturbanceManager
        dist_mgr = DisturbanceManager([
            {'t_start': 6.0, 't_end': 7.0,
             'kind': 'force_world', 'axis': 2, 'magnitude': +0.5},
        ])
        label = 'S6: vertical gust'
    elif scenario == 'S8':
        from nmpc_simulation import DisturbanceManager
        faults = [(4, 0.0)]
        dist_mgr = DisturbanceManager([
            {'t_start': 8.0, 't_end': 9.0,
             'kind': 'torque_body', 'axis': 0, 'magnitude': 0.02},
        ])
        label = 'S8: M5 fault + torque gust'
    else:
        raise ValueError(f"unknown scenario: {scenario}")

    logger, _ = run_scenario(
        params, FaultManager(schedule=[]), phi_ref, fdi_known=fdi_known,
        t_end=t_end, label=label, savepath=None,
        x0=np.zeros(params.NX_plant),
        z_ref_default=z_target,
        dist_mgr=dist_mgr,
        plan_open_loop_ascent=True,
        plan_z_target=z_target,
        plan_fault_delay_after_switch_s=fault_delay,
        plan_faults=list(faults),
        plan_phi_step=plan_phi_step,
    )
    return logger, params, label


def main() -> None:
    parser = argparse.ArgumentParser(description='Hexa-X 3D visualization')
    parser.add_argument('--scenario', default='S3a',
                        help='S1, S2a, S3a, S4, S6, S8')
    parser.add_argument('--decimate', type=int, default=10,
                        help='フレーム間引き (1kHz ログを N 倍間引き)')
    parser.add_argument('--interval', type=int, default=50,
                        help='アニメーション間隔 [ms]')
    parser.add_argument('--body-half', type=float, default=DEFAULT_BODY_HALF,
                        help='機体追尾ビューの半幅 [m]')
    parser.add_argument('--t-end', type=float, default=15.0)
    args = parser.parse_args()

    print(f"Running scenario {args.scenario} ...")
    logger, params, label = run_demo_scenario(args.scenario, t_end=args.t_end)
    print("Starting 3D animation (close window to exit).")
    animate_hex(logger, params, decimate=args.decimate, title=label,
                interval_ms=args.interval, body_half=args.body_half)
    plt.show()


if __name__ == '__main__':
    main()
