"""
nmpc_simulation_stress.py
=========================
nmpc_simulation.py（NX=8 / N=12 版）の複製に、実機との差を埋める 4 つの悪条件を足したストレス試験版。

    1. 計算遅延      compute_delay_cycles : 状態をサンプルしてから指令が出るまで 1〜3 制御周期
    2. 機体の効き    plant_moment_gain    : プラントの姿勢モーメントだけ NMPC モデルの 2 倍（推力・高度には掛けない）
    3. 再構築の停滞  rebuild_hold_cycles  : フル再構築した周期は 3 周期ぶん指令を保持し、その間 NMPC を呼ばない。
                                            再構築の解は「再構築を始めた時点の状態」から作ったものが 3 周期遅れで出る
    4. ジャイロバイアス sigma_gyro_bias_rw / gyro_bias_att_tau : ランダムウォークするバイアスを p,q,r に足し、
                                            相補フィルタ（時定数 tau）経由で φ,θ の推定にも漏らす（EKF の傾き誤差の代用）

2026-10-06 改: 実機 Benzen_NMPC（NX=5 / N=14 / 遅れ補償 DC6）に合わせた。
    - NMPC 側: NX=5, N=14, Q=1000/1000/80/80/40, R_u=620, terminal ×10, ADMM ρ=1・4 反復, reset_period=100（nmpc_params.hpp と同値）
    - プラント側: 質量 0.285 kg ちょうど（誤差倍率 1.0）、推力は飛行較正曲線
      Tf(u) = 0.8389(−0.2265u² + 1.0525u + 0.0452) [N/モータ]（20260916_bench_thrust_curve_overstates_flight_thrust...）。
      これでホバー duty が実測 0.5506 に一致する。NMPC モデルは実機と同じく線形 T_max=0.75。
    - 遅れ補償 delay_comp_steps（0 / 6）: 実機 nmpc_delay_predict() の移植。直近 n 周期に出した duty（古い順）で
      x0 を RK2・1 周期 4 分割で n 周期前方へ積分し、角度変化 ≥ 0.35 rad か角速度変化 ≥ 10 rad/s なら補償しない。

元の nmpc_simulation.py の S1〜S8 は main() から外し、高度 2 m のホバーから始めて t=8 s にロール 5° のステップを入れる条件行列を回す。
⚠ 元ファイルの open-loop 上昇（pwm_ascent=0.145）は旧パラメータ（T_max=2.725 N, mass=0.200 kg）用で、
  現行の実機パラメータでは離陸できない。このファイルでは上昇フェーズを使わない。
比較用に NX=5（モータ遅れを予測モデルに持たない、遅れ補償なし）も同じ悪条件で回す（--no-nx5 で省略）。

---- 以下は元ファイルの説明 ----
Hexa-X 配置・6 入力 NMPC を ADMM + 凝縮形 SR2 で実装したシミュレータ。
sample/nmpc.cpp + nmpc_model.cpp + nmpc_params.hpp の Python 鏡像 + Hexa 拡張。

クラス構成:
    Params        : physical / NMPC / cost / RTI 定数 (NU=6 拡張)
    HexaModel     : 連続時間ダイナミクス f / Jacobian / B_c 解析形 / RK4 + Euler 離散化
    AdmmNmpcHex   : 凝縮 NMPC コア (Sx, Su, H, SR2 rank-2, ADMM 1-iter, warm-start)
    FaultManager  : スケジュール式 η(t), u_max(t), FDI 検知イベント生成
    NoiseModel    : 真値/観測分離、プロセス・観測ノイズ
    Logger        : 履歴格納 + matplotlib プロット (x 真値, x_meas 観測)
    PKG-D         : 故障時 u_baseline ramp / PID・z_ref bumpless 再起動
    シナリオ S1..S4 + main()

設計判断:
    - 全体フローは sample C++ と 1:1 対応。
    - SR2 ヘシアン更新は C++ では LLT に直接 rank±1 を当てるが、
      Python では SciPy にその関数が無いので H を再構築して再 Cholesky で代用。
    - 数値型は float64 (Python シミュ向けに保守的)。
      Pico 2 移植時の単精度挙動は別途調査。
    - 入力コストは (U - U_ref)^T R_bar (U - U_ref) 形式 (sample 同等)。
      ホバー duty を U_ref に与え、故障時に動的更新。
"""

from __future__ import annotations

import copy
import math
import os
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np
import matplotlib

# Backend: 環境変数 MPLBACKEND が有ればそれを尊重。
# 無い場合は Qt5Agg → TkAgg → Agg の順で fallback (runge_stampfly_mpc_6.py と同じ流儀)。
if "MPLBACKEND" not in os.environ:
    for _be in ("Qt5Agg", "TkAgg", "MacOSX", "Agg"):
        try:
            matplotlib.use(_be, force=True)
            # 実際に backend モジュールを読み込ませて失敗を捕まえる
            import matplotlib.pyplot as _plt_probe  # noqa: F401
            break
        except Exception:
            continue

import matplotlib.pyplot as plt


def _configure_matplotlib_japanese_font() -> None:
    """日本語ラベルの文字化けを防ぐフォント設定."""
    import platform
    from matplotlib import font_manager

    if platform.system() == 'Darwin':
        candidates = [
            'Hiragino Sans',
            'Hiragino Maru Gothic Pro',
            'AppleGothic',
            'Arial Unicode MS',
        ]
    elif platform.system() == 'Windows':
        candidates = ['Yu Gothic', 'MS Gothic', 'Meiryo']
    else:
        candidates = [
            'Noto Sans CJK JP',
            'IPAPGothic',
            'TakaoPGothic',
            'DejaVu Sans',
        ]

    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams['font.family'] = name
            plt.rcParams['axes.unicode_minus'] = False
            return


_configure_matplotlib_japanese_font()


# ---- プラント状態インデックス (12D: ψ を p,q,r の直後に挿入) ----------
PI_PHI, PI_THETA = 0, 1
PI_P, PI_Q, PI_R = 2, 3, 4
PI_PSI = 5
PI_Z, PI_W = 6, 7
PI_X, PI_Y = 8, 9
PI_UVEL, PI_VVEL = 10, 11


def pad_plant_state(x_plant: np.ndarray, nx: int = 12) -> np.ndarray:
    """5/7/11 次元のレガシー状態を 12 次元プラント状態にパディング."""
    x = np.asarray(x_plant, dtype=np.float64)
    if x.size == nx:
        return x
    if x.size == 5:
        return np.concatenate([x, np.zeros(nx - 5)])
    if x.size == 7:
        return np.array([x[0], x[1], x[2], x[3], x[4], 0.0, x[5], x[6],
                         0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if x.size == 11:
        return np.array([x[0], x[1], x[2], x[3], x[4], 0.0,
                         x[5], x[6], x[7], x[8], x[9], x[10]], dtype=np.float64)
    raise ValueError(f"plant state size {x.size}, expected 5/7/11/{nx}")


# ---- 実行ごとの PNG 出力ディレクトリ --------------------------------
def make_run_img_dir() -> Path:
    """
    本スクリプトと同階層の img/<YYYYMMDD_HHMMSS>/ を作成して返す。
    1 回の main() 実行で 1 フォルダ (全シナリオの PNG を同一フォルダに格納)。
    """
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(__file__).resolve().parent / "img" / run_id
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


# ---- Cholesky / SPD solve (numpy only) -------------------------------
def _cholesky_lower(A: np.ndarray) -> np.ndarray:
    """Lower-triangular Cholesky factor (健全性チェック用にも使う)."""
    return np.linalg.cholesky(A)


# =========================================================================
# 1. Params
# =========================================================================
@dataclass
class Params:
    # ---- physical ----
    Ix: float = 1.8e-3
    Iy: float = 1.9e-3
    Iz: float = 3.0e-3
    Cp: float = 0.025
    Cq: float = 0.025
    Cr: float = 1.0e-4
    arm_length: float = 0.050      # L [m] (参照用。B_c は下の motor_x/y から構成)
    mass: float = 0.285            # [kg]
    gravity: float = 9.81
    C_QT: float = 1.54e-8 / 5.21e-7
    T_max: float = 0.75
    tau_act: float = 0.045

    # ---- 機体幾何 (実機 Hexa-X の実測配置) ----
    # M1 前右, M2 前左, M3 右横, M4 左横, M5 後右, M6 後左
    # s_yaw: CW = -1, CCW = +1
    motor_x: Tuple[float, ...] = (0.085, 0.085, 0.000, 0.000, -0.085, -0.085)
    motor_y: Tuple[float, ...] = (0.050, -0.050, 0.100, -0.100, 0.050, -0.050)
    motor_s: Tuple[float, ...] = (-1.0, +1.0, +1.0, -1.0, -1.0, +1.0)

    # ---- NMPC ----
    N: int = 14                    # 実機 nmpc_params.hpp（NX=5 版）
    dt: float = 0.01               # NMPC 予測周期 [s] (Sx/Su/H の離散化に使用、変更不可)
    NX: int = 5                    # 実機 nmpc_params.hpp（NX=5 版）。8 にすると遅れ状態つき
    NU: int = 6
    U_min: float = 0.0
    U_max: float = 0.95
    damping_eps: float = 1.0e-3

    # ---- マルチレート ----
    dt_sim: float = 0.001          # プラント RK4 周期 [s] (1 kHz)
    nmpc_decimation: int = 10      # NMPC / 高度 PID 実効周期 = dt_sim * decimation = 10 ms

    # ---- 高度 PID (PKG-B 再調整: 切替直後の bang-bang を回避) ----
    z_ref_default: float = -2.0    # NED [m] (下向き正、-2m で 2m ホバー)
    Kp_z: float = 2.0              # 3.0 → 2.0: 切替直後の P キック 33% 低減
    Ki_z: float = 0.3              # 据置き (定常偏差用)
    Kd_z: float = 1.0              # 2.5 → 1.0: w=1m/s で Kd·w=1N (機体推力 16N の 6%)
    i_z_limit: float = 0.2         # 積分項クランプ [N] (mg の ~10 %)
    T_cmd_min_ratio: float = 0.3   # T_cmd 下限 = ratio·mg (実機 motor_min_pwm 相当)
    trim_clip_cos: float = 0.5     # cos(phi)cos(theta) 下限 (大角度の発散防止)

    # ---- Open-loop 上昇フェーズ (PKG-A: 余剰推力を圧縮、bumpless transfer) ----
    # 地上 (z=0) から「目標 - z_switch_offset」までは全機固定 duty で open-loop 上昇。
    # その後 PID を起動し、z_ref を z_switch→z_target にランプで接続する。
    # Hexa-X: PKG-E2 (mass+5%, T_max-5%) + PKG-E3 (aero drag) を勘案して上方修正
    # 必要 duty ≈ mg·1.05 / (6·T_max·0.95) ≈ 0.133 (静止) + aero マージン
    pwm_ascent: float = 0.145         # 0.125→0.145: PKG-E のプラント誤差 + aero マージン
    z_switch_offset: float = 1.0      # 0.30→1.0: 高度 1 m で早めに PID 切替 → ホバーまで余裕
    z_ramp_duration: float = 3.0      # 1.5→3.0: ランプを延長し P 項のキックを抑制
    u_ref_lpf_tau: float = 0.0        # PKG-C: 加算合成移行で LPF 不要 (0 で無効)

    # Bumpless Transfer 用パラメータ (PKG-A)
    kd_fadein_duration: float = 0.5   # bumpless restart 後、D 項を 0 → Kd_z に立ち上げる時間 [s]
    w_ref_ramp_duration: float = 1.0  # bumpless restart 後、w_ref を w_at_restart → 0 にランプ [s]

    # 故障注入時の Bumpless Handling (PKG-D)
    fault_baseline_ramp_duration: float = 0.3  # u_baseline の故障時 ramp [s] (D1, 0 で無効)
    fault_iz_attenuation: float = 0.5          # 故障時 PID 積分項 i_z の減衰係数 (D3)

    # ---- cost (Bryson's Rule) ----
    Q_phi: float = 1000.0
    Q_theta: float = 1000.0
    Q_p: float = 80.0
    Q_q: float = 80.0
    Q_r: float = 40.0
    R_u: float = 620.0
    terminal_scale: float = 10.0

    # ---- RTI ----
    reset_period: int = 100
    sr2_damping_min: float = 0.2
    secant_min_norm: float = 1.0e-6
    admm_max_iter: int = 4
    admm_rho: float = 1.0

    # ---- ノイズ (真値 x_true / 観測 x_meas 分離) ----
    # noise_enable=False なら従来どおり perfect feedback (全 σ は無視)。
    noise_enable: bool = True
    noise_seed: int = 42
    # プロセス: 連続白ノイズ相当 → 各 RK4 ステップで σ * sqrt(dt_sim) を状態に加算
    sigma_p_proc: float = 0.05       # [rad/s] 相当の p 摂動 (1 kHz)
    sigma_q_proc: float = 0.05
    sigma_r_proc: float = 0.05
    sigma_w_proc: float = 0.10       # [m/s] 相当の w 摂動
    # 観測 (IMU / 高度): 各 dt_sim で独立サンプル
    sigma_phi_meas: float = 0.005    # [rad]
    sigma_theta_meas: float = 0.005
    sigma_gyro_meas: float = 0.010   # [rad/s] on p, q, r
    sigma_z_meas: float = 0.020      # [m]
    sigma_w_meas: float = 0.050      # [m/s]
    sigma_gyro_bias_rw: float = 0.005  # [rad/s / sqrt(s)] ジャイロバイアス RW (0 で無効)。STRESS 既定: 15 s で約 1 deg/s

    # ---- PKG-E1: モータ一次遅れ + アクチュエータ遅延 ----
    tau_motor: float = 0.05            # モータ一次遅れ時定数 [s] (50ms, 0で即応答)
    actuator_delay_steps: int = 2      # ESC + 通信遅延 [dt_sim 単位] (2ms)

    # ---- PKG-E2: プラント側の "真" 物理パラメータ (NMPC からの誤差を再現) ----
    plant_Ix_ratio: float = 1.20       # +20% (NMPC は I_x を実際より小さく見積もる)
    plant_Iy_ratio: float = 0.85       # -15%
    plant_Iz_ratio: float = 1.10       # +10%
    plant_mass_ratio: float = 1.0      # 実機合わせ: 0.285 kg ちょうど（元 1.05）
    plant_T_max_ratio: float = 1.0     # 実機合わせ: 推力は下の飛行較正曲線で与える（元 0.95）

    # ---- PKG-E3: 空力推力モデル (プロペラ前進比効果) ----
    w_induced_hover: float = 12.0      # ホバー誘導速度 [m/s], 0 で機能 OFF
    aero_factor_min: float = 0.30      # 下限 (下降中の暴走防止)
    aero_factor_max: float = 1.20      # 上限

    # ---- PKG-E4: センサ遅延 (dt_sim 単位, 0 で従来通り) ----
    imu_delay_steps: int = 5           # IMU 姿勢・角速度の遅延 (5 ms)
    alt_delay_steps: int = 20          # 高度 z, w の遅延 (20 ms)

    # ---- PKG-E5/E9: プラント状態次元 (ψ + 水平移動) ----
    # [φ, θ, p, q, r, ψ, z, w, x_pos, y_pos, u_vel, v_vel]
    NX_plant: int = 12

    # ---- PKG-E6: プロペラジャイロ (簡易版, J_eq=0 で無効) ----
    J_prop_eq: float = 5.0e-5          # プロペラ等価角運動量係数 [Nms]

    # ---- PKG-E7: 地面効果 (ge_height=0 で無効) ----
    ge_height: float = 0.5             # 地面効果が効く高度 [m]
    ge_max_boost: float = 0.15         # 地表ピーク時の推力増加率 (+15%)

    # ---- PKG-E10 (2026-10-08): 並進の空力（抗力・ロータ抗力・ブローバック・風）。全部 0 で無効＝従来どおり ----
    # SIL の program/sil_nmpc/host/plant6dof.py と同じ式・同じ推定値（--aero full で一括 ON）。
    #   形状抗力  F = −C v_a|v_a|（機体座標・対気速度）       ⚠ 推定 C=(0.006, 0.006, 0.025)
    #   ロータ抗力 F_xy = −k T v_a,xy                         ⚠ 推定 k=0.03 s/m
    #   ブローバック L = −k_bf,roll v_a,  M = +k_bf,pitch u_a  ⚠ 推定 7.5e-3 / 2.8e-3 N·m/(m/s)
    C_drag_u: float = 0.0
    C_drag_v: float = 0.0
    C_drag_w: float = 0.0
    k_rotor_drag: float = 0.0
    k_bf_roll: float = 0.0
    k_bf_pitch: float = 0.0
    wind_n: float = 0.0                # 一定風 NED [m/s]
    wind_e: float = 0.0
    wind_d: float = 0.0

    # ---- PKG-E8: ESC PWM 量子化 (esc_bits=0 で無効) ----
    esc_bits: int = 12                 # ESC PWM 分解能 [bit] (12 で 4096 段階)

    # ---- STRESS: 実機との差を埋める悪条件（0 / 1.0 で無効＝元のシミュと同じ） ----
    # 実機は 100 Hz 周期の頭で状態を取り、計算を終えた指令を次の周期以降に出す。元のシミュは同じ 1 ms で即適用していた。
    compute_delay_cycles: int = 1      # 指令が出るまでの制御周期数 [10 ms 単位]（実機は約 1）
    # 実機のプラント効きは閉ループログからの推定で約 2 倍（20260910_v7_prop_change_doubled_plant_gain、未確定）。
    plant_moment_gain: float = 1.0     # プラントの姿勢モーメント / NMPC モデル（推力曲線に上乗せする感度用倍率）
    # 実機のフル再構築は 28–39 ms。その間 core1 は次の NMPC を回せず、400 Hz 段は前回の duty を出し続ける。
    rebuild_hold_cycles: int = 3       # フル再構築時に指令を保持する制御周期数
    # ジャイロバイアスを姿勢推定へ漏らす相補フィルタの時定数。定常の傾き誤差 ≈ bias × tau。
    gyro_bias_att_tau: float = 1.0     # [s] (0 でバイアスは p,q,r にだけ乗る＝元のシミュ)
    # 実機のフェイルセーフ（FS_ATTITUDE_LIMIT / FS_RATE_LIMIT）相当。超えたら発散としてその場で打ち切る。
    # ---- 実機合わせ（2026-10-06） ----
    # プラントの 1 モータ推力 [N] = scale·(a u² + b u + c)。0919_v8 等の飛行で u_hover=0.5506 @ 285 g を再現する飛行較正曲線。
    plant_thrust_curve: bool = True
    thrust_curve_abc: Tuple[float, float, float] = (-0.2265, 1.0525, 0.0452)
    thrust_curve_scale: float = 0.8389
    # 実機の遅れ補償 DC（nmpc_params.hpp の delay_comp_steps）。NX=5 のときだけ使う。
    delay_comp_steps: int = 6
    # ---- 耐故障制御の方式（2026-10-06） ----
    #   "firmware": 実機 Benzen_NMPC（NX=5 版 control.cpp）と同じ。
    #       η は box だけ（故障発の duty を 0 に固定）。B と R には掛けない。故障で再構築しない。
    #       参照 U_ref は「故障発とその対角を 0、残り n_carry 発を u_hover·NU/n_carry」の釣り合い配分
    #       （20260915_uref_must_point_at_the_balanced_allocation）。u_hover は全発共通のスロットル。
    #       出力の下限: 故障発 0、対角 0、その他 min_duty_fw。
    #   "sim": 元の nmpc_simulation.py の FDI 既知処理。B_eff = B·diag(η)、案B（R_u/η）、
    #       box は故障発だけ 0、即時フル再構築、baseline は生存 5 発に均等、PKG-D のランプ類あり。
    ftc_formulation: str = "firmware"
    min_duty_fw: float = 0.1
    # 故障後のスロットル: False = 高度 PID（積極的な操縦者）、True = 故障時点で静的釣り合いの値に固定
    #   （u_carry = Tf⁻¹(mg/n_carry)。理想的な操縦者が一発でホバー値に合わせ、以後動かさない）
    ftc_throttle_hold: bool = False
    divergence_angle_deg: float = 90.0
    divergence_rate_dps: float = 360.0

    @property
    def NXT(self) -> int:
        return self.NX * self.N

    @property
    def NUT(self) -> int:
        return self.NU * self.N

    @property
    def u_hover_nom(self) -> float:
        """健全 6 機ホバー時 1 モータ duty（推力曲線あり: 曲線の逆関数 = 実測 0.5506）."""
        if self.plant_thrust_curve:
            return float(thrust_curve_inv(self.mass * self.gravity / 6.0, self))
        return self.mass * self.gravity / (6.0 * self.T_max)


def thrust_curve(u, p: "Params"):
    """プラントの 1 モータ推力 [N]。duty ≤ 0 では 0（停止）。"""
    a, b, c = p.thrust_curve_abc
    u = np.asarray(u, dtype=np.float64)
    return np.where(u > 1.0e-6, p.thrust_curve_scale * (a * u * u + b * u + c), 0.0)


def thrust_curve_inv(T, p: "Params"):
    """thrust_curve の逆関数（0 ≤ u ≤ U_max に切る）。"""
    a, b, c = p.thrust_curve_abc
    y = np.asarray(T, dtype=np.float64) / p.thrust_curve_scale
    disc = np.maximum(b * b - 4.0 * a * (c - y), 0.0)
    u = (-b + np.sqrt(disc)) / (2.0 * a)
    return np.clip(u, p.U_min, p.U_max)


# =========================================================================
# 2. HexaModel  (連続時間ダイナミクス + Jacobian + 離散化)
# =========================================================================
class HexaModel:
    """
    Hexa-X 配置の姿勢ダイナミクス。
        NMPC 状態 x = [phi, theta, p, q, r, a_p, a_q, a_r]
        PlantModel 内部では NX=5 として使い、6モータ遅れはプラント側に残す。
        入力 u = [u_M1..u_M6] in [0,1]
        モータ配置は Params.motor_x / motor_y / motor_s で与える (実機実測値)。
            M1: 前右 (+0.085, +0.050)  CW   s = -1
            M2: 前左 (+0.085, -0.050)  CCW  s = +1
            M3: 右横 ( 0.000, +0.100)  CCW  s = +1
            M4: 左横 ( 0.000, -0.100)  CW   s = -1
            M5: 後右 (-0.085, +0.050)  CW   s = -1
            M6: 後左 (-0.085, -0.050)  CCW  s = +1
        トルク:
            tau_x_i = -y_i * T_max * u_i
            tau_y_i = +x_i * T_max * u_i
            tau_z_i = s_i * C_QT * T_max * u_i
    """
    def __init__(self, p: Params):
        self.p = p
        self.x_motor = np.asarray(p.motor_x, dtype=np.float64)
        self.y_motor = np.asarray(p.motor_y, dtype=np.float64)
        self.s_yaw = np.asarray(p.motor_s, dtype=np.float64)
        # モータ効率 (Λ = diag(eta))。FDI 既知ケースでは NMPC モデル側にも反映する。
        self.eta = np.ones(6, dtype=np.float64)
        self._build_Bc()

    def _build_Bc(self) -> None:
        p = self.p
        Batt = np.zeros((3, 6), dtype=np.float64)
        for k in range(6):
            Batt[0, k] = -self.y_motor[k] * p.T_max / p.Ix
            Batt[1, k] = +self.x_motor[k] * p.T_max / p.Iy
            Batt[2, k] =  self.s_yaw[k] * p.C_QT * p.T_max / p.Iz
        self.Batt_nom = Batt
        self._refresh_Bc_eff()

    def _refresh_Bc_eff(self) -> None:
        """B_eff = B_nom @ diag(eta).  η を更新したら必ず呼ぶ."""
        self.Batt = self.Batt_nom * self.eta[np.newaxis, :]
        if self.p.NX == 5:
            self.Bc = np.zeros((5, 6), dtype=np.float64)
            self.Bc[2:5, :] = self.Batt
        else:
            # FTC の eta は box のみに反映し、モデル入力へ二重に掛けない。
            self.Batt = self.Batt_nom.copy()
            self.Bc = np.zeros((8, 6), dtype=np.float64)
            self.Bc[5:8, :] = self.Batt / self.p.tau_act

    def set_eta(self, eta: np.ndarray) -> None:
        """NMPC 内部モデルの モータ効率 Λ を設定 (FDI 既知ケース用)."""
        self.eta[:] = np.asarray(eta, dtype=np.float64).reshape(6)
        self._refresh_Bc_eff()

    # --- smooth |v| = sqrt(v^2 + eps^2) ---
    def _abs_s(self, v: float) -> float:
        return math.sqrt(v * v + self.p.damping_eps ** 2)

    def _abs_s_d(self, v: float) -> float:
        return v / math.sqrt(v * v + self.p.damping_eps ** 2)

    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """連続時間 xdot = f(x,u)."""
        p = self.p
        phi, theta, pr, qr, rr = float(x[0]), float(x[1]), float(x[2]), float(x[3]), float(x[4])

        sphi = math.sin(phi);  cphi = math.cos(phi)
        ctheta = math.cos(theta)
        if abs(ctheta) < 1e-3:
            ctheta = math.copysign(1e-3, ctheta) if ctheta != 0 else 1e-3
        ttheta = math.sin(theta) / ctheta

        ap = self._abs_s(pr); aq = self._abs_s(qr); ar = self._abs_s(rr)

        xdot = np.empty(p.NX, dtype=np.float64)
        xdot[0] = pr + qr * sphi * ttheta + rr * cphi * ttheta
        xdot[1] = qr * cphi - rr * sphi
        rigid = np.array([
            (-(p.Iz - p.Iy) * qr * rr - p.Cp * pr * ap) / p.Ix,
            (-(p.Ix - p.Iz) * rr * pr - p.Cq * qr * aq) / p.Iy,
            (-(p.Iy - p.Ix) * pr * qr - p.Cr * rr * ar) / p.Iz,
        ])
        if p.NX == 5:
            xdot[2:5] = self.Batt @ u + rigid
        else:
            xdot[2:5] = x[5:8] + rigid
            xdot[5:8] = (self.Batt @ u - x[5:8]) / p.tau_act
        return xdot

    def jacobians(self, x: np.ndarray, u: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """連続時間 Jacobian (A_c, B_c)."""
        p = self.p
        phi, theta, pr, qr, rr = float(x[0]), float(x[1]), float(x[2]), float(x[3]), float(x[4])
        sphi = math.sin(phi); cphi = math.cos(phi)
        ctheta = math.cos(theta)
        if abs(ctheta) < 1e-3:
            ctheta = math.copysign(1e-3, ctheta) if ctheta != 0 else 1e-3
        ttheta = math.sin(theta) / ctheta
        sec2 = 1.0 / (ctheta * ctheta)

        A = np.zeros((p.NX, p.NX), dtype=np.float64)
        A[0, 0] = (qr * cphi - rr * sphi) * ttheta
        A[0, 1] = (qr * sphi + rr * cphi) * sec2
        A[0, 2] = 1.0
        A[0, 3] = sphi * ttheta
        A[0, 4] = cphi * ttheta

        A[1, 0] = -qr * sphi - rr * cphi
        A[1, 3] =  cphi
        A[1, 4] = -sphi

        ap = self._abs_s(pr); dap = self._abs_s_d(pr)
        A[2, 2] = -p.Cp * (ap + pr * dap) / p.Ix
        A[2, 3] = -(p.Iz - p.Iy) * rr / p.Ix
        A[2, 4] = -(p.Iz - p.Iy) * qr / p.Ix

        aq = self._abs_s(qr); daq = self._abs_s_d(qr)
        A[3, 2] = -(p.Ix - p.Iz) * rr / p.Iy
        A[3, 3] = -p.Cq * (aq + qr * daq) / p.Iy
        A[3, 4] = -(p.Ix - p.Iz) * pr / p.Iy

        ar = self._abs_s(rr); dar = self._abs_s_d(rr)
        A[4, 2] = -(p.Iy - p.Ix) * qr / p.Iz
        A[4, 3] = -(p.Iy - p.Ix) * pr / p.Iz
        A[4, 4] = -p.Cr * (ar + rr * dar) / p.Iz

        if p.NX == 8:
            A[2:5, 5:8] = np.eye(3)
            A[5:8, 5:8] = -np.eye(3) / p.tau_act
        return A, self.Bc

    def discretize_euler(self, A_c: np.ndarray, B_c: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        dt = self.p.dt
        I = np.eye(self.p.NX, dtype=np.float64)
        Ad, Bd = I + dt * A_c, dt * B_c
        if self.p.NX == 8:
            alpha = math.exp(-dt / self.p.tau_act)
            beta = self.p.tau_act * (1.0 - alpha)
            gamma = dt - beta
            Ad[2:5, 5:8] = beta * np.eye(3)
            Ad[5:8, 5:8] = alpha * np.eye(3)
            Bd[2:5, :] = gamma * self.Batt
            Bd[5:8, :] = (1.0 - alpha) * self.Batt
        return Ad, Bd

    def step_euler(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """NMPC 内部用の Euler 1 ステップ."""
        x_next = x + self.p.dt * self.f(x, u)
        if self.p.NX == 8:
            alpha = math.exp(-self.p.dt / self.p.tau_act)
            beta = self.p.tau_act * (1.0 - alpha)
            gamma = self.p.dt - beta
            rigid = self.f(x, u)[2:5] - x[5:8]
            a_cmd = self.Batt @ u
            x_next[2:5] = x[2:5] + self.p.dt * rigid + beta * x[5:8] + gamma * a_cmd
            x_next[5:8] = alpha * x[5:8] + (1.0 - alpha) * a_cmd
        return x_next

    def step_rk4(self, x: np.ndarray, u: np.ndarray, dt: Optional[float] = None) -> np.ndarray:
        """プラント用の RK4 ステップ."""
        h = self.p.dt if dt is None else dt
        k1 = self.f(x, u)
        k2 = self.f(x + 0.5 * h * k1, u)
        k3 = self.f(x + 0.5 * h * k2, u)
        k4 = self.f(x + h * k3, u)
        return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


# =========================================================================
# 2.5 PlantModel  (7 状態: 姿勢 + 鉛直並進)
# =========================================================================
class PlantModel:
    """
    プラント側の真モデル。状態 7 次元:
        x = [phi, theta, p, q, r, z, w]
            z : NED 下向き正 [m]   (例: z=-1.0 で高度 1m)
            w : NED 下向き正 [m/s]
    NMPC の姿勢ダイナミクスは HexaModel をそのまま流用。
    並進ダイナミクス (z, w) と、力/トルク外乱を追加で重畳する。

    機体推力 T_total = T_max * Σ u_i は機体 -z_body 方向 (上向き) に作用。
    World z (下向き正) 軸への投影は -T_total * cos(phi) * cos(theta) / m。
    重力は +g (下向き)。
    """
    def __init__(self, p: Params):
        self.p = p   # NMPC が信じる公称 Params (制御パラメータの参照元)

        # ---- PKG-E2: プラント側は誤差を含む物理パラメータで HexaModel を構築 ----
        p_plant = copy.copy(p)
        p_plant.Ix = p.Ix * p.plant_Ix_ratio
        p_plant.Iy = p.Iy * p.plant_Iy_ratio
        p_plant.Iz = p.Iz * p.plant_Iz_ratio
        p_plant.mass = p.mass * p.plant_mass_ratio
        p_plant.T_max = p.T_max * p.plant_T_max_ratio
        p_plant.NX = 5  # 真のプラントは別経路の6モータ一次遅れを維持
        self.p_plant = p_plant
        self.attitude = HexaModel(p_plant)
        # STRESS: 姿勢モーメントの効きだけを gain 倍する（推力 T_total は p_plant.T_max のまま＝高度は変えない）
        if abs(p.plant_moment_gain - 1.0) > 1e-12:
            self.attitude.Batt_nom = self.attitude.Batt_nom * p.plant_moment_gain
            self.attitude._refresh_Bc_eff()

        # ---- PKG-E1: モータ一次遅れ + アクチュエータ遅延 ----
        self.u_actual = np.full(p.NU, p.u_hover_nom, dtype=np.float64)
        self._u_delay_buffer: deque = deque(maxlen=max(p.actuator_delay_steps, 1))
        for _ in range(max(p.actuator_delay_steps, 1)):
            self._u_delay_buffer.append(np.full(p.NU, p.u_hover_nom, dtype=np.float64))

    def reset_motors(self, u_init: Optional[np.ndarray] = None) -> None:
        """シナリオ開始時にモータ状態を初期化する (PKG-E1)."""
        if u_init is None:
            u_init = np.full(self.p.NU, self.p.u_hover_nom, dtype=np.float64)
        u_init = np.asarray(u_init, dtype=np.float64)
        self.u_actual[:] = u_init
        self._u_delay_buffer.clear()
        for _ in range(max(self.p.actuator_delay_steps, 1)):
            self._u_delay_buffer.append(u_init.copy())

    def apply_actuator_dynamics(self,
                                u_cmd: np.ndarray,
                                dt: float) -> np.ndarray:
        """PKG-E1: u_cmd → アクチュエータ遅延 → モータ一次遅れ → u_actual."""
        p = self.p
        if p.actuator_delay_steps > 0:
            self._u_delay_buffer.append(np.asarray(u_cmd, dtype=np.float64).copy())
            u_target = self._u_delay_buffer[0]
        else:
            u_target = np.asarray(u_cmd, dtype=np.float64)
        if p.tau_motor > 1e-9:
            alpha = dt / (p.tau_motor + dt)
            self.u_actual = self.u_actual + alpha * (u_target - self.u_actual)
        else:
            self.u_actual = u_target.copy()
        return self.u_actual.copy()

    def f_full(self,
               x_plant: np.ndarray,
               u: np.ndarray,
               F_world: np.ndarray,
               tau_body: np.ndarray) -> np.ndarray:
        """連続時間 xdot = f(x_plant, u, F_world, tau_body).

        プラント状態 (PKG-E9):
            x_plant = [φ, θ, p, q, r, ψ, z, w, x_pos, y_pos, u_vel, v_vel]  (12 次元)
            5/7/11 次元は pad_plant_state で自動パディング (後方互換)。
        """
        p = self.p_plant   # PKG-E2: 真値のプラントパラメータを使う
        x_plant = pad_plant_state(x_plant, self.p.NX_plant)
        u_duty = np.asarray(u, dtype=np.float64)
        # 実機合わせ: 推力曲線 Tf(u) を「等価な線形 duty」Tf(u)/T_max に直してから下の線形式へ渡す。
        #   モーメント（Batt·u_eq）と総推力（T_max·Σu_eq）の両方が Σ Tf(u) で一致する。
        if self.p.plant_thrust_curve:
            u = thrust_curve(u_duty, self.p) / p.T_max

        # ---- 姿勢 ----
        x5 = x_plant[:5]
        xdot5 = self.attitude.f(x5, u)
        xdot5[2] += tau_body[0] / p.Ix
        xdot5[3] += tau_body[1] / p.Iy
        xdot5[4] += tau_body[2] / p.Iz

        # ---- PKG-E6: プロペラジャイロ (簡易版) ----
        if self.p.J_prop_eq > 1e-12:
            s_yaw = self.attitude.s_yaw
            H_prop = self.p.J_prop_eq * float(np.sum(s_yaw * u_duty))
            pr = float(x_plant[PI_P]); qr = float(x_plant[PI_Q])
            xdot5[2] += (-H_prop * qr) / p.Ix
            xdot5[3] += (+H_prop * pr) / p.Iy

        # ---- 推力計算 (E3 空力 + E7 地面効果) ----
        phi   = float(x_plant[PI_PHI])
        theta = float(x_plant[PI_THETA])
        z     = float(x_plant[PI_Z])
        w     = float(x_plant[PI_W])
        u_vel = float(x_plant[PI_UVEL])
        v_vel = float(x_plant[PI_VVEL])

        T_total_raw = p.T_max * float(np.sum(u))

        # PKG-E3: プロペラ前進比 (上昇中 V_z>0 で推力低下)
        if self.p.w_induced_hover > 1e-9:
            V_z_into_propeller = -w   # 上向き正
            aero_factor = float(np.clip(
                1.0 - V_z_into_propeller / self.p.w_induced_hover,
                self.p.aero_factor_min, self.p.aero_factor_max
            ))
        else:
            aero_factor = 1.0

        # PKG-E7: 地面効果
        altitude = -z
        if (self.p.ge_height > 1e-9) and (altitude < self.p.ge_height):
            h_norm = max(altitude, 0.0) / self.p.ge_height
            ge_factor = 1.0 + self.p.ge_max_boost * (1.0 - h_norm)
        else:
            ge_factor = 1.0

        T_total = T_total_raw * aero_factor * ge_factor

        cphi = math.cos(phi); sphi = math.sin(phi)
        cth  = math.cos(theta); sth = math.sin(theta)
        if abs(cth) < 1e-3:
            cth = math.copysign(1e-3, cth) if cth != 0 else 1e-3
        cphi_cth = max(cphi * cth, self.p.trim_clip_cos)

        psi = float(x_plant[PI_PSI])
        cpsi = math.cos(psi); spsi = math.sin(psi)

        # ---- 並進 (NED, ψ 回転込み) ----
        ax0 = -(T_total / p.mass) * sth
        ay0 =  (T_total / p.mass) * sphi * cth
        a_x =  cpsi * ax0 + spsi * ay0 + F_world[0] / p.mass
        a_y = -spsi * ax0 + cpsi * ay0 + F_world[1] / p.mass
        a_z =  p.gravity - (T_total / p.mass) * cphi_cth + F_world[2] / p.mass

        # ---- PKG-E10: 空力（抗力・ロータ抗力・ブローバック）。係数がすべて 0 なら何もしない ----
        P0 = self.p
        if (P0.C_drag_u or P0.C_drag_v or P0.C_drag_w or P0.k_rotor_drag or P0.k_bf_roll or P0.k_bf_pitch):
            R = np.array([   # 機体 → NED（ZYX オイラー）
                [cth * cpsi, sphi * sth * cpsi - cphi * spsi, cphi * sth * cpsi + sphi * spsi],
                [cth * spsi, sphi * sth * spsi + cphi * cpsi, cphi * sth * spsi - sphi * cpsi],
                [-sth, sphi * cth, cphi * cth]])
            v_ned = np.array([u_vel, v_vel, w])
            va = R.T @ (v_ned - np.array([P0.wind_n, P0.wind_e, P0.wind_d]))
            Cd = np.array([P0.C_drag_u, P0.C_drag_v, P0.C_drag_w])
            Fb = -Cd * va * np.abs(va)
            Fb[0] -= P0.k_rotor_drag * T_total * va[0]
            Fb[1] -= P0.k_rotor_drag * T_total * va[1]
            Fn = R @ Fb
            a_x += Fn[0] / p.mass; a_y += Fn[1] / p.mass; a_z += Fn[2] / p.mass
            xdot5[2] += -P0.k_bf_roll * va[1] / p.Ix
            xdot5[3] += P0.k_bf_pitch * va[0] / p.Iy

        xdot = np.zeros(self.p.NX_plant, dtype=np.float64)
        xdot[:5] = xdot5
        xdot[PI_PSI] = (float(x_plant[PI_Q]) * sphi / cth
                        + float(x_plant[PI_R]) * cphi / cth)
        xdot[PI_Z]    = w
        xdot[PI_W]    = a_z
        xdot[PI_X]    = u_vel
        xdot[PI_Y]    = v_vel
        xdot[PI_UVEL] = a_x
        xdot[PI_VVEL] = a_y
        return xdot

    def step_rk4(self,
                 x_plant: np.ndarray,
                 u: np.ndarray,
                 F_world: np.ndarray,
                 tau_body: np.ndarray,
                 dt: float) -> np.ndarray:
        """1 ステップ RK4 (12 次元プラント, レガシー次元は自動パディング)."""
        x_plant = pad_plant_state(x_plant, self.p.NX_plant)
        k1 = self.f_full(x_plant,                u, F_world, tau_body)
        k2 = self.f_full(x_plant + 0.5 * dt * k1, u, F_world, tau_body)
        k3 = self.f_full(x_plant + 0.5 * dt * k2, u, F_world, tau_body)
        k4 = self.f_full(x_plant + dt * k3,       u, F_world, tau_body)
        return x_plant + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


# =========================================================================
# 2.6 AltitudePID  (フル PID + 重力補償 + アンチワインドアップ)
# =========================================================================
class AltitudePID:
    """
    出力: T_cmd [N] (機体推力の総和コマンド)
        T_cmd = T_trim + Kp*e_z + Ki*∫e_z dt + Kd*(w_ref - w)
        e_z   = z_ref - z
        T_trim = m * g / (cos(phi)*cos(theta))   ← 姿勢による重力補償

    NED 座標: z 下向き正、w 下向き正なので、
        z_ref = -1.0 (m) のとき "高度 1m" になる。
    """
    def __init__(self, p: Params):
        self.p = p
        self.i_z: float = 0.0

    def reset(self) -> None:
        self.i_z = 0.0

    def update(self,
               z: float, w: float,
               z_ref: float, w_ref: float,
               phi: float, theta: float,
               dt: float,
               sat_limit_hit: bool = False,
               kd_scale: float = 1.0) -> Tuple[float, dict]:
        """
        NED (z 下向き正) で:
            - z > z_ref → 機体が低い (落下方向ずれ) → 推力 ↑ で z を減らす (上昇)
            - w > 0     → 落下中                   → 推力 ↑ で減速
        誤差を e = (z - z_ref) として、Kp, Ki, Kd > 0 で素直に動く。

        Bumpless Transfer (PKG-A):
            kd_scale ∈ [0, 1]: D 項のフェードイン用ゲイン。
                切替直後 0 → kd_fadein_duration で 1。
                呼び出し側で時間ベースに kd_scale を計算して渡す。
            w_ref も呼び出し側で w_at_switch → 0 にランプして渡す想定。
        """
        p = self.p
        e_z = z - z_ref          # 「落下方向ずれ」が正

        # アンチワインドアップ: アクチュエータがサチュっているときは積分停止
        if not sat_limit_hit:
            self.i_z = float(np.clip(self.i_z + e_z * dt,
                                     -p.i_z_limit, +p.i_z_limit))

        # 姿勢による重力補償 (T_trim)
        cphi_cth = math.cos(phi) * math.cos(theta)
        cphi_cth = max(cphi_cth, p.trim_clip_cos)
        T_trim = p.mass * p.gravity / cphi_cth

        # フィードバック項 (D 項に kd_scale を掛けて bumpless transfer)
        T_fb = (p.Kp_z * e_z
                + p.Ki_z * self.i_z
                + kd_scale * p.Kd_z * (w - w_ref))

        T_cmd = T_trim + T_fb

        # 下限クリップ: 実機 motor_min_pwm 相当 (急減速時に thrust 0 = 自由落下を回避)
        T_cmd_min = p.T_cmd_min_ratio * p.mass * p.gravity
        T_cmd = max(T_cmd, T_cmd_min)

        info = dict(e_z=e_z, i_z=self.i_z, T_trim=T_trim, T_fb=T_fb,
                    kd_scale=kd_scale, w_ref=w_ref)
        return T_cmd, info


def t_cmd_to_u_hover(T_cmd: float,
                     eta: np.ndarray,
                     params: Params) -> np.ndarray:
    """
    η 重みつき配分 (案A): 生存機に同じ duty u を与え、Σ(η·u)·T_max = T_cmd を満たす。
    故障機 (η≈0) は duty=0。
    """
    eta = np.asarray(eta, dtype=np.float64).reshape(params.NU)
    sum_eta = float(np.sum(eta))
    if sum_eta < 1e-3:
        return np.zeros(params.NU, dtype=np.float64)
    if params.plant_thrust_curve:
        # 実機は操縦者がスロットルでホバー duty を探す。ここでは同じ推力曲線の逆関数で配分する。
        per = float(thrust_curve_inv(T_cmd / sum_eta, params))
    else:
        per = T_cmd / (sum_eta * params.T_max)
    per = float(np.clip(per, params.U_min, params.U_max))
    return np.where(eta > 1.0e-3, per, 0.0).astype(np.float64)


# =========================================================================
# 2.7 DisturbanceManager  (力/トルク 外乱のスケジュール)
# =========================================================================
class DisturbanceManager:
    """
    schedule: list of dict
        {
            't_start': float, 't_end': float,
            'kind':   'force_world' | 'torque_body',
            'axis':   0 | 1 | 2,
            'magnitude': float,    # [N] or [N*m]
        }

    get_wrench(t) は t に応じた (F_world ∈ R^3, tau_body ∈ R^3) を返す。
    本シミュは姿勢+鉛直のみ扱うため、
        F_world[0..1] (水平外力) は plant に反映されない (情報としてログのみ)。
        F_world[2]   (鉛直外力)  は wdot に効く。
        tau_body[0..2] は p_dot, q_dot, r_dot に効く。
    """
    def __init__(self, schedule: Optional[List[dict]] = None):
        self.schedule: List[dict] = list(schedule) if schedule else []

    def get_wrench(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        F_world  = np.zeros(3, dtype=np.float64)
        tau_body = np.zeros(3, dtype=np.float64)
        for d in self.schedule:
            if d['t_start'] <= t < d['t_end']:
                axis = int(d['axis'])
                mag  = float(d['magnitude'])
                if d['kind'] == 'force_world':
                    F_world[axis]  += mag
                elif d['kind'] == 'torque_body':
                    tau_body[axis] += mag
        return F_world, tau_body


# =========================================================================
# 2.8 NoiseModel  (プロセスノイズ + IMU 相当の観測ノイズ)
# =========================================================================
class NoiseModel:
    """
    プラント真値 x_true と制御器が参照する観測 x_meas を分離する。

    - プロセスノイズ: RK4 積分後に p,q,r,w へ σ_proc·sqrt(dt) の摂動
    - 観測ノイズ: φ,θ と p,q,r,z,w へホワイトノイズ (+ オプションのジャイロバイアス RW)
    """
    def __init__(self, p: Params, rng: Optional[np.random.Generator] = None):
        self.p = p
        self.rng = rng if rng is not None else np.random.default_rng(p.noise_seed)
        self.bias_pqr = np.zeros(3, dtype=np.float64)
        self.att_err = np.zeros(2, dtype=np.float64)   # STRESS: バイアス由来の φ,θ 推定誤差

        # ---- PKG-E4: センサ遅延バッファ (FIFO, dt_sim 単位) ----
        # IMU (idx 0..4): φ, θ, p, q, r
        # 高度 (idx 5, 6): z, w
        self._imu_delay: deque = deque(maxlen=max(p.imu_delay_steps, 1))
        self._alt_delay: deque = deque(maxlen=max(p.alt_delay_steps, 1))
        for _ in range(max(p.imu_delay_steps, 1)):
            self._imu_delay.append(np.zeros(5, dtype=np.float64))
        for _ in range(max(p.alt_delay_steps, 1)):
            self._alt_delay.append(np.zeros(2, dtype=np.float64))

    def reset(self) -> None:
        self.bias_pqr[:] = 0.0
        self.att_err[:] = 0.0
        # PKG-E4: 遅延バッファも初期化
        self._imu_delay.clear()
        self._alt_delay.clear()
        for _ in range(max(self.p.imu_delay_steps, 1)):
            self._imu_delay.append(np.zeros(5, dtype=np.float64))
        for _ in range(max(self.p.alt_delay_steps, 1)):
            self._alt_delay.append(np.zeros(2, dtype=np.float64))

    def active(self) -> bool:
        return bool(self.p.noise_enable)

    def apply_process(self, x_true: np.ndarray, dt: float) -> np.ndarray:
        """RK4 後の真状態にプロセスノイズを加える。"""
        if not self.active():
            return x_true
        p = self.p
        x = np.asarray(x_true, dtype=np.float64).copy()
        sdt = math.sqrt(dt)
        if p.sigma_p_proc > 0.0:
            x[2] += self.rng.normal(0.0, p.sigma_p_proc * sdt)
        if p.sigma_q_proc > 0.0:
            x[3] += self.rng.normal(0.0, p.sigma_q_proc * sdt)
        if p.sigma_r_proc > 0.0:
            x[4] += self.rng.normal(0.0, p.sigma_r_proc * sdt)
        if x.size >= PI_W + 1 and p.sigma_w_proc > 0.0:
            x[PI_W] += self.rng.normal(0.0, p.sigma_w_proc * sdt)
        return x

    def observe(self, x_true: np.ndarray) -> np.ndarray:
        """真状態から観測値 (制御器入力) を生成。

        PKG-E4: noise_enable=True の場合、遅延バッファ経由で IMU 5ms / 高度 20ms 遅延を適用。
        noise_enable=False は完全に「真値そのまま」(後方互換)。
        """
        x_true = np.asarray(x_true, dtype=np.float64)
        if not self.active():
            return x_true.copy()
        p = self.p

        # ---- PKG-E4: センサ遅延 ----
        # IMU (idx 0..4)
        if p.imu_delay_steps > 0:
            self._imu_delay.append(x_true[:5].copy())
            imu_delayed = self._imu_delay[0].copy()
        else:
            imu_delayed = x_true[:5].copy()
        x_true = pad_plant_state(x_true, p.NX_plant)
        if p.alt_delay_steps > 0:
            self._alt_delay.append(x_true[PI_Z:PI_W + 1].copy())
            alt_delayed = self._alt_delay[0].copy()
        else:
            alt_delayed = x_true[PI_Z:PI_W + 1].copy()

        # 観測ベクトル: IMU 遅延 + ψ パススルー + 高度遅延 + 水平状態
        x = x_true.copy()
        x[:5] = imu_delayed
        x[PI_Z:PI_W + 1] = alt_delayed

        # ---- 観測ノイズ追加 (既存処理) ----
        if p.sigma_gyro_bias_rw > 0.0:
            self.bias_pqr += self.rng.normal(
                0.0, p.sigma_gyro_bias_rw * math.sqrt(p.dt_sim), 3)
        # STRESS: 推定器はバイアス込みのジャイロを積分し、加速度計で時定数 tau で引き戻す
        #   d(att_err)/dt = bias - att_err / tau  （ホバー近傍で φ'≈p, θ'≈q）
        if p.gyro_bias_att_tau > 1e-9:
            self.att_err += p.dt_sim * (self.bias_pqr[:2] - self.att_err / p.gyro_bias_att_tau)
            x[0] += self.att_err[0]
            x[1] += self.att_err[1]
        if p.sigma_phi_meas > 0.0:
            x[0] += self.rng.normal(0.0, p.sigma_phi_meas)
        if p.sigma_theta_meas > 0.0:
            x[1] += self.rng.normal(0.0, p.sigma_theta_meas)
        if p.sigma_gyro_meas > 0.0:
            x[2:5] += self.bias_pqr + self.rng.normal(0.0, p.sigma_gyro_meas, 3)
        elif p.sigma_gyro_bias_rw > 0.0:
            x[2:5] += self.bias_pqr
        if p.sigma_z_meas > 0.0:
            x[PI_Z] += self.rng.normal(0.0, p.sigma_z_meas)
        if p.sigma_w_meas > 0.0:
            x[PI_W] += self.rng.normal(0.0, p.sigma_w_meas)
        return x


# =========================================================================
# 3. AdmmNmpcHex  (凝縮形 NMPC + SR2 + ADMM)
# =========================================================================
class AdmmNmpcHex:
    """sample/nmpc.cpp の Python 鏡像."""

    def __init__(self, model: HexaModel, p: Params):
        self.model = model
        self.p = p
        N, NX, NU = p.N, p.NX, p.NU
        self.NXT = N * NX
        self.NUT = N * NU

        # 状態凝縮行列
        self.Sx = np.zeros((self.NXT, NX), dtype=np.float64)
        self.Su = np.zeros((self.NXT, self.NUT), dtype=np.float64)
        # ヘシアン H と Cholesky L
        self.H = np.zeros((self.NUT, self.NUT), dtype=np.float64)
        self.L_chol: Optional[np.ndarray] = None

        # コスト重みベクトル
        Q = np.array(
            [p.Q_phi, p.Q_theta, p.Q_p, p.Q_q, p.Q_r, 0.0, 0.0, 0.0],
            dtype=np.float64,
        )[:NX]
        Qf = Q * p.terminal_scale
        self.qbar = np.empty(self.NXT, dtype=np.float64)
        for k in range(N):
            Qk = Qf if k == N - 1 else Q
            self.qbar[k * NX:(k + 1) * NX] = Qk
        self.rbar = np.full(self.NUT, p.R_u, dtype=np.float64)

        # warm-start (ADMM)
        self.U = np.zeros(self.NUT, dtype=np.float64)
        self.z = np.zeros(self.NUT, dtype=np.float64)
        self.lam = np.zeros(self.NUT, dtype=np.float64)
        self.ws_valid = False

        # 前周期スナップショット (TR1 用)
        self.x0_prev = np.zeros(NX, dtype=np.float64)
        self.U_lin_prev = np.zeros(self.NUT, dtype=np.float64)
        self.X_lin_prev = np.zeros(self.NXT, dtype=np.float64)

        # 状態フラグ
        self.cycles_since_reset = 0
        self.need_full_rebuild = True
        self.last_admm_iters = 0
        self.last_was_reset = False
        self.last_cost = 0.0

        # モータごとの box 制約 (初期: 全機 0..1)
        self.u_min_per_motor = np.full(NU, p.U_min, dtype=np.float64)
        self.u_max_per_motor = np.full(NU, p.U_max, dtype=np.float64)

        # PKG-C: 加算合成モード用の baseline (None なら旧 absolute モード)
        self.u_baseline: Optional[np.ndarray] = None

        # 案B: モータ別 R (η が小さいほど Δu を抑制)
        self.eta_r_floor: float = 0.1

    # --- 外部 API ------------------------------------------------------
    def set_box(self, u_min: np.ndarray, u_max: np.ndarray) -> None:
        self.u_min_per_motor[:] = u_min
        self.u_max_per_motor[:] = u_max

    def force_reset(self) -> None:
        self.need_full_rebuild = True

    def set_rbar_from_eta(self, eta: np.ndarray) -> None:
        """案B: rbar[k] = R_u / max(η_k, eta_r_floor) をホライズン全体にタイル。"""
        p = self.p
        eta_s = np.maximum(
            np.asarray(eta, dtype=np.float64).reshape(p.NU), self.eta_r_floor)
        r_motor = p.R_u / eta_s
        for k in range(p.N):
            self.rbar[k * p.NU:(k + 1) * p.NU] = r_motor

    def reset_rbar_uniform(self) -> None:
        """FDI 未知時: 均一 R_u に戻す。"""
        self.rbar[:] = self.p.R_u

    def _box_lo_hi(self) -> Tuple[np.ndarray, np.ndarray]:
        lo = np.tile(self.u_min_per_motor, self.p.N)
        hi = np.tile(self.u_max_per_motor, self.p.N)
        return lo, hi

    # --- 非線形ロールアウト -------------------------------------------
    def _rollout_nonlinear(self, x0: np.ndarray, U: np.ndarray) -> np.ndarray:
        """
        PKG-C: 加算合成モードでは U は「差分入力 Δu」。
        実際にプラントに渡す入力は u_actual = U + u_baseline。
        u_baseline が None なら旧 absolute モード (u_actual = U)。
        """
        p = self.p
        X = np.empty(self.NXT, dtype=np.float64)
        x = x0.copy()
        for k in range(p.N):
            u_k = U[k * p.NU:(k + 1) * p.NU]
            if self.u_baseline is not None:
                u_k = u_k + self.u_baseline
            x = self.model.step_euler(x, u_k)
            X[k * p.NX:(k + 1) * p.NX] = x
        return X

    # --- Sx, Su フル再構築 --------------------------------------------
    def _full_rebuild_sx_su(self, x0: np.ndarray, U_lin: np.ndarray) -> None:
        """
        PKG-C: 加算合成モードでは U_lin は「差分線形化点 Δu_lin」。
        Jacobian と状態遷移は絶対 u_actual = Δu_lin + u_baseline で評価する。
        Sx, Su は依然「Δu に対する偏差」を表現する (B_d は同じ)。
        """
        p = self.p
        N, NX, NU = p.N, p.NX, p.NU
        self.Sx[:] = 0.0
        self.Su[:] = 0.0
        x_traj = x0.copy()
        for k in range(N):
            u_k = U_lin[k * NU:(k + 1) * NU]
            if self.u_baseline is not None:
                u_actual = u_k + self.u_baseline
            else:
                u_actual = u_k
            A_c, B_c = self.model.jacobians(x_traj, u_actual)
            A_d, B_d = self.model.discretize_euler(A_c, B_c)
            row = k * NX
            if k == 0:
                self.Sx[row:row + NX, :] = A_d
                self.Su[row:row + NX, 0:NU] = B_d
            else:
                row_prev = (k - 1) * NX
                self.Sx[row:row + NX, :] = A_d @ self.Sx[row_prev:row_prev + NX, :]
                for j in range(k):
                    col = j * NU
                    self.Su[row:row + NX, col:col + NU] = \
                        A_d @ self.Su[row_prev:row_prev + NX, col:col + NU]
                self.Su[row:row + NX, k * NU:(k + 1) * NU] = B_d
            x_traj = self.model.step_euler(x_traj, u_actual)

    # --- H = Su^T Q_bar Su + R_bar + rho I の Cholesky --------------
    def _full_rebuild_chol(self) -> None:
        Su_w = self.Su * np.sqrt(self.qbar)[:, None]   # (NXT, NUT)
        self.H = Su_w.T @ Su_w
        idx = np.arange(self.NUT)
        self.H[idx, idx] += self.rbar + self.p.admm_rho
        self.H = 0.5 * (self.H + self.H.T)
        # PKG-C: 加算合成では H の条件数が悪化することがあるため
        # 段階的にリッジを増やしてフォールバック
        ok = False
        try:
            self.L_chol = _cholesky_lower(self.H)
            ok = True
        except np.linalg.LinAlgError:
            for jit in (1.0e-3, 1.0e-2, 1.0e-1, 1.0, 10.0):
                self.H[idx, idx] += jit
                try:
                    self.L_chol = _cholesky_lower(self.H)
                    ok = True
                    break
                except np.linalg.LinAlgError:
                    continue
        if not ok:
            raise np.linalg.LinAlgError(
                "AdmmNmpcHex: Cholesky failed even with ridge fallback")

    # --- M x = rhs (M = H + rho I は既に H に組込済) -------------------
    def _solve_M(self, rhs: np.ndarray) -> np.ndarray:
        return np.linalg.solve(self.H, rhs)

    # --- TR1 (Broyden rank-1) で Sx, Su を更新 ------------------------
    def _tr1_update(self, x0_now: np.ndarray, U_lin_now: np.ndarray, X_nl_now: np.ndarray):
        s_x = x0_now - self.x0_prev
        s_u = U_lin_now - self.U_lin_prev
        norm_sq = float(s_x @ s_x + s_u @ s_u)
        if norm_sq < self.p.secant_min_norm ** 2:
            return None
        residual = X_nl_now - self.X_lin_prev - self.Sx @ s_x - self.Su @ s_u
        u_rk1 = residual / norm_sq
        self.Sx += np.outer(u_rk1, s_x)
        self.Su += np.outer(u_rk1, s_u)
        return u_rk1, s_u

    # --- SR2 (rank-2) で H を更新 + 再 Cholesky -----------------------
    def _sr2_update(self, tr1_result) -> None:
        if tr1_result is None:
            return
        u_rk1, v_u = tr1_result
        Q_u = self.qbar * u_rk1
        a = self.Su.T @ Q_u
        c = float(u_rk1 @ Q_u)
        a = a - v_u * c

        if (not math.isfinite(c)) or abs(c) > 1.0e6:
            self.need_full_rebuild = True
            return

        p_vec = a + 0.5 * (c + 1.0) * v_u
        q_vec = a + 0.5 * (c - 1.0) * v_u

        # C++ では LLT に rank±1 を当てるが、Python では H に直接当てて再 Cholesky
        self.H += np.outer(p_vec, p_vec) - np.outer(q_vec, q_vec)
        self.H = 0.5 * (self.H + self.H.T)
        try:
            self.L_chol = _cholesky_lower(self.H)
        except np.linalg.LinAlgError:
            self.need_full_rebuild = True

    # --- 勾配 g = Su^T Q_bar (Sx x0 - X_ref) - R_bar U_ref -----------
    def _compute_gradient(self, x0: np.ndarray, X_ref: np.ndarray, U_ref: np.ndarray) -> np.ndarray:
        dx = self.Sx @ x0 - X_ref
        Qdx = self.qbar * dx
        g = self.Su.T @ Qdx - self.rbar * U_ref
        return g

    # --- ADMM (box 制約) ----------------------------------------------
    def _admm(self, g: np.ndarray) -> int:
        rho = self.p.admm_rho
        lo, hi = self._box_lo_hi()
        if not self.ws_valid:
            self.U[:] = 0.0
            self.z[:] = 0.0
            self.lam[:] = 0.0
            self.ws_valid = True
        iters = 0
        for _ in range(self.p.admm_max_iter):
            rhs = -g + rho * (self.z - self.lam)
            self.U = self._solve_M(rhs)
            self.z = np.clip(self.U + self.lam, lo, hi)
            self.lam = self.lam + self.U - self.z
            iters += 1
        return iters

    # --- warm-start を 1 周期シフト ------------------------------------
    def _warm_start_shift(self) -> None:
        if not self.ws_valid:
            return
        NU = self.p.NU
        N = self.p.N
        for arr in (self.U, self.z, self.lam):
            arr[:(N - 1) * NU] = arr[NU:N * NU]
            # 末尾はそのまま (1 周期前の最後をホールド)

    # --- メインステップ ------------------------------------------------
    def step(self,
             x0: np.ndarray,
             X_ref: np.ndarray,
             U_ref: np.ndarray,
             u_baseline: Optional[np.ndarray] = None) -> np.ndarray:
        """
        PKG-C: u_baseline (6,) を渡すと加算合成モード。
            - 最適化変数 U は「差分 Δu」を意味する
            - 内部ロールアウトと Jacobian は u_actual = Δu + u_baseline で評価
            - box 制約は呼び出し側で set_box([-u_baseline, U_max - u_baseline]) しておく
            - 返り値は Δu (差分)。呼び出し側で u_cmd = u_baseline + Δu に合成する
        u_baseline が None なら旧 absolute モード (後方互換)。
        """
        p = self.p
        x0 = np.asarray(x0, dtype=np.float64)
        # 加算合成モードの baseline を内部状態に保存
        # (1 周期内で _rollout_nonlinear / _full_rebuild_sx_su が参照するため)
        if u_baseline is not None:
            self.u_baseline = np.asarray(u_baseline, dtype=np.float64).reshape(6)
        else:
            self.u_baseline = None

        # 1) warm-start shift
        self._warm_start_shift()
        U_lin = self.U.copy()

        # 2) 再構築 or 差分更新
        reset_now = self.need_full_rebuild or (self.cycles_since_reset >= p.reset_period)
        if reset_now:
            self._full_rebuild_sx_su(x0, U_lin)
            self._full_rebuild_chol()
            self.cycles_since_reset = 0
            self.need_full_rebuild = False
            self.last_was_reset = True
        else:
            X_nl = self._rollout_nonlinear(x0, U_lin)
            tr1 = self._tr1_update(x0, U_lin, X_nl)
            self._sr2_update(tr1)
            self.cycles_since_reset += 1
            self.last_was_reset = False
            if self.need_full_rebuild:
                self._full_rebuild_sx_su(x0, U_lin)
                self._full_rebuild_chol()
                self.cycles_since_reset = 0
                self.need_full_rebuild = False
                self.last_was_reset = True

        # 3) 勾配
        g = self._compute_gradient(x0, X_ref, U_ref)

        # 4) ADMM
        self.last_admm_iters = self._admm(g)

        # 5) 結果取り出し (先頭 NU 個, 必ず box に収める)
        u_opt = np.clip(self.z[:p.NU], self.u_min_per_motor, self.u_max_per_motor).copy()

        # 6) 次周期用スナップショット
        self.x0_prev = x0.copy()
        self.U_lin_prev = U_lin.copy()
        self.X_lin_prev = self.Sx @ x0 + self.Su @ U_lin

        return u_opt


# =========================================================================
# 4. FaultManager
# =========================================================================
class FaultManager:
    """
    schedule: list of (t_fail, motor_idx, eta_target)
        t_fail >= 0 [s], motor_idx ∈ [0,5], eta_target ∈ [0,1]
    hard_step=True  : 即時 step
    hard_step=False : 1次遅れ (τ = tau_fault)
    """
    def __init__(self,
                 schedule: List[Tuple[float, int, float]],
                 tau_fault: float = 0.05,
                 hard_step: bool = True):
        self.schedule = schedule
        self.tau = tau_fault
        self.hard_step = hard_step
        self.eta = np.ones(6, dtype=np.float64)
        self._fired = [False] * len(schedule)

    def update(self, t: float, dt: float) -> Tuple[np.ndarray, List[Tuple[int, float]]]:
        """eta を更新し、その瞬間に発火した (motor_idx, eta_target) 一覧を返す."""
        events = []
        for i, (tf, k, eta_t) in enumerate(self.schedule):
            if t >= tf:
                if not self._fired[i]:
                    self._fired[i] = True
                    events.append((k, eta_t))
                if self.hard_step:
                    self.eta[k] = eta_t
                else:
                    self.eta[k] += (eta_t - self.eta[k]) * dt / self.tau
        return self.eta.copy(), events

    def apply_faults(self,
                     faults: List[Tuple[int, float]]) -> List[Tuple[int, float]]:
        """
        即時にモータ故障を適用 (hard_step 相当)。
        戻り値は FDI ハンドラ用の (motor_idx, eta_target) イベント一覧。
        """
        events: List[Tuple[int, float]] = []
        for k, eta_t in faults:
            k = int(k)
            eta_t = float(eta_t)
            if self.eta[k] != eta_t:
                events.append((k, eta_t))
            self.eta[k] = eta_t
        return events


# =========================================================================
# 5. Logger
# =========================================================================
class Logger:
    """
    多レート実装後はメインループ周期 dt_sim (1 kHz) で毎ステップ呼ばれる（PID/NMPC は 100 Hz）。
    `x` にはプラント真値 7 次元 (phi, theta, p, q, r, z, w) を渡す。
    `x_meas` は制御器が参照した観測値 (ノイズ無効時は x と同一)。
    高度・PID・外乱関連フィールドは省略可能 (None 渡しで全 0 を埋める)。
    """
    def __init__(self):
        self.t: List[float] = []
        self.x: List[np.ndarray] = []                # (T, 7) 真値
        self.x_meas: List[np.ndarray] = []          # (T, 7) 観測
        self.u_cmd: List[np.ndarray] = []            # (T, 6)
        self.u_app: List[np.ndarray] = []            # (T, 6)
        self.eta: List[np.ndarray] = []              # (T, 6)
        self.iter_us: List[float] = []
        self.was_reset: List[bool] = []
        self.x_ref: List[np.ndarray] = []            # (T, 7)
        # 高度・PID・外乱 (新規)
        self.z_ref: List[float] = []
        self.T_cmd: List[float] = []                 # PID 出力 [N]
        self.T_trim: List[float] = []                # 重力補償項
        self.T_fb: List[float] = []                  # PID FB 項
        self.pid_iz: List[float] = []                # 積分項
        self.F_d: List[np.ndarray] = []              # (T, 3) world 力外乱
        self.tau_d: List[np.ndarray] = []            # (T, 3) body トルク外乱

    def log(self, t, x, u_cmd, u_app, eta, iter_us, was_reset, x_ref,
            z_ref: float = 0.0,
            T_cmd: float = 0.0,
            T_trim: float = 0.0,
            T_fb: float = 0.0,
            pid_iz: float = 0.0,
            F_d: Optional[np.ndarray] = None,
            tau_d: Optional[np.ndarray] = None,
            x_meas: Optional[np.ndarray] = None):
        self.t.append(float(t))
        xa = pad_plant_state(x, 12)
        self.x.append(xa)
        if x_meas is None:
            self.x_meas.append(xa.copy())
        else:
            xm = pad_plant_state(x_meas, 12)
            self.x_meas.append(xm)
        xra = np.array(x_ref, dtype=np.float64)
        if xra.size == 5:
            xra = np.concatenate([xra, np.zeros(2)])
        self.x_ref.append(xra)
        self.u_cmd.append(np.array(u_cmd))
        self.u_app.append(np.array(u_app))
        self.eta.append(np.array(eta))
        self.iter_us.append(float(iter_us))
        self.was_reset.append(bool(was_reset))
        self.z_ref.append(float(z_ref))
        self.T_cmd.append(float(T_cmd))
        self.T_trim.append(float(T_trim))
        self.T_fb.append(float(T_fb))
        self.pid_iz.append(float(pid_iz))
        self.F_d.append(np.zeros(3) if F_d is None else np.array(F_d))
        self.tau_d.append(np.zeros(3) if tau_d is None else np.array(tau_d))

    def arrays(self) -> dict:
        return dict(
            t=np.asarray(self.t),
            x=np.asarray(self.x),
            x_meas=np.asarray(self.x_meas),
            u_cmd=np.asarray(self.u_cmd),
            u_app=np.asarray(self.u_app),
            eta=np.asarray(self.eta),
            iter_us=np.asarray(self.iter_us),
            was_reset=np.asarray(self.was_reset),
            x_ref=np.asarray(self.x_ref),
            z_ref=np.asarray(self.z_ref),
            T_cmd=np.asarray(self.T_cmd),
            T_trim=np.asarray(self.T_trim),
            T_fb=np.asarray(self.T_fb),
            pid_iz=np.asarray(self.pid_iz),
            F_d=np.asarray(self.F_d),
            tau_d=np.asarray(self.tau_d),
        )

    def plot(self, title: str = "", savepath: Optional[str] = None):
        """Poster layout: 2x2 plots for roll, pitch, yaw, and altitude."""
        d = self.arrays()
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        if title:
            fig.suptitle(title, fontsize=14)
        t = d['t']
        lw = 1.8

        axs[0, 0].plot(t, d['x'][:, PI_PHI], label='Measured', color='C0', lw=lw)
        axs[0, 0].plot(t, d['x_ref'][:, 0], '--', label='Reference',
                       color='C1', alpha=0.8, lw=lw)
        axs[0, 0].set_ylabel('Roll φ [rad]', fontsize=11)
        axs[0, 0].set_title('Roll', fontsize=12)

        axs[0, 1].plot(t, d['x'][:, PI_THETA], label='Measured', color='C0', lw=lw)
        axs[0, 1].plot(t, d['x_ref'][:, 1], '--', label='Reference',
                       color='C1', alpha=0.8, lw=lw)
        axs[0, 1].set_ylabel('Pitch θ [rad]', fontsize=11)
        axs[0, 1].set_title('Pitch', fontsize=12)

        axs[1, 0].plot(t, d['x'][:, PI_PSI], label='Measured', color='C0', lw=lw)
        axs[1, 0].set_ylabel('Yaw ψ [rad]', fontsize=11)
        axs[1, 0].set_title('Yaw', fontsize=12)
        axs[1, 0].set_xlabel('Time [s]', fontsize=11)

        axs[1, 1].plot(t, -d['x'][:, PI_Z], label='Measured', color='C0', lw=lw)
        axs[1, 1].plot(t, -d['z_ref'], '--', label='Reference',
                       color='C1', alpha=0.8, lw=lw)
        axs[1, 1].set_ylabel('Altitude [m]', fontsize=11)
        axs[1, 1].set_title('Altitude', fontsize=12)
        axs[1, 1].set_xlabel('Time [s]', fontsize=11)

        for ax in axs.flat:
            ax.grid(True, alpha=0.3)
            ax.legend(loc='best', fontsize=10)
            ax.tick_params(labelsize=10)

        fig.tight_layout(rect=[0, 0, 1, 0.95] if title else None)
        if savepath:
            p = Path(savepath)
            p.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(p, dpi=200, bbox_inches='tight')
            print(f"  [saved] {p}")
        return fig


# =========================================================================
# 6. Sanity Checks
# =========================================================================
def sanity_checks(params: Params) -> None:
    print("\n=== Sanity Checks ===")
    model = HexaModel(params)
    x0 = np.zeros(params.NX, dtype=np.float64)

    # 1) B_att の数値
    print("  B_att (3x6) =")
    for row in model.Batt:
        print("    [{}]".format(", ".join(f"{v:+7.3f}" for v in row)))
    expected_phi_M1 = -params.motor_y[0] * model.p.T_max / model.p.Ix
    assert abs(model.Batt[0, 0] - expected_phi_M1) < 1e-6

    trim_torque = model.Batt_nom @ np.ones(6)
    print(f"  [B_att·1] = {trim_torque}  (expect ~0)")
    assert np.max(np.abs(trim_torque)) < 1e-9, \
        f"B_c·1 != 0 (機体幾何が非対称): {trim_torque}"

    # 2) M1 のみ duty=1: actuator derivative に正しい符号が出る
    u = np.zeros(6); u[0] = 1.0
    xd = model.f(x0, u)
    print(f"  [M1 only] xdot = {xd}")
    ip, iq = (5, 6) if params.NX == 8 else (2, 3)   # NX=8 は遅れ状態 a_p,a_q、NX=5 は ṗ,q̇ を見る
    assert xd[ip] < 0 and xd[iq] > 0

    # 3) M5 のみ duty=1: q̇<0 (後右を持ち上げ → 機首下げ), ṗ<0 (右を持ち上げ)
    u = np.zeros(6); u[4] = 1.0
    xd = model.f(x0, u)
    print(f"  [M5 only] xdot = {xd}")
    assert xd[iq] < 0 and xd[ip] < 0

    # 4) ホバートリム: 全機 u_hover_nom → 角加速度 ≈ 0
    u = np.full(6, params.u_hover_nom)
    xd = model.f(x0, u)
    print(f"  [hover all-6] xdot = {xd}  (expect ~0)")
    assert np.max(np.abs(xd)) < 1e-6, "ホバートリムで角加速度が出ている"

    # 5) 総推力 = m*g（推力曲線ありはプラントの曲線で数える）
    total_thrust = (float(np.sum(thrust_curve(u, params))) if params.plant_thrust_curve
                    else np.sum(u) * params.T_max)
    print(f"  [total thrust] {total_thrust:.4f} N vs m*g={params.mass*params.gravity:.4f} N")
    assert abs(total_thrust - params.mass * params.gravity) < 1e-6

    # 6) B_c 行列のランク (ロール/ピッチ/ヨーの 3 行)
    rank_attitude = np.linalg.matrix_rank(model.Batt)
    print(f"  rank(B_att) = {rank_attitude}  (期待 3, null space 3-dim)")
    assert rank_attitude == 3

    # 7) eta は NX8 モデルへ二重適用しない（NX=5 では元シミュどおり B_eff = B·diag(η)。健全試験では無関係）
    if params.NX == 8:
        eta_test = np.array([1.0, 1.0, 1.0, 1.0, 0.5, 1.0])  # M5 部分故障
        model.set_eta(eta_test)
        assert np.allclose(model.Batt, model.Batt_nom)
        model.set_eta(np.ones(6))  # 元に戻す

    # 8) AltitudePID: ホバートリム (z=z_ref, w=0, phi=theta=0) で T_cmd ≈ m*g
    pid = AltitudePID(params)
    T_cmd, info = pid.update(z=-1.0, w=0.0, z_ref=-1.0, w_ref=0.0,
                             phi=0.0, theta=0.0, dt=params.dt)
    mg = params.mass * params.gravity
    print(f"  [PID hover] T_cmd={T_cmd:.5f} N vs m*g={mg:.5f} N "
          f"(T_trim={info['T_trim']:.5f}, T_fb={info['T_fb']:.5f})")
    assert abs(T_cmd - mg) < 1e-6, "ホバー定常で T_cmd != m*g"

    # 9) 重力補償: phi=0.3 で T_trim = m*g / cos(0.3)^2
    pid.reset()
    T_cmd2, info2 = pid.update(z=-1.0, w=0.0, z_ref=-1.0, w_ref=0.0,
                               phi=0.3, theta=0.0, dt=params.dt)
    expected_trim = mg / math.cos(0.3)
    print(f"  [PID tilt phi=0.3] T_trim={info2['T_trim']:.5f} N, "
          f"expect mg/cos(phi)={expected_trim:.5f} N")
    assert abs(info2['T_trim'] - expected_trim) < 1e-6

    # 10) アンチワインドアップ: 大誤差 & サチュフラグありで積分項が増えない
    pid.reset()
    for _ in range(20):  # 200 ms @ 100 Hz
        pid.update(z=-0.5, w=0.0, z_ref=-1.0, w_ref=0.0,
                   phi=0.0, theta=0.0, dt=params.dt,
                   sat_limit_hit=True)
    print(f"  [anti-windup] i_z after 200ms (sat=True) = {pid.i_z:.5f} (expect 0)")
    assert abs(pid.i_z) < 1e-9

    # 11) PlantModel: ホバー条件で姿勢角加速度 ≈ 0
    # PKG-E2 でプラント側の mass/T_max が NMPC とズレているため、
    # NMPC の u_hover_nom では w_dot=0 にならない。代わりに p_plant 自身の
    # ホバー duty を使って姿勢角加速度のみ検証する。地面効果は無効化前提。
    plant = PlantModel(params)
    pp = plant.p_plant
    u_hover_plant = (float(thrust_curve_inv(pp.mass * pp.gravity / params.NU, params))
                     if params.plant_thrust_curve else pp.mass * pp.gravity / (params.NU * pp.T_max))
    # 地面効果を避けるため z は ge_height より深く
    z_for_test = -max(params.ge_height + 0.5, 1.0)
    x7_h = np.zeros(params.NX_plant); x7_h[PI_Z] = z_for_test
    u_h  = np.full(params.NU, u_hover_plant)
    xdot_h = plant.f_full(x7_h, u_h, np.zeros(3), np.zeros(3))
    print(f"  [Plant hover] xdot = {xdot_h}  "
          f"(姿勢 [2:5] と w_dot [{PI_W}] が ~0 を期待, 12-dim)")
    assert np.max(np.abs(xdot_h[:5])) < 1e-6, "姿勢ホバートリム失敗"
    assert abs(xdot_h[PI_W]) < 1e-6, f"w_dot != 0: {xdot_h[PI_W]}"

    # 12) DisturbanceManager: スケジュール反映
    dm = DisturbanceManager([
        {'t_start': 1.0, 't_end': 2.0, 'kind': 'force_world',  'axis': 2, 'magnitude': -0.5},
        {'t_start': 1.0, 't_end': 2.0, 'kind': 'torque_body',  'axis': 0, 'magnitude': 0.02},
    ])
    F_a, tau_a = dm.get_wrench(0.5)
    F_b, tau_b = dm.get_wrench(1.5)
    F_c, tau_c = dm.get_wrench(2.5)
    print(f"  [Dist] t=0.5  F={F_a}, tau={tau_a} (全0期待)")
    print(f"  [Dist] t=1.5  F={F_b}, tau={tau_b} (F_z=-0.5, tau_x=0.02)")
    print(f"  [Dist] t=2.5  F={F_c}, tau={tau_c} (全0期待)")
    assert np.allclose(F_a, 0) and np.allclose(tau_a, 0)
    assert abs(F_b[2] + 0.5) < 1e-9 and abs(tau_b[0] - 0.02) < 1e-9
    assert np.allclose(F_c, 0) and np.allclose(tau_c, 0)

    # 13) t_cmd_to_u_hover (案A: η 重みつき配分)
    eta_all = np.ones(params.NU)
    uh = t_cmd_to_u_hover(mg, eta_all, params)
    print(f"  [u_hover] T_cmd=mg → per-motor = {uh[0]:.5f} (expect {params.u_hover_nom:.5f})")
    assert np.allclose(uh, params.u_hover_nom)
    # 完全故障: sum_eta=5
    eta_fail = np.array([1, 1, 1, 1, 0, 1], dtype=np.float64)
    uh2 = t_cmd_to_u_hover(mg, eta_fail, params)
    expected_per = (float(thrust_curve_inv(mg / 5.0, params)) if params.plant_thrust_curve
                    else mg / (5.0 * params.T_max))
    print(f"  [u_hover fail M5] per = {uh2[0]:.5f} (expect {expected_per:.5f}), "
          f"M5 duty = {uh2[4]:.5f} (expect 0)")
    assert abs(uh2[0] - expected_per) < 1e-9 and uh2[4] == 0.0
    # 部分故障 η=0.5: Σ(η·u)·T_max = T_cmd
    eta_partial = np.array([1, 1, 1, 1, 0.5, 1], dtype=np.float64)
    uh3 = t_cmd_to_u_hover(mg, eta_partial, params)
    thrust = (float(np.sum(eta_partial * thrust_curve(uh3, params))) if params.plant_thrust_curve
              else float(np.sum(eta_partial * uh3)) * params.T_max)
    print(f"  [u_hover partial η5=0.5] thrust={thrust:.5f} N (expect mg={mg:.5f})")
    assert abs(thrust - mg) < 1e-6

    # 14) PKG-C 加算合成: u_baseline = u_hover_nom·1 で Δu ≈ 0
    nmpc_test = AdmmNmpcHex(HexaModel(params), params)
    u_base = np.full(6, params.u_hover_nom, dtype=np.float64)
    nmpc_test.set_box(
        np.full(6, params.U_min - params.u_hover_nom),
        np.full(6, params.U_max - params.u_hover_nom),
    )
    x0_hover = np.zeros(params.NX)
    X_ref_h = np.zeros(params.NXT)
    U_ref_h = np.zeros(params.NUT)
    # warm-start を 1 回安定化させるため 2 回呼ぶ (初回は full rebuild)
    nmpc_test.step(x0_hover, X_ref_h, U_ref_h, u_baseline=u_base)
    delta_u_h = nmpc_test.step(x0_hover, X_ref_h, U_ref_h, u_baseline=u_base)
    print(f"  [加算合成 hover] Δu = {delta_u_h}  (期待: 全成分 ≈ 0)")
    assert np.max(np.abs(delta_u_h)) < 1e-3, "ホバー条件で Δu が 0 にならない"

    # 15) 加算合成: u_baseline + Δu が firmware box 内に収まる
    u_total = u_base + delta_u_h
    assert np.all(u_total >= -1e-6) and np.all(u_total <= params.U_max + 1e-6), \
        f"u_total が box を超過: {u_total}"

    # 16) PKG-D1: u_baseline ramp の補間ロジック検証
    print("\n  [PKG-D1] u_baseline ramp 補間テスト")
    u_old = np.full(6, 0.125)
    u_new = np.array([0.144, 0.144, 0.144, 0.144, 0.0, 0.144])
    eta_after = np.array([1.0, 1.0, 1.0, 1.0, 0.0, 1.0])
    beta = 0.0
    u_mid = (1.0 - beta) * u_old + beta * u_new
    u_mid = np.where(eta_after > 1.0e-3, u_mid, 0.0)
    print(f"    β=0.0  u_baseline = {u_mid}  (期待: M5=0, 他は 0.125)")
    assert abs(u_mid[4]) < 1e-9 and abs(u_mid[0] - 0.125) < 1e-9
    beta = 1.0
    u_mid = (1.0 - beta) * u_old + beta * u_new
    u_mid = np.where(eta_after > 1.0e-3, u_mid, 0.0)
    print(f"    β=1.0  u_baseline = {u_mid}  (期待: M5=0, 他は 0.144)")
    assert abs(u_mid[4]) < 1e-9 and abs(u_mid[0] - 0.144) < 1e-9
    beta = 0.5
    u_mid = (1.0 - beta) * u_old + beta * u_new
    u_mid = np.where(eta_after > 1.0e-3, u_mid, 0.0)
    print(f"    β=0.5  u_baseline = {u_mid}  (期待: M5=0, 他は 0.1345)")
    assert abs(u_mid[4]) < 1e-9 and abs(u_mid[0] - 0.1345) < 1e-6
    print("    ✓ ramp interpolation 正常")

    # =====================================================================
    # PKG-E (実機リアリティ) サニティチェック 17〜24
    # =====================================================================

    # 17) PKG-E1: モータ一次遅れの応答確認 (τ 後に 1-1/e ≈ 63%)
    print("\n  [PKG-E1] モータ一次遅れ動作確認")
    if params.tau_motor > 1e-9:
        plant_t = PlantModel(params)
        plant_t.reset_motors(np.full(params.NU, 0.0))
        u_step = np.full(params.NU, 1.0)
        n_step = max(1, int(params.tau_motor / params.dt_sim))
        for _ in range(n_step):
            u_a = plant_t.apply_actuator_dynamics(u_step, params.dt_sim)
        # アクチュエータ遅延 (delay_steps) が n_step の数 ms 程度を消費するため、
        # 一次遅れで 1-1/e に到達するのは実時間ベースで τ 経過後 (≈ 63%)。
        # 遅延ステップぶん遅れる場合は到達率が下がる。許容範囲を広めに取る。
        print(f"    τ_motor={params.tau_motor}s 後の u_actual = {u_a[0]:.4f} "
              f"(期待: 0.55〜0.70, 遅延 {params.actuator_delay_steps}step 込み)")
        assert 0.45 < u_a[0] < 0.75, f"一次遅れ応答が範囲外: {u_a[0]}"
    else:
        print("    (tau_motor=0 のため SKIP)")

    # 18) PKG-E2: プラント側慣性モーメントが NMPC と別物
    print("\n  [PKG-E2] プラント慣性モーメント誤差")
    plant_t2 = PlantModel(params)
    print(f"    NMPC I_x={params.Ix:.4e}, Plant I_x={plant_t2.p_plant.Ix:.4e} "
          f"(ratio={plant_t2.p_plant.Ix/params.Ix:.3f})")
    print(f"    NMPC mass={params.mass:.4f}, Plant mass={plant_t2.p_plant.mass:.4f}")
    assert abs(plant_t2.p_plant.Ix - params.Ix * params.plant_Ix_ratio) < 1e-12
    assert abs(plant_t2.p_plant.mass - params.mass * params.plant_mass_ratio) < 1e-12

    # 19) PKG-E3: 上昇中の推力低下 (V_z>0 で w_dot がより大きな正値)
    print("\n  [PKG-E3] 空力推力モデル(上昇時の推力低下)")
    plant_t3 = PlantModel(params)
    x_hover = np.zeros(params.NX_plant); x_hover[PI_Z] = -1.0
    x_asc   = x_hover.copy(); x_asc[PI_W] = -3.0   # w=-3 m/s (3m/s 上昇)
    u_h_e   = np.full(params.NU, params.u_hover_nom)
    xdot_h_e = plant_t3.f_full(x_hover, u_h_e, np.zeros(3), np.zeros(3))
    xdot_a_e = plant_t3.f_full(x_asc,   u_h_e, np.zeros(3), np.zeros(3))
    print(f"    ホバー w_dot   = {xdot_h_e[PI_W]:+.4f}")
    print(f"    上昇中 w_dot   = {xdot_a_e[PI_W]:+.4f}  (期待: より大きな正値=推力不足)")
    if params.w_induced_hover > 1e-9:
        assert xdot_a_e[PI_W] > xdot_h_e[PI_W] - 1e-6, "上昇時に推力が減らない"

    # 20) PKG-E4: センサ遅延 (IMU)
    print("\n  [PKG-E4] センサ遅延 (IMU)")
    if params.imu_delay_steps > 0 and params.noise_enable:
        # ノイズシード固定で再現性確保
        nm_test = NoiseModel(params, rng=np.random.default_rng(0))
        seq_phi = []
        for k in range(40):
            x_true_k = np.zeros(7)
            x_true_k[0] = 0.1 if k >= 10 else 0.0   # k=10 で phi=0.1 にステップ
            x_meas_k = nm_test.observe(x_true_k)
            seq_phi.append(float(x_meas_k[0]))
        delay_seen_at = next(
            (i for i, v in enumerate(seq_phi) if v > 0.05), -1)
        expected = 10 + params.imu_delay_steps
        print(f"    phi=0.1 step が観測値 >0.05 になるサンプル = {delay_seen_at} "
              f"(期待 ≈ {expected} ± 2)")
        assert abs(delay_seen_at - expected) <= 3, \
            f"IMU 遅延が想定外: got {delay_seen_at}, want ≈ {expected}"
    else:
        print("    (imu_delay_steps=0 または noise_enable=False のため SKIP)")

    # 21) PKG-E5: 水平移動 (姿勢傾斜で x/y 加速度が出る)
    print("\n  [PKG-E5] 水平移動 (姿勢傾斜の応答)")
    plant_t5 = PlantModel(params)
    x_tilted = np.zeros(params.NX_plant); x_tilted[PI_Z] = -1.0
    x_tilted[PI_THETA] = 0.2   # theta = +0.2 rad (前傾)
    xdot_tilt = plant_t5.f_full(x_tilted, u_h_e, np.zeros(3), np.zeros(3))
    print(f"    theta=+0.2 rad の a_x_world = {xdot_tilt[PI_UVEL]:+.4f} m/s² "
          f"(a_x = -T/m·sin θ なので theta>0 で負)")
    assert abs(xdot_tilt[PI_UVEL]) > 1e-3, "傾斜時に水平加速度が発生していない"
    # roll +0.2 → a_y_world > 0 を期待
    x_roll = np.zeros(params.NX_plant); x_roll[PI_Z] = -1.0; x_roll[PI_PHI] = 0.2
    xdot_roll = plant_t5.f_full(x_roll, u_h_e, np.zeros(3), np.zeros(3))
    print(f"    phi=+0.2 rad の a_y_world = {xdot_roll[PI_VVEL]:+.4f} m/s² "
          f"(右傾で +y 方向加速が期待)")
    assert xdot_roll[PI_VVEL] > 0.0, "右傾で +y 加速にならない"

    # 22) PKG-E6: プロペラジャイロ
    print("\n  [PKG-E6] プロペラジャイロ (健全ホバー = ジャイロ寄与 0)")
    if params.J_prop_eq > 1e-12:
        plant_t6 = PlantModel(params)
        x_spin = np.zeros(params.NX_plant); x_spin[PI_Z] = -1.0; x_spin[PI_P] = 1.0
        # ホバー入力で Σ s_i·u_i = 0 → ジャイロ寄与は 0
        s_yaw = plant_t6.attitude.s_yaw
        net = float(np.sum(s_yaw * u_h_e))
        print(f"    Σ s_i·u_i (ホバー時) = {net:.4e}  (期待 ≈ 0)")
        assert abs(net) < 1e-9
    else:
        print("    (J_prop_eq=0 のため SKIP)")

    # 23) PKG-E7: 地面効果 (低空 → 推力ブースト → w_dot がより負方向)
    print("\n  [PKG-E7] 地面効果")
    if params.ge_height > 1e-9:
        plant_t7 = PlantModel(params)
        x_low  = np.zeros(params.NX_plant); x_low[PI_Z]  = -0.1
        x_high = np.zeros(params.NX_plant); x_high[PI_Z] = -2.0
        xdot_l = plant_t7.f_full(x_low,  u_h_e, np.zeros(3), np.zeros(3))
        xdot_h = plant_t7.f_full(x_high, u_h_e, np.zeros(3), np.zeros(3))
        print(f"    高度0.1m w_dot={xdot_l[PI_W]:+.4f} (推力ブースト中)")
        print(f"    高度2.0m w_dot={xdot_h[PI_W]:+.4f} (通常)")
        assert xdot_l[PI_W] < xdot_h[PI_W], "地面効果が効いていない"
    else:
        print("    (ge_height=0 のため SKIP)")

    # 24) PKG-E8: ESC 量子化
    print("\n  [PKG-E8] ESC PWM 量子化")
    if params.esc_bits > 0:
        u_test = np.array([0.123456, 0.5, 0.999, 0.0001])
        levels = (1 << params.esc_bits) - 1
        u_q = np.round(u_test * levels) / levels
        print(f"    入力     {u_test}")
        print(f"    量子化後 {u_q}  ({params.esc_bits}-bit, {levels+1} 段階)")
        assert np.max(np.abs(u_q - u_test)) <= 1.0 / levels + 1e-9
    else:
        print("    (esc_bits=0 のため SKIP)")

    print("  → all sanity checks PASSED.\n")


# =========================================================================
# 7. シナリオランナー
# =========================================================================
def make_ref_from_schedule(t: float, schedule: List[Tuple[float, np.ndarray]]) -> np.ndarray:
    """schedule: [(t_start, x_ref_5dof), ...]  最後に t_start <= t となるものを採用."""
    ref = np.zeros(5, dtype=np.float64)
    for ts, val in schedule:
        if t >= ts:
            ref = val
    return ref


def phi_step_schedule(steps: List[Tuple[float, float]]) -> List[Tuple[float, np.ndarray]]:
    """[(t_start, phi_ref)] → 5DOF refs."""
    out = []
    for ts, phi in steps:
        x = np.zeros(5, dtype=np.float64)
        x[0] = phi
        out.append((ts, x))
    return out


def z_ref_step_schedule(steps: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """z_ref のステップ系列をそのまま返す (signature 統一用)."""
    return list(steps)


def make_z_ref(t: float, schedule: List[Tuple[float, float]], default: float) -> float:
    """[(t_start, z_ref), ...] から t 時刻の z_ref を返す."""
    z = default
    for ts, val in schedule:
        if t >= ts:
            z = val
    return z


def plot_scenario_comparison(results: dict, savepath: str) -> None:
    """全シナリオの主要メトリクスを 1 枚の棒グラフに集約。

    上段: 姿勢誤差 RMS (φ, θ) と 高度誤差 RMS (z)
    中段: 角速度ピーク (p, q) と 鉛直速度ピーク |w|
    下段: 入力サチュ率 [%] と NMPC 平均反復時間 [μs]
    """
    names = list(results.keys())
    n = len(names)

    def vals(key, scale=1.0):
        return np.array([results[k][key] * scale for k in names])

    metrics = [
        ('rms_phi',      'RMS φ error [rad]',        1.0),
        ('rms_theta',    'RMS θ error [rad]',        1.0),
        ('rms_z_err',    'RMS z error [m]',          1.0),
        ('peak_p',       'peak |p| [rad/s]',         1.0),
        ('peak_q',       'peak |q| [rad/s]',         1.0),
        ('peak_w',       'peak |w| [m/s]',           1.0),
        ('sat_rate',     'input saturation [%]',     100.0),
        ('mean_iter_us', 'NMPC mean iter [μs]',      1.0),
    ]

    fig, axes = plt.subplots(4, 2, figsize=(12, 12))
    axes = axes.ravel()
    x = np.arange(n)
    palette = plt.cm.tab10(np.linspace(0, 1, n))

    for ax, (key, title, scale) in zip(axes, metrics):
        y = vals(key, scale)
        bars = ax.bar(x, y, color=palette, edgecolor='black', linewidth=0.5)
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=0, fontsize=9)
        ax.set_title(title, fontsize=10)
        ax.grid(True, axis='y', linestyle=':', alpha=0.6)
        ymax = float(np.max(y)) if y.size else 1.0
        ax.set_ylim(0, ymax * 1.18 + 1e-9)
        for b, v in zip(bars, y):
            ax.text(b.get_x() + b.get_width() / 2, v + ymax * 0.02,
                    f"{v:.3g}", ha='center', va='bottom', fontsize=7)

    fig.suptitle(f"Scenario Performance Comparison ({names[0]}-{names[-1]})",
                 fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(savepath, dpi=140, bbox_inches='tight')
    print(f"  [saved] {savepath}")


def save_scenario_metrics_csv(results: dict, savepath: str) -> None:
    """各シナリオの全メトリクスを CSV で保存 (レポート/解析用)。"""
    keys = ['rms_phi', 'rms_theta', 'rms_z_err',
            'peak_p', 'peak_q', 'peak_r', 'peak_w',
            'sat_rate', 'mean_iter_us', 'p50_iter_us', 'p99_iter_us']
    with open(savepath, 'w') as f:
        f.write("scenario," + ",".join(keys) + "\n")
        for name, m in results.items():
            f.write(name + "," + ",".join(f"{m[k]:.6f}" for k in keys) + "\n")
    print(f"  [saved] {savepath}")


def compute_metrics(d: dict, t_settle_thresh: float = 0.05) -> dict:
    rms_phi   = float(np.sqrt(np.mean((d['x'][:, 0] - d['x_ref'][:, 0]) ** 2)))
    rms_theta = float(np.sqrt(np.mean((d['x'][:, 1] - d['x_ref'][:, 1]) ** 2)))
    peak_p = float(np.max(np.abs(d['x'][:, 2])))
    peak_q = float(np.max(np.abs(d['x'][:, 3])))
    peak_r = float(np.max(np.abs(d['x'][:, 4])))
    u_cmd = d['u_cmd']
    sat_rate = float(np.mean((u_cmd >= 0.999) | (u_cmd <= 0.001)))

    # 高度系メトリクス (z は NED 下向き正)
    if d['x'].shape[1] > PI_W and d['z_ref'].size > 0:
        z_err = d['x'][:, PI_Z] - d['z_ref']
        rms_z_err = float(np.sqrt(np.mean(z_err ** 2)))
        peak_w    = float(np.max(np.abs(d['x'][:, PI_W])))
    else:
        rms_z_err = 0.0
        peak_w    = 0.0

    # iter_us が 0 で埋まっている (NMPC 未呼出) ステップを除外して統計
    iter_us = d['iter_us']
    mask = iter_us > 0.0
    if np.any(mask):
        iter_active = iter_us[mask]
    else:
        iter_active = iter_us

    return dict(
        rms_phi=rms_phi, rms_theta=rms_theta,
        peak_p=peak_p, peak_q=peak_q, peak_r=peak_r,
        sat_rate=sat_rate,
        rms_z_err=rms_z_err, peak_w=peak_w,
        mean_iter_us=float(np.mean(iter_active)),
        p50_iter_us=float(np.percentile(iter_active, 50)),
        p99_iter_us=float(np.percentile(iter_active, 99)),
    )


# 対角の相手 [FR,FL,MR,ML,RR,RL] → FR↔RL, FL↔RR, MR↔ML（実機 kDiag と同じ。シミュの M1..M6 も同順）
DIAG_IDX = np.array([5, 4, 3, 2, 1, 0])


def ftc_metrics(d: dict, params: "Params", t_fault: Optional[float]) -> dict:
    """故障後（t ≥ t_fault）の指標。故障が無ければ NaN。"""
    nan = float('nan')
    out = dict(t_fault=(t_fault if t_fault is not None else nan))
    keys = ['alt_min_after', 'alt_drop', 'r_peak_after_dps', 'r_mean_after_dps',
            'att_peak_after_deg', 'att_rms_after_deg', 'carry_sat_after', 'u_carry_mean_after']
    if t_fault is None:
        out.update({k: nan for k in keys}); return out
    t = d['t']; m = t >= t_fault
    if np.sum(m) < 10:
        out.update({k: nan for k in keys}); return out
    x = d['x'][m]; xr = d['x_ref'][m]; u = d['u_cmd'][m]; eta = d['eta'][m][-1]
    alt = -x[:, PI_Z]
    out['alt_min_after'] = float(np.min(alt))
    out['alt_drop'] = float(-d['x'][~m][-1, PI_Z] - np.min(alt)) if np.any(~m) else nan
    out['r_peak_after_dps'] = float(np.degrees(np.max(np.abs(x[:, 4]))))
    out['r_mean_after_dps'] = float(np.degrees(np.mean(x[:, 4])))
    err = np.degrees(np.sqrt((x[:, 0] - xr[:, 0]) ** 2 + (x[:, 1] - xr[:, 1]) ** 2))
    out['att_peak_after_deg'] = float(np.max(err))
    out['att_rms_after_deg'] = float(np.sqrt(np.mean(err ** 2)))
    alive = eta > 0.5
    carry = alive & alive[DIAG_IDX]
    uc = u[:, carry] if np.any(carry) else u[:, alive]
    out['carry_sat_after'] = float(np.mean(uc >= params.U_max - 1e-6))
    out['u_carry_mean_after'] = float(np.mean(uc))
    return out


def delay_predict(model: "HexaModel", x0: np.ndarray, hist: List[np.ndarray],
                  dt: float) -> Tuple[np.ndarray, bool]:
    """実機 nmpc_delay_predict() の移植（NX=5）。hist は直近に出した duty（古い順）。"""
    SUB = 4
    h = dt / SUB
    x = np.asarray(x0, dtype=np.float64).copy()
    for u in hist:
        for _ in range(SUB):
            k1 = model.f(x, u)
            k2 = model.f(x + 0.5 * h * k1, u)
            x = x + h * k2
    d = x - x0
    ok = (np.all(np.isfinite(x)) and abs(d[0]) < 0.35 and abs(d[1]) < 0.35
          and abs(d[2]) < 10.0 and abs(d[3]) < 10.0 and abs(d[4]) < 10.0)
    return (x, True) if ok else (np.asarray(x0, dtype=np.float64).copy(), False)


def _band_share(x: np.ndarray, fs: float, lo: float, hi: float) -> float:
    """平均を引いた x のパワーのうち [lo, hi) Hz にある割合（全区間 1 本の FFT）。"""
    x = np.asarray(x, dtype=np.float64)
    if x.size < 16:
        return float('nan')
    X = np.fft.rfft((x - x.mean()) * np.hanning(x.size))
    f = np.fft.rfftfreq(x.size, 1.0 / fs)
    P = np.abs(X) ** 2
    P[0] = 0.0
    tot = P.sum()
    return float(P[(f >= lo) & (f < hi)].sum() / tot) if tot > 0 else float('nan')


def stress_metrics(d: dict, params: Params, t_switch: Optional[float],
                   diverged_t: Optional[float], diverged_why: str,
                   n_rebuild_hold: int) -> dict:
    """制御区間（PID 切替 +1 s 以降。上昇フェーズを使わない場合は t=1 s 以降）だけで測る指標。"""
    t = d['t']
    t0 = (t_switch if t_switch is not None else 0.0) + 1.0
    m = (t >= t0)
    dec = params.nmpc_decimation
    out = dict(diverged=diverged_t is not None,
               t_fail=(diverged_t if diverged_t is not None else float('nan')),
               why=diverged_why, n_rebuild_hold=n_rebuild_hold,
               t_ctrl=float(t[m][-1] - t[m][0]) if np.any(m) else 0.0)
    if np.sum(m) < 10 * dec:
        out.update(rms_phi_ctrl_deg=float('nan'), rms_theta_ctrl_deg=float('nan'),
                   peak_pq_ctrl_dps=float('nan'), p_band_2_5=float('nan'),
                   du_hf5=float('nan'), sat_box=float('nan'))
        return out
    x = d['x'][m]; xr = d['x_ref'][m]; u = d['u_cmd'][m]
    out['rms_phi_ctrl_deg'] = float(np.degrees(np.sqrt(np.mean((x[:, 0] - xr[:, 0]) ** 2))))
    out['rms_theta_ctrl_deg'] = float(np.degrees(np.sqrt(np.mean((x[:, 1] - xr[:, 1]) ** 2))))
    out['peak_pq_ctrl_dps'] = float(np.degrees(np.max(np.abs(x[:, 2:4]))))
    # 100 Hz に間引いて実機ログと同じ条件で見る
    p100 = x[::dec, 2]
    out['p_band_2_5'] = _band_share(p100, 1.0 / params.dt, 2.0, 5.0)
    u100 = u[::dec]
    du = u100 - u100.mean(axis=1, keepdims=True)       # 共通モード（高度）を除いた差分
    out['du_hf5'] = float(np.nanmean([_band_share(du[:, k], 1.0 / params.dt, 5.0, 50.1)
                                      for k in range(du.shape[1])]))
    out['sat_box'] = float(np.mean((u100 >= params.U_max - 1e-9) | (u100 <= 1e-9)))
    return out


def run_scenario(params: Params,
                 fault_mgr: FaultManager,
                 ref_schedule: List[Tuple[float, np.ndarray]],
                 fdi_known: bool,
                 t_end: float,
                 label: str,
                 savepath: Optional[str] = None,
                 x0: Optional[np.ndarray] = None,
                 dist_mgr: Optional[DisturbanceManager] = None,
                 z_ref_schedule: Optional[List[Tuple[float, float]]] = None,
                 z_ref_default: Optional[float] = None,
                 plan_open_loop_ascent: bool = False,
                 plan_z_target: Optional[float] = None,
                 plan_fault_delay_after_switch_s: float = 2.0,
                 plan_faults: Optional[List[Tuple[int, float]]] = None,
                 plan_phi_step: Optional[Tuple[float, float]] = None) -> Tuple[Logger, dict]:
    """
    マルチレートシミュレーション:
        dt_sim (1 kHz) でプラント RK4
        n%dec==0 (100 Hz) で高度 PID + NMPC を呼び、U_ref / u_cmd を ZOH で保持

    高度制御は 2 段階:
      (1) phase=ascent   : z > z_switch の間 open-loop PWM (params.pwm_ascent)
                            PID 凍結、NMPC 出力は無視 (姿勢は乱れない想定)
      (2) phase=control  : z ≤ z_switch で PID 起動 + NMPC 通常動作
                            z_ref は z_at_switch → z_target に z_ramp_duration s で線形ランプ

    plan_open_loop_ascent: True で上記 2 段階制御を有効化
    plan_z_target        : ホバー目標 (NED, デフォルトは params.z_ref_default)
    plan_fault_delay_after_switch_s: PID 切替時刻からの故障注入遅延 [s]
    plan_faults          : 故障注入 [(motor_idx, eta), ...]
    plan_phi_step        : (delay_after_fault_s, phi_rad)
    """
    print(f"\n--- Running: {label}  (FDI_known={fdi_known}) ---")
    plant   = PlantModel(params)
    nmpc_model = HexaModel(params)
    nmpc = AdmmNmpcHex(nmpc_model, params)
    alt_pid = AltitudePID(params)
    if dist_mgr is None:
        dist_mgr = DisturbanceManager()
    noise = NoiseModel(params)
    logger = Logger()
    if noise.active():
        print(f"    [noise] ON  seed={params.noise_seed}  "
              f"proc σ_pqr={params.sigma_p_proc}/{params.sigma_q_proc}/"
              f"{params.sigma_r_proc}, σ_w={params.sigma_w_proc}  "
              f"meas σ_φθ={params.sigma_phi_meas}/{params.sigma_theta_meas}, "
              f"σ_gyro={params.sigma_gyro_meas}, σ_zw={params.sigma_z_meas}/"
              f"{params.sigma_w_meas}")

    # 初期状態 (12 次元): [φ, θ, p, q, r, ψ, z, w, x_pos, y_pos, u_vel, v_vel]
    NX_PLANT = params.NX_plant
    if x0 is None:
        x7_true = np.zeros(NX_PLANT, dtype=np.float64)
    else:
        x0a = np.asarray(x0, dtype=np.float64)
        if x0a.size in (5, 7, 11, NX_PLANT):
            x7_true = pad_plant_state(x0a, NX_PLANT)
        else:
            raise ValueError(f"x0 must be 5/7/11/{NX_PLANT} dim, got {x0a.size}")

    # PKG-E1: モータ状態をシナリオ開始時に ascent PWM で初期化 (bumpless transfer)
    init_u = (np.full(params.NU, params.pwm_ascent, dtype=np.float64)
              if plan_open_loop_ascent else
              np.full(params.NU, params.u_hover_nom, dtype=np.float64))
    plant.reset_motors(init_u)

    x7_meas = noise.observe(x7_true)

    z_ref_def = params.z_ref_default if z_ref_default is None else z_ref_default
    z_sched  = z_ref_schedule if z_ref_schedule is not None else []

    # 入力 ZOH 保持変数
    u_cmd = np.full(6, params.u_hover_nom, dtype=np.float64)
    u_hover = u_cmd.copy()
    U_ref = np.tile(u_hover, params.N)
    last_iter_us  = 0.0
    last_was_reset = False
    x_ref = np.zeros(params.NX, dtype=np.float64)
    a_hat = nmpc_model.Batt @ init_u

    # ---- STRESS: 計算遅延・再構築停滞 ----
    pending: deque = deque()        # (出力する制御周期番号, u_cmd)
    busy_until_tick = -1            # この周期番号までは NMPC を呼べない（再構築中）
    n_rebuild_hold = 0
    diverged_t: Optional[float] = None
    diverged_why = ""
    # 実機合わせ: 遅れ補償用の出力 duty 履歴（実機 G_DUTY_HIST_N=8、NMPC を回した周期ごとに 1 個積む）
    duty_hist: deque = deque([init_u.copy() for _ in range(8)], maxlen=8)
    t_fault_evt: Optional[float] = None
    n_dc_reject = 0

    # ---- PKG-C: LPF は撤去 (tau=0 で通過動作)、変数は構造保持のため維持 ----
    _tau = params.u_ref_lpf_tau
    dt_ctrl = params.dt              # NMPC / PID 共通 = dt_sim * decimation
    alpha_lpf: float = dt_ctrl / (_tau + dt_ctrl) if _tau > 1e-9 else 1.0
    u_hover_prev = u_hover.copy()
    u_baseline = u_hover.copy()      # PKG-C: 加算合成モード用 baseline (control 中のみ意味を持つ)

    # 高度 PID ログ用 ZOH (control フェーズ、非 decimation ステップで保持)
    _mg = params.mass * params.gravity
    T_cmd = _mg
    pid_info: dict = dict(e_z=0.0, i_z=0.0, T_trim=_mg, T_fb=0.0)

    dt_sim = params.dt_sim
    dec    = params.nmpc_decimation
    n_sim_steps = int(round(t_end / dt_sim))

    # ---- プラン (open-loop ascent + 絶対時刻イベント) ----
    use_plan = plan_open_loop_ascent
    phase: str = "ascent" if use_plan else "control"
    z_target = (plan_z_target if plan_z_target is not None else z_ref_def)
    z_switch = z_target + params.z_switch_offset      # NED, 目標より 0.3 m 上 (z 軸下向き正なので "+offset" で「30 cm 下から」)
    z_at_switch: float = 0.0
    w_at_switch: float = 0.0    # ascent→control 切替時の w を保存 (bumpless transfer 用)
    t_switch: Optional[float] = None
    t_fault: Optional[float] = None
    fault_done = False
    faults_to_apply = list(plan_faults) if plan_faults is not None else []
    ascent_pwm_vec = np.full(6, params.pwm_ascent, dtype=np.float64)

    # PKG-D: bumpless transfer の現在の起点 (ascent→control + 故障時に再設定)
    t_bumpless_restart: Optional[float] = None
    z_at_bumpless_restart: float = 0.0
    w_at_bumpless_restart: float = 0.0

    # PKG-D1: u_baseline 故障時 ramp
    u_baseline_pre_fault: Optional[np.ndarray] = None
    t_baseline_ramp_start: Optional[float] = None

    # plan 無しで最初から control の場合は bumpless 起点を t=0 に
    if phase == "control":
        t_bumpless_restart = 0.0
        z_at_bumpless_restart = float(x7_true[PI_Z])
        w_at_bumpless_restart = float(x7_true[PI_W])

    if use_plan:
        print(f"    [plan] phase=ascent  open-loop PWM={params.pwm_ascent:.3f}, "
              f"z_target={z_target:.2f} m, z_switch={z_switch:.2f} m, "
              f"fault_delay={plan_fault_delay_after_switch_s:.1f}s after switch")

    for n in range(n_sim_steps):
        t = n * dt_sim

        # FaultManager / 外乱 (毎周期更新)
        eta, events = fault_mgr.update(t, dt_sim)
        # PKG-F: 制御層が信じる η（FDI 未知は全機健全と仮定、プラントは真 η）
        eta_control = (fault_mgr.eta.copy() if fdi_known
                       else np.ones(params.NU, dtype=np.float64))
        F_world, tau_body = dist_mgr.get_wrench(t)

        # ---- フェーズ遷移: ascent → control (PID 切替) ----
        if phase == "ascent" and float(x7_true[PI_Z]) <= z_switch:
            phase = "control"
            t_switch = t
            z_at_switch = float(x7_true[PI_Z])
            w_at_switch = float(x7_true[PI_W])         # PKG-A: bumpless ramp 用に w を保存
            alt_pid.reset()
            # A: 切替時に LPF 初期値を ascent PWM に合わせる (ステップ防止)
            u_hover_prev = ascent_pwm_vec.copy()
            # PKG-C: 加算合成モードに切替 → 絶対 u の warm-start を破棄
            nmpc.U[:] = 0.0           # Δu の warm-start は 0 から
            nmpc.z[:] = 0.0
            nmpc.lam[:] = 0.0
            nmpc.ws_valid = True
            nmpc.force_reset()         # SR2 履歴も破棄、次周期で _full_rebuild_sx_su
            # PKG-D: bumpless restart の起点を初期化
            t_bumpless_restart = t
            z_at_bumpless_restart = z_at_switch
            w_at_bumpless_restart = w_at_switch
            print(f"    [plan] phase=control t={t:.3f}s  "
                  f"(z={z_at_switch:.3f} m, w={w_at_switch:.3f} m/s, "
                  f"PID 起動, 加算合成モード ON, "
                  f"w_ref_ramp={params.w_ref_ramp_duration}s, "
                  f"kd_fadein={params.kd_fadein_duration}s)")

        # ---- フェーズ遷移: control → faulted (絶対時刻、切替+delay) ----
        if (use_plan and not fault_done and t_switch is not None
                and t >= t_switch + plan_fault_delay_after_switch_s):
            fault_done = True
            t_fault = t
            if faults_to_apply:
                plan_events = fault_mgr.apply_faults(faults_to_apply)
                events = events + plan_events
                desc = ", ".join(
                    f"M{k+1} η→{et:.2f}" for k, et in faults_to_apply)
                print(f"    [plan] 故障適用 t={t:.3f}s "
                      f"(switch+{plan_fault_delay_after_switch_s:.1f}s): {desc}")
            else:
                print(f"    [plan] 故障なしマーカ t={t:.3f}s "
                      f"(S1 健全、event 同期点)")

        # ---- FDI 既知ケース: Λ (B_eff) / box / SR2 + PKG-D bumpless ----
        # u_max は η で**縮減しない** (B_eff にスケール反映済)。η≈0 のみ u_max=0。
        if events:
            t_fault_evt = t
        if events and fdi_known and params.ftc_formulation == "firmware":
            # 実機: スイッチで Motor_eta を 0 にするだけ（B・R・Hessian は触らない、ランプ無し）
            print(f"    [FTC firmware @t={t:.3f}s] " + ", ".join(
                f"M{k+1} η→{e:.2f}" for k, e in events) + "（box と釣り合い参照だけ変更）")
        elif events and fdi_known:
            # PKG-D1: u_baseline ramp の出発点
            u_baseline_pre_fault = u_baseline.copy()
            t_baseline_ramp_start = t

            # PKG-D3+D4: bumpless transfer 再起動
            t_bumpless_restart = t
            z_at_bumpless_restart = float(x7_true[PI_Z])
            w_at_bumpless_restart = float(x7_true[PI_W])

            # PKG-D3: PID 積分項を減衰
            alt_pid.i_z *= params.fault_iz_attenuation

            for k_fail, eta_t in events:
                nmpc_model.set_eta(fault_mgr.eta)
                nmpc.set_rbar_from_eta(fault_mgr.eta)  # 案B
                new_u_max = nmpc.u_max_per_motor.copy()
                if float(eta_t) <= 1.0e-3:
                    new_u_max[k_fail] = 0.0
                else:
                    new_u_max[k_fail] = float(params.U_max)
                nmpc.set_box(nmpc.u_min_per_motor, new_u_max)
                nmpc.force_reset()
                n_alive_dbg = int(np.sum(fault_mgr.eta > 1.0e-3))
                print(f"    [FDI@t={t:.3f}s] M{k_fail+1} η→{eta_t:.2f}, "
                      f"u_max[{k_fail}]→{new_u_max[k_fail]:.2f}, "
                      f"alive={n_alive_dbg}, B_eff updated, SR2 reset, "
                      f"bumpless restart (kd_fadein, w_ref ramp, z_ref ramp, "
                      f"i_z×{params.fault_iz_attenuation:.2f})")

        # ---- 姿勢参照 (プラン時: 故障+delay で roll ステップ) ----
        if use_plan:
            x_ref = np.zeros(params.NX, dtype=np.float64)
            if (plan_phi_step is not None and fault_done
                    and t_fault is not None):
                delay_s, phi_val = plan_phi_step
                if t >= t_fault + delay_s:
                    x_ref[0] = float(phi_val)
        else:
            x_ref5 = make_ref_from_schedule(t, ref_schedule)
            x_ref = np.zeros(params.NX, dtype=np.float64)
            x_ref[:5] = x_ref5

        # ---- z_ref の決定 (ascent 中は不問、control 後はランプ) ----
        if use_plan:
            if phase == "ascent":
                # ascent: PID 凍結。z_ref はログ用に z_target を入れる (実制御には未使用)
                z_ref_t = z_target
            else:
                # control: z_at_bumpless_restart → z_target (PKG-D: 故障時に再起動)
                restart_ref = t_bumpless_restart if t_bumpless_restart is not None else t
                tau = t - restart_ref
                if tau >= params.z_ramp_duration:
                    z_ref_t = z_target
                else:
                    alpha = tau / max(params.z_ramp_duration, 1e-6)
                    z_ref_t = ((1.0 - alpha) * z_at_bumpless_restart
                                 + alpha * z_target)
        else:
            z_ref_t = make_z_ref(t, z_sched, z_ref_def)

        # ---- ascent: U_ref / ログ用推力 (毎 dt_sim、PID は未使用) ----
        if phase == "ascent":
            T_trim = params.mass * params.gravity / max(
                math.cos(float(x7_true[0])) * math.cos(float(x7_true[1])),
                params.trim_clip_cos)
            T_cmd = float(np.sum(ascent_pwm_vec)) * params.T_max
            pid_info = dict(e_z=0.0, i_z=0.0, T_trim=T_trim,
                            T_fb=T_cmd - T_trim)
            U_ref = np.tile(ascent_pwm_vec, params.N)
            u_hover_prev = ascent_pwm_vec.copy()

        # ---- STRESS: 予定の周期に達した指令を出力へ（以後 ZOH） ----
        is_ctrl_tick = (n % dec == 0)
        k_tick = n // dec
        if is_ctrl_tick and phase == "control":
            while pending and pending[0][0] <= k_tick:
                u_cmd = pending.popleft()[1]
        nmpc_free = k_tick > busy_until_tick

        # ---- 100 Hz: 高度 PID + NMPC (decimation 周期、ZOH) ----
        if is_ctrl_tick and nmpc_free:
            if phase == "control":
                # PKG-A+D Bumpless: restart 起点から kd_scale / w_ref ramp
                restart_ref = (t_bumpless_restart if t_bumpless_restart is not None
                               else t)
                tau_since_restart = t - restart_ref
                if params.kd_fadein_duration > 1e-9:
                    kd_scale = float(np.clip(
                        tau_since_restart / params.kd_fadein_duration,
                        0.0, 1.0))
                else:
                    kd_scale = 1.0
                if params.w_ref_ramp_duration > 1e-9:
                    ramp_w = max(0.0,
                                 1.0 - tau_since_restart
                                 / params.w_ref_ramp_duration)
                    w_ref_t = w_at_bumpless_restart * ramp_w
                else:
                    w_ref_t = 0.0

                sat_hit = bool(np.any((u_cmd >= 0.999) | (u_cmd <= 0.001)))
                T_cmd, pid_info = alt_pid.update(
                    z=float(x7_meas[PI_Z]), w=float(x7_meas[PI_W]),
                    z_ref=z_ref_t, w_ref=w_ref_t,
                    phi=float(x7_meas[0]), theta=float(x7_meas[1]),
                    dt=dt_ctrl, sat_limit_hit=sat_hit,
                    kd_scale=kd_scale)

                # PKG-C: PID 出力 → u_baseline (PKG-F: eta_control で配分)
                u_baseline_raw = t_cmd_to_u_hover(T_cmd, eta_control, params)
                # LPF は撤去 (tau=0 で alpha_lpf=1 → 通過)。構造維持のため式は残す
                u_hover = (alpha_lpf * u_baseline_raw
                           + (1.0 - alpha_lpf) * u_hover_prev)
                u_hover_prev = u_hover.copy()

                # PKG-D1: 故障直後 u_baseline_pre_fault → u_hover を線形 ramp
                if (u_baseline_pre_fault is not None
                        and t_baseline_ramp_start is not None
                        and params.fault_baseline_ramp_duration > 1e-9):
                    tau_ramp = t - t_baseline_ramp_start
                    if tau_ramp < params.fault_baseline_ramp_duration:
                        beta = (tau_ramp
                                / max(params.fault_baseline_ramp_duration, 1e-9))
                        u_baseline = ((1.0 - beta) * u_baseline_pre_fault
                                      + beta * u_hover)
                        u_baseline = np.where(eta > 1.0e-3, u_baseline, 0.0)
                    else:
                        u_baseline = u_hover.copy()
                        u_baseline_pre_fault = None
                        t_baseline_ramp_start = None
                else:
                    u_baseline = u_hover.copy()

                # PKG-C: NMPC の box 制約を動的更新
                #   Δu ∈ [U_min - u_baseline, U_max - u_baseline]
                #   FDI 既知: 故障機 (η≈0) は Δu=0。FDI 未知(PKG-F): 全機 box 開放
                new_u_min = np.zeros(params.NU, dtype=np.float64)
                new_u_max = np.zeros(params.NU, dtype=np.float64)
                alive_mask = eta_control > 1.0e-3
                for k_m in range(params.NU):
                    if alive_mask[k_m]:
                        new_u_min[k_m] = params.U_min - float(u_baseline[k_m])
                        new_u_max[k_m] = params.U_max - float(u_baseline[k_m])
                    else:
                        new_u_min[k_m] = 0.0
                        new_u_max[k_m] = 0.0
                nmpc.set_box(new_u_min, new_u_max)

                # PKG-C: U_ref は 0 ベクトル (Δu のホバー基準は 0)
                U_ref = np.zeros(params.NUT, dtype=np.float64)

                if params.ftc_formulation == "firmware":
                    # 実機 control.cpp と同じ: 共通スロットル u_h、box は η だけ、参照は釣り合い配分
                    alive = eta_control > 0.5
                    carry = alive & alive[DIAG_IDX]
                    n_carry = int(np.sum(carry))
                    T_req = T_cmd
                    if params.ftc_throttle_hold and t_fault_evt is not None:
                        T_req = params.mass * params.gravity   # 静的釣り合い（傾き 0）に固定
                    if n_carry > 0:
                        # 操縦者が「釣り合い配分で T_req が出る」スロットルを探す理想化
                        if params.plant_thrust_curve:
                            u_carry = float(thrust_curve_inv(T_req / n_carry, params))
                        else:
                            u_carry = T_req / (n_carry * params.T_max)
                        u_h = u_carry * n_carry / params.NU
                    else:
                        u_h = 0.0
                    u_h = float(np.clip(u_h, 0.0, params.U_max))
                    u_baseline = np.full(params.NU, u_h, dtype=np.float64)
                    nmpc.set_box(np.full(params.NU, -u_h),
                                 eta_control * params.U_max - u_h)
                    shift = np.zeros(params.NU, dtype=np.float64)
                    if 0 < n_carry < params.NU:
                        shift = np.where(carry, u_h * (params.NU - n_carry) / n_carry, -u_h)
                    U_ref = np.tile(shift, params.N)

            X_ref = np.tile(x_ref, params.N)
            t0 = time.perf_counter()
            if phase == "control":
                # PKG-C: 加算合成モード (NMPC は観測状態 x7_meas を使用)
                x_ctrl = (x7_meas[:5].copy() if params.NX == 5
                          else np.concatenate([x7_meas[:5], a_hat]))
                if params.NX == 5 and params.delay_comp_steps > 0:
                    n_dc = min(params.delay_comp_steps, len(duty_hist))
                    x_ctrl, dc_ok = delay_predict(nmpc_model, x_ctrl, list(duty_hist)[-n_dc:], params.dt)
                    n_dc_reject += (0 if dc_ok else 1)
                try:
                    delta_u = nmpc.step(x_ctrl, X_ref, U_ref, u_baseline=u_baseline)
                except np.linalg.LinAlgError:
                    diverged_t, diverged_why = t, "solver(LinAlgError)"
                    break
                if not np.all(np.isfinite(delta_u)):
                    diverged_t, diverged_why = t, "solver(non-finite)"
                    break
                u_new = np.clip(u_baseline + delta_u, 0.0, params.U_max)
                if params.ftc_formulation == "firmware":
                    # 実機 400 Hz 段の clamp: 故障発 0、故障発の対角は下限なし、その他は minimum_duty
                    dead = eta_control <= 0.5
                    floor = np.where(dead[DIAG_IDX], 0.0, params.min_duty_fw)
                    u_new = np.where(dead, 0.0, np.maximum(u_new, floor))
                duty_hist.append(u_new.copy())          # 実機 duty_hist_push() 相当
                # STRESS: 再構築した周期は rebuild_hold_cycles 周期遅れで出し、その間 NMPC を止める
                if nmpc.last_was_reset and params.rebuild_hold_cycles > 0:
                    lag = max(params.compute_delay_cycles, params.rebuild_hold_cycles)
                    busy_until_tick = k_tick + params.rebuild_hold_cycles - 1
                    n_rebuild_hold += 1
                else:
                    lag = params.compute_delay_cycles
                if lag <= 0:
                    u_cmd = u_new
                else:
                    pending.append((k_tick + lag, u_new))
            else:
                # ascent: 旧来通り絶対モード (u_cmd は下で ascent_pwm_vec に上書き)
                x_ctrl = (x7_meas[:5].copy() if params.NX == 5
                          else np.concatenate([x7_meas[:5], a_hat]))
                _ = nmpc.step(x_ctrl, X_ref, U_ref)
            last_iter_us  = (time.perf_counter() - t0) * 1.0e6
            last_was_reset = nmpc.last_was_reset
        else:
            last_was_reset = False

        # ---- ascent フェーズは open-loop PWM (NMPC 出力は無視) ----
        if phase == "ascent":
            u_cmd = ascent_pwm_vec.copy()

        # ---- 毎 dt_sim: PKG-E8 量子化 → PKG-E1 アクチュエータ → プラント RK4 ----
        if params.esc_bits > 0:
            levels = (1 << params.esc_bits) - 1
            u_cmd_q = np.clip(np.round(u_cmd * levels) / levels, 0.0, 1.0)
        else:
            u_cmd_q = u_cmd
        alpha_hat = math.exp(-dt_sim / params.tau_act)
        a_hat = alpha_hat * a_hat + (1.0 - alpha_hat) * (nmpc_model.Batt @ u_cmd_q)
        u_actual = plant.apply_actuator_dynamics(u_cmd_q, dt_sim)
        u_applied = eta * u_actual
        x7_true = plant.step_rk4(x7_true, u_applied, F_world, tau_body, dt_sim)
        x7_true = noise.apply_process(x7_true, dt_sim)
        x7_meas = noise.observe(x7_true)

        # ---- ログ (真値 x7_true / 観測 x7_meas) ----
        # x_ref を 7 次元に拡張 (高度参照を含める)
        x_ref_full = np.concatenate([x_ref, [z_ref_t, 0.0]])
        logger.log(t, x7_true, u_cmd, u_applied, eta,
                   last_iter_us, last_was_reset, x_ref_full,
                   z_ref=z_ref_t,
                   T_cmd=T_cmd, T_trim=pid_info['T_trim'],
                   T_fb=pid_info['T_fb'], pid_iz=pid_info['i_z'],
                   F_d=F_world, tau_d=tau_body,
                   x_meas=x7_meas)

        # ---- STRESS: 実機フェイルセーフ相当の発散判定（判定した周期まではログに残す） ----
        if diverged_t is None:
            if not np.all(np.isfinite(x7_true[:5])):
                diverged_t, diverged_why = t, "non-finite state"
            elif max(abs(x7_true[0]), abs(x7_true[1])) > math.radians(params.divergence_angle_deg):
                diverged_t, diverged_why = t, f"|angle|>{params.divergence_angle_deg:.0f}deg"
            elif float(np.max(np.abs(x7_true[2:5]))) > math.radians(params.divergence_rate_dps):
                diverged_t, diverged_why = t, f"|rate|>{params.divergence_rate_dps:.0f}deg/s"
            elif -float(x7_true[PI_Z]) < 0.0:
                diverged_t, diverged_why = t, "altitude<0 (落下)"
        if diverged_t is not None:
            break

    d = logger.arrays()
    m = compute_metrics(d)
    m.update(stress_metrics(d, params, t_switch, diverged_t, diverged_why, n_rebuild_hold))
    m['n_dc_reject'] = n_dc_reject
    m.update(ftc_metrics(d, params, t_fault_evt))
    u_ctrl = d['u_cmd'][d['t'] >= 1.0]
    m['u_mean'] = float(np.mean(u_ctrl)) if u_ctrl.size else float('nan')
    n_reset_total = int(np.sum(d['was_reset']))
    n_ctrl_calls  = int(round(n_sim_steps / dec))
    print(f"  steps={n_sim_steps} (dt_sim), NMPC/alt_PID calls={n_ctrl_calls}, "
          f"full_rebuild={n_reset_total}回, sim 時間={n_sim_steps*dt_sim:.2f}s")
    print(f"  追従誤差   RMS phi={m['rms_phi']:.4f} rad, RMS theta={m['rms_theta']:.4f} rad, "
          f"RMS z_err={m['rms_z_err']:.4f} m")
    print(f"  角速度ピーク p={m['peak_p']:.3f}, q={m['peak_q']:.3f}, r={m['peak_r']:.3f} rad/s, "
          f"peak |w|={m['peak_w']:.3f} m/s")
    print(f"  サチュ率    {m['sat_rate']*100:.2f} %")
    print(f"  NMPC iter   mean={m['mean_iter_us']:.1f} us, "
          f"p50={m['p50_iter_us']:.1f} us, p99={m['p99_iter_us']:.1f} us")
    print(f"  → 制御ループ実行可能周波数(mean) ≈ {1e6/m['mean_iter_us']:.0f} Hz "
          f"(目標 100 Hz)")
    print(f"  [STRESS] 発散={'あり t=%.2fs (%s)' % (diverged_t, diverged_why) if diverged_t is not None else 'なし'}  "
          f"再構築で保持={n_rebuild_hold} 回  "
          f"制御区間: RMS φ={m['rms_phi_ctrl_deg']:.2f}° θ={m['rms_theta_ctrl_deg']:.2f}°  "
          f"p の 2–5 Hz 帯={m['p_band_2_5']*100:.1f}%  du の 5 Hz 超={m['du_hf5']*100:.1f}%  "
          f"0.95/0 張り付き={m['sat_box']*100:.2f}%")

    fig = logger.plot(title=label, savepath=savepath)
    return logger, m


# =========================================================================
# 8. main  (実機合わせ NX=5/N=14: 遅れ補償 DC0 と DC6 の比較)
# =========================================================================
# 条件: 名前 → (計算遅延[周期], 効き倍率, 再構築保持[周期], バイアスRW, 姿勢漏れtau)
CONDITIONS = [
    ("base",   0, 1.0, 0, 0.0,   0.0),   # 悪条件なし
    ("R1",     1, 1.0, 3, 0.005, 1.0),   # 実機相当: 計算遅延 1 + 再構築保持 3 + バイアス
    ("R2",     2, 1.0, 3, 0.005, 1.0),
    ("R3",     3, 1.0, 3, 0.005, 1.0),
    ("R1_g2",  1, 2.0, 3, 0.005, 1.0),   # 感度: R1 + 機体の効き 2 倍
]
DC_VARIANTS = [("DC0", 0, "遅れ補償なし"), ("DC6", 6, "遅れ補償あり（DC6）")]

SUMMARY_KEYS = ['diverged', 't_fail', 'rms_phi_ctrl_deg', 'rms_theta_ctrl_deg',
                'peak_pq_ctrl_dps', 'p_band_2_5', 'du_hf5', 'sat_box', 'n_rebuild_hold',
                'n_dc_reject', 'u_mean']


def make_case_params(cond, dc_steps: int, seed: int, nx: int = 5, n_h: int = 14) -> Params:
    _, delay, gain, hold, bias_rw, att_tau = cond
    p = Params()
    p.NX, p.N = nx, n_h
    p.compute_delay_cycles = delay
    p.plant_moment_gain = gain
    p.rebuild_hold_cycles = hold
    p.sigma_gyro_bias_rw = bias_rw
    p.gyro_bias_att_tau = att_tau
    p.delay_comp_steps = dc_steps
    p.noise_seed = seed
    apply_aero(p)
    return p


AERO_ARGS = ("off", 1.0)


def apply_aero(p: "Params") -> "Params":
    """--aero full: SIL（plant6dof.preset('full')）と同じ推定値を入れる。"""
    mode, k = AERO_ARGS
    if mode == "full":
        p.C_drag_u, p.C_drag_v, p.C_drag_w = 0.006, 0.006, 0.025
        p.k_rotor_drag = 0.03
        p.k_bf_roll, p.k_bf_pitch = 7.5e-3 * k, 2.8e-3 * k
    return p


def plot_stress_overlay(runs: dict, savepath: str) -> None:
    """真の φ, θ [deg]。列 = 遅れ補償の有無、色 = 条件（同条件の別シードは同色）。"""
    groups = list(dict.fromkeys(r['group'] for r in runs.values()))
    fig, axs = plt.subplots(2, len(groups), figsize=(8 * len(groups), 8), squeeze=False)
    for col, g in enumerate(groups):
        keys = [k for k, r in runs.items() if r['group'] == g]
        conds = list(dict.fromkeys(runs[k]['cond'] for k in keys))
        colors = {c: plt.cm.tab10(i % 10) for i, c in enumerate(conds)}
        for c in conds:
            ks = [k for k in keys if runs[k]['cond'] == c]
            nfail = sum(runs[k]['m']['diverged'] for k in ks)
            lab = f"{c}  発散 {nfail}/{len(ks)}"
            for n_k, k in enumerate(ks):
                d = runs[k]['d']
                for r in range(2):
                    axs[r, col].plot(d['t'], np.degrees(d['x'][:, r]), lw=1.0, color=colors[c],
                                     alpha=0.85, label=(lab if n_k == 0 else None))
        axs[0, col].set_title(f"{g}  ロール φ [deg]")
        axs[1, col].set_title(f"{g}  ピッチ θ [deg]")
        axs[1, col].set_xlabel("時刻 [s]（t=8 s にロール参照ステップ）")
        for r in range(2):
            axs[r, col].set_ylim(-40, 40)
            axs[r, col].grid(True, alpha=0.3)
        axs[0, col].legend(fontsize=8, loc='upper left', ncol=2)
    fig.tight_layout()
    fig.savefig(savepath, dpi=140, bbox_inches='tight')
    print(f"  [saved] {savepath}")


def main():
    import argparse
    ap = argparse.ArgumentParser(description="実機合わせ NX=5/N=14 NMPC: 遅れ補償 DC0/DC6 比較")
    ap.add_argument("--seeds", default="42,7,123", help="雑音シード（カンマ区切り）")
    ap.add_argument("--only", default="", help="この文字列を含む条件だけ（例: R1）")
    ap.add_argument("--t-end", type=float, default=15.0)
    ap.add_argument("--phi-step-deg", type=float, default=5.0)
    ap.add_argument("--step-t", type=float, default=8.0)
    ap.add_argument("--suite", choices=["healthy", "ftc"], default="healthy",
                    help="healthy: 健全ホバーの DC0/DC6 比較 / ftc: FR 停止の耐故障試験")
    ap.add_argument("--ftc-form", default="all", help="ftc: FW / SIM / all")
    ap.add_argument("--mass", type=int, default=0, help="ftc: 質量 [g]（0 で 285/271/250 すべて）")
    ap.add_argument("--t-fault", type=float, default=5.0, help="ftc: FR を止める時刻 [s]")
    ap.add_argument("--throttle", choices=["pid", "hold", "both"], default="both",
                    help="ftc: 故障後のスロットル（高度 PID / 釣り合い値で固定 / 両方）")
    ap.add_argument("--aero", choices=["off", "full"], default="off",
                    help="PKG-E10 の空力（抗力・ロータ抗力・ブローバック）を推定値で入れる（2026-10-08）")
    ap.add_argument("--bf-scale", type=float, default=1.0, help="--aero full のときのブローバック係数の倍率")
    args = ap.parse_args()
    global AERO_ARGS
    AERO_ARGS = (args.aero, args.bf_scale)
    if args.suite == "ftc":
        main_ftc(args)
        return

    wall_t0 = time.time()
    base = Params()
    print("=" * 70)
    print(" 実機合わせ NX=5/N=14 NMPC: 遅れ補償 DC0 / DC6 比較")
    print(f"  u_hover_nom={base.u_hover_nom:.4f}（実測 0.5506）  T_max(model)={base.T_max}  mass={base.mass}")
    print("=" * 70)
    sanity_checks(base)
    img_dir = make_run_img_dir()
    print(f"  PNG 保存先: {img_dir}")

    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]
    runs: dict = {}
    for dc_name, dc_steps, dc_label in DC_VARIANTS:
        for cond in CONDITIONS:
            if args.only and args.only not in cond[0]:
                continue
            for seed in seeds:
                p = make_case_params(cond, dc_steps, seed)
                key = f"{dc_name}_{cond[0]}_s{seed}"
                x0_hover = np.zeros(p.NX_plant)
                x0_hover[PI_Z] = p.z_ref_default
                ref = phi_step_schedule([(0.0, 0.0), (args.step_t, math.radians(args.phi_step_deg))])
                label = (f"{key}: NX={p.NX} N={p.N} DC={p.delay_comp_steps} delay={p.compute_delay_cycles} "
                         f"gain={p.plant_moment_gain} hold={p.rebuild_hold_cycles} bias_rw={p.sigma_gyro_bias_rw}")
                try:
                    logger, m = run_scenario(
                        p, FaultManager(schedule=[]), ref, fdi_known=True,
                        t_end=args.t_end, label=label,
                        savepath=str(img_dir / f"stress_{key}.png"),
                        x0=x0_hover, z_ref_default=p.z_ref_default,
                        plan_open_loop_ascent=False)
                finally:
                    plt.close('all')
                runs[key] = dict(m=m, d=logger.arrays(), group=dc_label, cond=cond[0])

    print("\n" + "=" * 132)
    print(" Summary（t=1 s 以降。ロール参照は t=step_t で 0 → phi_step_deg）")
    print("=" * 132)
    hdr = (f" {'case':<16} | {'発散':>4} | {'t_fail':>6} | {'RMSφ°':>6} | {'RMSθ°':>6} | {'peak pq°/s':>10} | "
           f"{'p 2-5Hz':>7} | {'du>5Hz':>6} | {'張付%':>6} | {'保持回':>5} | {'DC棄却':>6} | {'u平均':>6}")
    print(hdr)
    print(" " + "-" * (len(hdr) - 1))
    for k, r in runs.items():
        m = r['m']
        print(f" {k:<16} | {'YES' if m['diverged'] else '-':>4} | {m['t_fail']:>6.2f} | "
              f"{m['rms_phi_ctrl_deg']:>6.2f} | {m['rms_theta_ctrl_deg']:>6.2f} | {m['peak_pq_ctrl_dps']:>10.1f} | "
              f"{m['p_band_2_5']*100:>6.1f}% | {m['du_hf5']*100:>5.1f}% | {m['sat_box']*100:>6.2f} | "
              f"{m['n_rebuild_hold']:>5d} | {m['n_dc_reject']:>6d} | {m['u_mean']:>6.3f}")

    csv_path = img_dir / "stress_metrics.csv"
    with open(csv_path, "w") as f:
        f.write("case," + ",".join(SUMMARY_KEYS) + ",why\n")
        for k, r in runs.items():
            m = r['m']
            f.write(k + "," + ",".join(str(m[x]) for x in SUMMARY_KEYS) + f",{m['why']}\n")
    print(f"  [saved] {csv_path}")
    plot_stress_overlay(runs, str(img_dir / "stress_overlay.png"))
    print(f"\n 全体実行時間: {time.time() - wall_t0:.1f} 秒   PNG 保存先: {img_dir}")



# =========================================================================
# 9. 耐故障制御（FTC）試験: ホバー中の t=t_fault に FR（M1）が完全停止
# =========================================================================
FTC_FORMS = [("FW", "firmware", "実機方式"), ("SIM", "sim", "元シミュ方式")]
FTC_MASSES = [285, 271, 250]            # [g]（271 g は 20260916_static_4motor_ftc_viable_at_271g、250 g は閾値探し）
FTC_CONDS = [c for c in CONDITIONS if c[0] in ("base", "R1")]
FTC_KEYS = ['diverged', 't_fail', 'why', 't_fault', 'alt_min_after', 'alt_drop',
            'att_peak_after_deg', 'att_rms_after_deg', 'r_peak_after_dps', 'r_mean_after_dps',
            'carry_sat_after', 'u_carry_mean_after', 'n_dc_reject', 'n_rebuild_hold']


def plot_ftc_overlay(runs: dict, t_fault: float, savepath: str) -> None:
    """列 = 方式×質量、行 = |姿勢誤差|・ヨー率 r・高度・担当発の平均 duty。色 = DC×条件。"""
    cols = list(dict.fromkeys(r['col'] for r in runs.values()))
    rows = [("姿勢 |φ|,|θ| の大きい方 [deg]", lambda d: np.degrees(np.max(np.abs(d['x'][:, 0:2]), axis=1)), (0, 40)),
            ("ヨー率 r [deg/s]", lambda d: np.degrees(d['x'][:, 4]), (-200, 200)),
            ("高度 [m]", lambda d: -d['x'][:, PI_Z], (0, 2.6)),
            ("担当 4 発の平均 duty", lambda d: np.mean(d['u_cmd'][:, [1, 2, 3, 4]], axis=1), (0, 1.0))]
    fig, axs = plt.subplots(len(rows), len(cols), figsize=(5.2 * len(cols), 3.0 * len(rows)),
                            squeeze=False, sharex=True)
    for c_i, col in enumerate(cols):
        keys = [k for k, r in runs.items() if r['col'] == col]
        conds = list(dict.fromkeys(runs[k]['cond'] for k in keys))
        colors = {c: plt.cm.tab10(i % 10) for i, c in enumerate(conds)}
        for cond in conds:
            ks = [k for k in keys if runs[k]['cond'] == cond]
            nfail = sum(runs[k]['m']['diverged'] for k in ks)
            for n_k, k in enumerate(ks):
                d = runs[k]['d']
                for r_i, (_, fn, _) in enumerate(rows):
                    axs[r_i, c_i].plot(d['t'], fn(d), lw=0.9, color=colors[cond], alpha=0.85,
                                       label=(f"{cond}  墜落 {nfail}/{len(ks)}" if (n_k == 0 and r_i == 0) else None))
        axs[0, c_i].set_title(col, fontsize=11)
        for r_i, (lab, _, ylim) in enumerate(rows):
            ax = axs[r_i, c_i]
            ax.axvline(t_fault, color='k', ls=':', lw=1)
            ax.set_ylim(*ylim); ax.grid(True, alpha=0.3)
            if c_i == 0:
                ax.set_ylabel(lab, fontsize=9)
        axs[3, c_i].axhline(0.95, color='r', ls='--', lw=0.8)
        axs[-1, c_i].set_xlabel(f"時刻 [s]（点線 t={t_fault:g} s で FR 停止）")
        axs[0, c_i].legend(fontsize=7, loc='upper left')
    fig.tight_layout()
    fig.savefig(savepath, dpi=130, bbox_inches='tight')
    print(f"  [saved] {savepath}")


def plot_ftc_summary(runs: dict, savepath: str, title: str) -> None:
    """条件ごと（3 シードの平均・最小〜最大）の棒グラフ: 墜落率・故障後の姿勢誤差・高度低下・ヨー率・担当発の張り付き。"""
    groups: dict = {}
    for k, r in runs.items():
        gk = (r['col'], r['cond'])
        groups.setdefault(gk, []).append(r['m'])
    labels = [f"{c}\n{cond}" for (c, cond) in groups]
    metrics = [("墜落率 [%]", lambda m: 100.0 * float(m['diverged'])),
               ("故障後の姿勢誤差 RMS [deg]", lambda m: m['att_rms_after_deg']),
               ("故障後の姿勢誤差 最大 [deg]", lambda m: m['att_peak_after_deg']),
               ("高度の低下 [m]", lambda m: m['alt_drop']),
               ("ヨー率 r の最大 [deg/s]", lambda m: m['r_peak_after_dps']),
               ("担当 4 発が 0.95 に張り付いた割合 [%]", lambda m: 100.0 * m['carry_sat_after'])]
    fig, axs = plt.subplots(len(metrics), 1, figsize=(max(10, 1.1 * len(labels)), 2.6 * len(metrics)), sharex=True)
    x = np.arange(len(labels))
    cols = list(dict.fromkeys(c for (c, _) in groups))
    cmap = {c: plt.cm.tab10(i % 10) for i, c in enumerate(cols)}
    colors = [cmap[c] for (c, _) in groups]
    for ax, (name, fn) in zip(axs, metrics):
        vals = [[fn(m) for m in ms] for ms in groups.values()]
        mean = np.array([np.nanmean(v) for v in vals])
        lo = mean - np.array([np.nanmin(v) for v in vals])
        hi = np.array([np.nanmax(v) for v in vals]) - mean
        ax.bar(x, mean, color=colors, edgecolor='black', linewidth=0.5,
               yerr=[lo, hi], capsize=3, error_kw=dict(lw=0.8))
        for xi, v in zip(x, mean):
            ax.text(xi, v, f"{v:.3g}", ha='center', va='bottom', fontsize=7)
        ax.set_ylabel(name, fontsize=9)
        ax.grid(True, axis='y', alpha=0.3)
    axs[-1].set_xticks(x)
    axs[-1].set_xticklabels(labels, fontsize=8, rotation=0)
    fig.suptitle(title + "（棒 = 3 シード平均、ひげ = 最小〜最大）", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(savepath, dpi=130, bbox_inches='tight')
    print(f"  [saved] {savepath}")


def main_ftc(args) -> None:
    wall_t0 = time.time()
    base = Params()
    print("=" * 70)
    print(" 耐故障制御（FTC）試験: NX=5/N=14、ホバー中に FR（M1）完全停止")
    print("=" * 70)
    sanity_checks(base)
    img_dir = make_run_img_dir()
    print(f"  PNG 保存先: {img_dir}")
    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]
    forms = [f for f in FTC_FORMS if (args.ftc_form in ("all", f[0]))]
    throttles = {"pid": [False], "hold": [True], "both": [False, True]}[args.throttle]
    masses = FTC_MASSES if args.mass == 0 else [args.mass]
    runs: dict = {}
    for f_name, f_key, f_label in forms:
      for hold in throttles:
        if hold and f_key != "firmware":
            continue        # スロットル固定は実機方式だけに意味がある（元シミュ方式は高度 PID 経路のみ）
        for mass_g in masses:
            for dc_name, dc_steps, _ in DC_VARIANTS:
                for cond in FTC_CONDS:
                    for seed in seeds:
                        p = make_case_params(cond, dc_steps, seed)
                        p.mass = mass_g / 1000.0
                        p.ftc_formulation = f_key
                        p.ftc_throttle_hold = hold
                        f_tag = f_name + ("h" if hold else "")
                        key = f"{f_tag}{mass_g}_{dc_name}_{cond[0]}_s{seed}"
                        x0_hover = np.zeros(p.NX_plant)
                        x0_hover[PI_Z] = p.z_ref_default
                        ref = phi_step_schedule([(0.0, 0.0)])
                        label = (f"{key}: {f_label} {mass_g} g NX={p.NX} N={p.N} DC={dc_steps} "
                                 f"delay={p.compute_delay_cycles} hold={p.rebuild_hold_cycles} "
                                 f"bias_rw={p.sigma_gyro_bias_rw}  FR 停止 t={args.t_fault}s")
                        try:
                            logger, m = run_scenario(
                                p, FaultManager(schedule=[(args.t_fault, 0, 0.0)]), ref, fdi_known=True,
                                t_end=args.t_end, label=label,
                                savepath=str(img_dir / f"ftc_{key}.png"),
                                x0=x0_hover, z_ref_default=p.z_ref_default,
                                plan_open_loop_ascent=False)
                        finally:
                            plt.close('all')
                        runs[key] = dict(m=m, d=logger.arrays(),
                                         col=f"{f_label}・{'スロットル固定' if hold else '高度PID'} {mass_g} g",
                                         cond=f"{dc_name}_{cond[0]}")

    print("\n" + "=" * 140)
    print(f" FTC Summary（FR 停止 t={args.t_fault} s 以降の指標）")
    print("=" * 140)
    hdr = (f" {'case':<22} | {'墜落':>4} | {'t_fail':>6} | {'理由':<20} | {'高度低下m':>8} | {'姿勢峰°':>7} | "
           f"{'姿勢RMS°':>8} | {'r峰°/s':>7} | {'r平均':>7} | {'担当duty':>8} | {'担当張付%':>8}")
    print(hdr); print(" " + "-" * (len(hdr) - 1))
    for k, r in runs.items():
        m = r['m']
        print(f" {k:<22} | {'YES' if m['diverged'] else '-':>4} | {m['t_fail']:>6.2f} | {m['why'][:20]:<20} | "
              f"{m['alt_drop']:>8.2f} | {m['att_peak_after_deg']:>7.1f} | {m['att_rms_after_deg']:>8.2f} | "
              f"{m['r_peak_after_dps']:>7.1f} | {m['r_mean_after_dps']:>7.1f} | {m['u_carry_mean_after']:>8.3f} | "
              f"{m['carry_sat_after']*100:>8.1f}")
    csv_path = img_dir / "ftc_metrics.csv"
    with open(csv_path, "w") as f:
        f.write("case," + ",".join(FTC_KEYS) + "\n")
        for k, r in runs.items():
            f.write(k + "," + ",".join(str(r['m'][x]) for x in FTC_KEYS) + "\n")
    print(f"  [saved] {csv_path}")
    plot_ftc_overlay(runs, args.t_fault, str(img_dir / "ftc_overlay.png"))
    plot_ftc_summary(runs, str(img_dir / "ftc_summary.png"),
                     f"FR 停止 FTC 試験 指標まとめ（質量 {', '.join(str(m) for m in masses)} g, T_max={base.T_max}）")
    print(f"\n 全体実行時間: {time.time() - wall_t0:.1f} 秒   PNG 保存先: {img_dir}")

if __name__ == "__main__":
    main()
