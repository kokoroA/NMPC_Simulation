"""plant6dof の単体試験（python3 test_plant6dof.py）。"""
import math
import numpy as np
from plant6dof import (Plant6DOF, Plant6Params, preset, rot_body_to_ned, rot_body_to_ned_euler,
                       quat_from_euler, euler_from_quat, thrust_curve, IV, IW)
from plant import AttitudePlant, PlantParams

ok = True
def check(name, cond, info=''):
    global ok
    print(('PASS ' if cond else 'FAIL ') + name + (f'  [{info}]' if str(info) else ''))
    ok &= bool(cond)

rng = np.random.default_rng(0)
# 1. オイラー表現とクォータニオン表現の一致（スライドの 2 式）
err = 0.0
for _ in range(200):
    a = rng.uniform(-1.2, 1.2, 3); a[2] = rng.uniform(-3, 3)
    q = quat_from_euler(*a)
    err = max(err, np.abs(rot_body_to_ned(q) - rot_body_to_ned_euler(*a)).max())
    err = max(err, np.abs(np.array(euler_from_quat(q)) - a).max())
check('オイラー/クォータニオン変換の一致', err < 1e-12, f'max err {err:.1e}')

# 2. ホバー平衡（全部入り、無風）: 1 s 後も高度・姿勢がほぼ動かない
for name in ('legacy', 'rigid', 'full'):
    pp = preset(name)
    pl = Plant6DOF(pp)
    d = pp.hover_duty
    if name == 'full':   # 地面効果なしの高さ、軸流は 0 速度で効かない
        pass
    pl.set_airborne(2.0, d)
    for _ in range(400): pl.step(np.full(6, d))
    check(f'ホバー平衡 [{name}]', abs(pl.altitude - 2.0) < 2e-3 and abs(pl.phi) < 1e-9 and np.abs(pl.rates).max() < 1e-9,
          f'Δh={pl.altitude - 2.0:+.2e} m')
    f = pl.specific_force_body()
    check(f'ホバー比力 = −g ẑ [{name}]', np.allclose(f, [0, 0, -pp.gravity], atol=2e-3), np.round(f, 4))   # full は 2 m でも地面効果 +3e-5

# 3. 自由落下: 比力 0、1 s で 4.905 m
pp = preset('rigid'); pl = Plant6DOF(pp); pl.set_airborne(10.0, 0.0)
for _ in range(400): pl.step(np.zeros(6))
check('自由落下 1 s', abs((10.0 - pl.altitude) - 0.5 * pp.gravity) < 1e-3, f'{10 - pl.altitude:.4f} m')
check('自由落下の比力 = 0', np.abs(pl.specific_force_body()).max() < 1e-9)

# 4. 抗力: 水平 5 m/s・水平姿勢での減速度 = (C u² + k T u)/m
pp = preset('full', k_bf_roll=0.0, k_bf_pitch=0.0, ge_enable=False, v_pitch_speed=0.0)
pl = Plant6DOF(pp); d = pp.hover_duty; pl.set_airborne(5.0, d, (5.0, 0.0, 0.0))
pl.step(np.full(6, d))
Tsum = pp.mass * pp.gravity
expect = (pp.C_drag[0] * 25 + pp.k_rotor_drag * Tsum * 5) / pp.mass
got = (5.0 - pl.vel_body[0]) / pl.dt
check('抗力による減速', abs(got - expect) / expect < 0.02, f'{got:.3f} vs {expect:.3f} m/s²')
check('抗力は比力に現れる（加速度計 x）', pl.specific_force_body()[0] < -0.5 * expect, f'{pl.specific_force_body()[0]:.3f}')

# 5. ブローバックの向き: 右へ流れる（v>0）と左へ戻す（ṗ<0）、前へ流れる（u>0）と機首上げ（q̇>0）
pp = preset('full', C_drag=(0, 0, 0), k_rotor_drag=0.0, ge_enable=False, v_pitch_speed=0.0)
pl = Plant6DOF(pp); d = pp.hover_duty; pl.set_airborne(5.0, d, (0.0, 2.0, 0.0)); pl.step(np.full(6, d))
check('ブローバック: 右流れ → 左ロール', pl.rates[0] < 0, f'p={pl.rates[0]:.4f}')
pl = Plant6DOF(pp); pl.set_airborne(5.0, d, (2.0, 0.0, 0.0)); pl.step(np.full(6, d))
check('ブローバック: 前流れ → 機首上げ', pl.rates[1] > 0, f'q={pl.rates[1]:.4f}')

# 6. 旧プラント互換（legacy プリセット・duty 遅れ）: 同じ差動入力で姿勢が一致
pp6 = preset('legacy', translation_coupled=False, n_substeps=1)
old = AttitudePlant(PlantParams(Ix=pp6.Ix, Iy=pp6.Iy, Iz=pp6.Iz, Cp=pp6.Cp, Cq=pp6.Cq, Cr=pp6.Cr, mass=pp6.mass,
                                C_QT=pp6.C_QT, motor_tau=0.05))
old.p.thrust_of = lambda dd: thrust_curve(dd)
new = Plant6DOF(pp6)
old.on_ground = False; new.on_ground = False; new.s[2] = -5.0
hd = pp6.hover_duty; old.duty_actual[:] = hd; new.s[13:19] = hd
u = np.array([hd - 0.03, hd + 0.03, hd - 0.03, hd + 0.03, hd - 0.02, hd + 0.04])
emax = 0.0
for k in range(200):
    old.step(u); new.step(u)
    emax = max(emax, abs(old.phi - new.phi), abs(old.theta - new.theta), np.abs(old.rates - new.rates).max())
check('旧 AttitudePlant との一致（0.5 s の差動入力）', emax < 5e-3, f'max diff {emax:.1e}（離散化の差）')

# 7. 回転数モデル: 小信号の応答時定数が τ（63 % 到達）
pp = preset('full', ge_enable=False, v_pitch_speed=0.0, C_drag=(0, 0, 0), k_rotor_drag=0.0, k_bf_roll=0, k_bf_pitch=0)
pl = Plant6DOF(pp); pl.set_airborne(5.0, d)
T0 = pp.mass * pp.gravity
target = float(thrust_curve(d + 0.05)) * 6
ts = None
for k in range(200):
    pl.step(np.full(6, d + 0.05))
    if ts is None and pl.aux['T'] >= T0 + 0.632 * (target - T0): ts = (k + 1) * pl.dt
check('回転数一次遅れ: 推力 63 % 到達 ≈ τ/2〜τ', ts is not None and 0.02 <= ts <= 0.06, f'{ts} s')

# 8. ロータ加速の反トルク: 対角でない組を上げるとヨーが動く（J_r>0）
pl = Plant6DOF(pp); pl.set_airborne(5.0, d)
du = np.full(6, d); du[[1, 2, 5]] += 0.05; du[[0, 3, 4]] -= 0.05     # kS=+1 の発を上げ、−1 の発を下げる（和は一定）
pl.step(du)
check('ヨー: kS=+1 群を上げると r>0', pl.rates[2] > 0, f'r={pl.rates[2]:.4f}')

print('\n全体:', 'OK' if ok else 'NG')
