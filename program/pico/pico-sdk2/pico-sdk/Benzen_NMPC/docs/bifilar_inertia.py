# -*- coding: utf-8 -*-
"""二線式(2点吊り)振り子による慣性モーメント測定 — 設計&換算ツール
   I = m g D^2 T^2 / (16 pi^2 L)      (D=ワイヤ間隔, L=ワイヤ長, T=周期)
   等価: T = 2 pi sqrt( I L / (m g (D/2)^2) )
"""
import numpy as np
g=9.81

def period_from_I(I, m, L, D):
    b=D/2.0
    return 2*np.pi*np.sqrt(I*L/(m*g*b*b))
def I_from_period(T, m, L, D):
    return m*g*D*D*T*T/(16*np.pi**2*L)

# --- 機体 (nmpc_params.hpp 現行推定) ---
m=0.356
I_est={'Ix (roll)':6.10e-3,'Iy (pitch)':6.53e-3,'Iz (yaw)':1.16e-2}

print("=== 設計: 各軸で期待される振動周期 (この機体の推定 I から) ===")
print(f"m={m} kg")
for L in [0.8,1.0,1.2]:
    for D in [0.15,0.20,0.25]:
        ts=[period_from_I(I,m,L,D) for I in I_est.values()]
        print(f" L={L:.1f}m D={D:.2f}m -> T[s]: "+
              ", ".join(f"{k.split()[0]}={t:.2f}" for k,t in zip(I_est,ts)))
    print()

print("=== 換算例: L=1.0m, D=0.20m で周期を測ったら ===")
L,D=1.0,0.20
for T in [2.0,2.6,3.0,3.6]:
    print(f" T={T:.2f}s -> I={I_from_period(T,m,L,D)*1e3:.3f} e-3 kg m^2")

# 感度 (誤差伝播): dI/I = 2 dT/T + 2 dD/D - dL/L + dm/m
print("\n=== 相対感度 (I ∝ m·D^2·T^2 / L) ===")
print(" dI/I = dm/m + 2·dD/D + 2·dT/T − dL/L")
print(" → D と T の測定が最も効く(2乗)。T は多周期計測で誤差低減。")
