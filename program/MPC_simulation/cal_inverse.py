import numpy as np

l = 0.044                                 #モーター間の距離(m)
# Ct = 8.3e-7                                 #推力係数
# CQ = 3.0e-8#4.53e-14#3.0e-8                 #トルク係数
Ct = 1e-8                               #推力係数
CQ = 9.17e-11#4.53e-14#3.0e-8                 #トルク係数
kapa = CQ/Ct
x1, x2, x3, x4 = 0.5*l, 0.5*l, 0.5*l, 0.5*l #重心からモーターの距離
y1, y2, y3, y4 = 0.5*l, 0.5*l, 0.5*l, 0.5*l #重心からモーターの距離

B = np.array([
                [-1, -1, -1, -1],
                [-x1, -x2, x3, x4],
                [y1, -y2, -y3, y4],
                [1, -1, 1, -1]])

C = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,0,],
                [0,0,0,kapa]])

# === 逆行列を計算 ===
try:
    B_inv = np.linalg.inv(B)
    print("元の行列 B:")
    print(B)
    print("\n逆行列 B^-1:")
    print(B_inv)

    # === 検算（A * A^-1 = 単位行列になるか） ===
    identity_check = np.dot(B, B_inv)
    print("\nB * B^-1（検算結果）:")
    print(identity_check)

    C_inv = np.linalg.inv(C)
    print("元の行列 C:")
    print(C)
    print("\n逆行列 C^-1:")
    print(C_inv)

    # === 検算（A * A^-1 = 単位行列になるか） ===
    identity_check = np.dot(C, C_inv)
    print("\nC * C^-1（検算結果）:")
    print(identity_check)

except np.linalg.LinAlgError:
    print("❌ この行列は逆行列を持ちません（特異行列：行列式が0）")
