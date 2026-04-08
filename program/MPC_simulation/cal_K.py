import numpy as np
from scipy.linalg import solve_discrete_are
from numpy.linalg import matrix_rank
from discritization_euler import discretize_euler

# 状態空間次元
n = 5  # 状態数
# n = 6  # 状態数
# n = 2  # 状態数
m = 3  # 入力数

A = np.array([
    [0, 0, 1, 0, 0], # phi  
    [0, 0, 0, 1, 0], # theta
    [0, 0, 0, 0, 0], # p
    [0, 0, 0, 0, 0], # q
    [0, 0, 0, 0, 0] # r
])

B = np.array([
    [0, 0, 0],
    [0, 0, 0],
    [382.28, 0, 0],
    [0, 382.28, 0],
    [0, 0, 87.29]
], dtype=np.float32)

Ab,Bb = discretize_euler(A, B, 0.01)  # 離散化のためのサンプリング周期を指定
# print(Ab)
# print(Bb)
# B = np.array([
#     [0, 0, 0],
#     [276.9/100, 0, 0],
#     [0, 0, 0],
#     [0, 276.9/100, 0],
#     [0, 0, 0],
#     [0, 0, 304.6/100]
# ], dtype=np.float32)

# 重み行列 Q (5x5)
Q = np.eye(n, dtype=np.float32)
Q[0, 0] = 1/0.3**2 #phi
Q[1, 1] = 1/0.3**2 #theta
Q[2, 2] = 1/3.5**2 #p
Q[3, 3] = 1/3.5**2 # p
Q[4, 4] = 1/0.1**2 # r
# Q[0, 0] = 150 #phi_dot
# Q[1, 1] = 150#theta_dot
# Q[2, 2] = 150#p_dot
# Q[3, 3] = 150 # p_dot
# Q[4, 4] = 150 # q_dot

# 重み行列 R (3x3)
R = np.eye(m, dtype=np.float32)
R[0, 0] = 1.0  # Roll rate
R[1, 1] = 1.0  # Pitch rate
R[2, 2] = 0.1 # Yaw rate

# 離散時間リカッチ方程式を解く
P = solve_discrete_are(Ab, Bb, Q, R)
# print("P")
# print(P)
# 最適フィードバックゲイン K
K = np.linalg.inv(Bb.T @ P @ Bb + R) @ (Bb.T @ P @ Ab)
# K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A) 

print("Gain K:")
print(K)
ctrl_matrix = np.hstack([Bb, Ab @ Bb, Ab @ Ab @ Bb, Ab @ Ab @ Ab @ Bb, Ab @ Ab @ Ab @ Ab @ Bb])
print("可制御ランク:", matrix_rank(ctrl_matrix))  # → 5ならOK