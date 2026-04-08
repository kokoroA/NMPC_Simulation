import numpy.linalg as LA
import numpy as np
def solve_are(A, B, Q, R):
    # 1. ハミルトンマトリクスを置く
    H = np.block([[A.T, -B @ LA.inv(R) @ B.T],
                  [-Q , -A]])
    # 2.固有値分解する
    eigenvalue, w = LA.eig(H)
    # 3.補助行列を置く
    Y_, Z_ = [], []
    n = len(w[0])//2
    # 固有値をsortしておく
    index_array = sorted([i for i in range(2*n)],
        key = lambda x:eigenvalue[x].real)
    # 実部が小さいものからn個を採用する
    for i in index_array[:n]:
        Y_.append(w.T[i][:n])
        Z_.append(w.T[i][n:])
    Y = np.array(Y_).T
    Z = np.array(Z_).T
    # 4.Pが求まる
    if LA.det(Y) != 0:
        return Z @ LA.inv(Y)
    else:
        print("Warning: Y is not regular matrix. Result may be wrong!")
        return Z @ LA.pinv(Y)

# 状態空間次元
n = 6  # 状態数
m = 3  # 入力数

A = np.array([
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0]
])

# 入力行列 B (5x3)
B = np.array([
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [276.9/100, 0, 0],
    [0, 276.9/100, 0],
    [0, 0, 304.6/100]
], dtype=np.float32)

# 重み行列 Q (5x5)
Q = np.eye(n, dtype=np.float32)
Q[0, 0] = 1.0 #phi_dot
Q[1, 1] = 1.0 #theta_dot
Q[2, 2] = 10.0 #psi_dot
Q[3, 3] = 10.0 # p_dot
Q[4, 4] = 10.0 # q_dot
Q[5, 5] = 10.0 # r_dot

# 重み行列 R (3x3)
R = np.eye(m, dtype=np.float32)
R[0, 0] = 1.0  # Roll rate
R[1, 1] = 1.0  # Pitch rate
R[2, 2] = 1.0 # Yaw rate

P = solve_are(A, B, Q, R)

# 最適フィードバックゲイン K
K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A) * 100

print("Gain K:")
print(K)
print("P")
print(P)
print("Riccati代数方程式の左辺")
print(A@P + P@A.T + Q - P@B@LA.inv(R)@B.T@P)