import numpy as np
from scipy.linalg import expm

def discretize_matrix_exponential(A, B, dt):
    n = A.shape[0]
    m = B.shape[1]

    # 拡張行列Mの構築
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B

    # 行列指数関数を計算
    M_d = expm(M * dt)

    # A_dとB_dを抽出
    A_d = M_d[:n, :n]
    B_d = M_d[:n, n:]

    return A_d, B_d

# モデルパラメータ
# Ix, Iy, Iz = 6.1e-3, 6.53e-3, 1.16e-2       # 慣性モーメント
Ix,Iy,Iz = 4.4e-4,4.4e-4,4e-4
# Ix,Iy,Iz = 2.14e-5,2.19e-5,4.08e-5
# 状態行列 A（6x6）
# A = np.array([
#     [0, 0, 0, 1, 0, 0],
#     [0, 0, 0, 0, 1, 0],
#     [0, 0, 0, 0, 0, 1],
#     [0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0]
# ])

A = np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ], dtype=float)

# 入力行列 B（6x3）
B = np.array([
    [0, 0, 0],
    [0, 0, 0],
    [1/Ix, 0, 0],
    [0, 1/Iy, 0],
    [0, 0, 1/Iz]
])
dt = 0.01  # サンプリング周期
# dt = 0.0025  # サンプリング周期
A_d, B_d = discretize_matrix_exponential(A, B, dt)

# 確認
print(1/Ix)
print(1/Iy)
print(1/Iz)
print("A_d =\n", A_d)
print("B_d =\n", B_d)

# A_d =
#  [[1.   0.   0.   0.01 0.   0.  ]
#  [0.   1.   0.   0.   0.01 0.  ]
#  [0.   0.   1.   0.   0.   0.01]
#  [0.   0.   0.   1.   0.   0.  ]
#  [0.   0.   0.   0.   1.   0.  ]
#  [0.   0.   0.   0.   0.   1.  ]]
# B_d =
#  [[0.00819672 0.         0.        ]
#  [0.         0.00765697 0.        ]
#  [0.         0.         0.00431034]
#  [1.63934426 0.         0.        ]
#  [0.         1.53139357 0.        ]
#  [0.         0.         0.86206897]]