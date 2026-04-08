import numpy as np
from numpy.linalg import matrix_rank

def is_controllable(A, B):
    """
    可制御性をチェックする
    A: nxn行列
    B: nxm行列
    """
    n = A.shape[0]
    controllability_matrix = B
    for i in range(1, n):
        controllability_matrix = np.hstack((controllability_matrix, np.linalg.matrix_power(A, i) @ B))

    rank = matrix_rank(controllability_matrix)
    print("=== 可制御性チェック ===")
    print("Controllability matrix:\n", controllability_matrix)
    print("Rank:", rank)
    if rank == n:
        print("✅ システムは可制御です。\n")
        return True
    else:
        print("❌ システムは可制御ではありません。\n")
        return False

def is_observable(A, C):
    """
    可観測性をチェックする
    A: nxn行列
    C: pxn行列
    """
    n = A.shape[0]
    observability_matrix = C
    for i in range(1, n):
        observability_matrix = np.vstack((observability_matrix, C @ np.linalg.matrix_power(A, i)))

    rank = matrix_rank(observability_matrix)
    print("=== 可観測性チェック ===")
    print("Observability matrix:\n", observability_matrix)
    print("Rank:", rank)
    if rank == n:
        print("✅ システムは可観測です。\n")
        return True
    else:
        print("❌ システムは可観測ではありません。\n")
        return False

# === 行列定義（例）===
A = np.array([
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
], dtype=float)

B = np.array([
    [0, 0, 0],
    [0, 0, 0],
    [1/0.0061, 0, 0],
    [0, 1/0.00653, 0],
    [0, 0, 1/0.0116]
], dtype=float)

# 例として C = 単位行列（全状態を観測できる場合）
C = np.eye(5)

# 実行
is_controllable(A, B)
is_observable(A, C)
