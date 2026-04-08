import numpy as np

def interior_point_lp(A, b, c, tol=1e-6, max_iter=50):
    m, n = A.shape

    # 初期点（正の点からスタート）
    x = np.ones(n)
    mu = 1.0
    mu_decay = 0.5

    for k in range(max_iter):
        
        # 勾配とヘッセ行列（目的 + バリア項）
        grad_phi = c - mu / x
        hess_phi = np.diag(mu / (x ** 2))

        # KKTシステムの構築
        # [
        #   H     A^T
        #   A     0
        # ]

        KKT_mat = np.block([
            [hess_phi, A.T],
            [A, np.zeros((m, m))]
        ])

        rhs = -np.concatenate([
            grad_phi,
            A @ x - b
        ])

        # ニュートンステップ解く
        delta = np.linalg.solve(KKT_mat, rhs)
        delta_x = delta[:n]

        # ステップサイズ：x + alpha * delta_x > 0 を保証
        alpha = 1.0
        while np.any(x + alpha * delta_x <= 0):
            alpha *= 0.5

        # 更新
        x += alpha * delta_x
        mu *= mu_decay

        # 収束判定
        if np.linalg.norm(delta_x) < tol:
            print(f"converged in {k+1} iterations")
            break

    return x

A = np.array([[1, 1]])
b = np.array([1])
c = np.array([1, 1])

x_opt = interior_point_lp(A, b, c)
print("最適解:", x_opt)
print("最小値:", c @ x_opt)
