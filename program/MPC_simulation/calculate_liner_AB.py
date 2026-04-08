import sympy as sp

# === 記号定義 ===
omega, e = sp.symbols('omega e')  # 状態と入力
dt, k, R, J, D, Cq, Qf = sp.symbols('dt k R J D Cq Qf')  # 定数

# === 状態方程式 f(omega, e) ===
omega_fr_dot = (k * e_fr * dt) / (R * J) \
            - ((D + (k**2) / R) * omega_fr * dt) / J \
            - (Cq * omega_fr**2 * dt) / J \
            - (Qf * dt) / J

omega_dot = (k * e * dt) / (R * J) \
            - ((D + (k**2) / R) * omega * dt) / J \
            - (Cq * omega**2 * dt) / J \
            - (Qf * dt) / J
            
omega_dot = (k * e * dt) / (R * J) \
            - ((D + (k**2) / R) * omega * dt) / J \
            - (Cq * omega**2 * dt) / J \
            - (Qf * dt) / J

omega_dot = (k * e * dt) / (R * J) \
            - ((D + (k**2) / R) * omega * dt) / J \
            - (Cq * omega**2 * dt) / J \
            - (Qf * dt) / J
# === 変数ベクトル定義 ===
x = sp.Matrix([omega])  # 状態変数
u = sp.Matrix([e])      # 入力変数

# === f(x,u)として定義 ===
f = sp.Matrix([omega_dot])

# === ヤコビアンによる線形化 ===
A = f.jacobian(x)  # ∂f/∂x → A行列
B = f.jacobian(u)  # ∂f/∂u → B行列

# === 平衡点の定義 ===（例: omega = 0, e = 0）
equilibrium = {omega: 0, e: 0}

# === 線形化後の評価 ===
A_eval = A.subs(equilibrium)
B_eval = B.subs(equilibrium)

# === 出力 ===
print("=== A行列 (∂f/∂ω) ===")
sp.pprint(A_eval)

print("\n=== B行列 (∂f/∂e) ===")
sp.pprint(B_eval)
