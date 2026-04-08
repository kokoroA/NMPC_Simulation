import sympy as sp

# === 記号定義 ===
omega_fr = sp.Symbol('omega_fr')  # 状態
deltaT, deltaL, deltaM, deltaN = sp.symbols('deltaT deltaL deltaM deltaN')  # 入力

m, g, R, J = sp.symbols('m g R J')
D, k, Cq, Qf = sp.symbols('D k Cq Qf')

# === 非線形方程式定義 ===
numerator = (-0.25 * deltaT - 27.778 * deltaL + 27.778 * deltaM - 0.25 * 27.667 * deltaN)
input_term = numerator / (2 * m * g * R * J)

omega_dot_expr = input_term - ((D + k**2 / R) * omega_fr) / J - (Cq * omega_fr**2) / J - Qf / J

# === 状態と入力ベクトル ===
x = sp.Matrix([omega_fr])
u = sp.Matrix([deltaT, deltaL, deltaM, deltaN])

# === ヤコビアン（線形化） ===
A = sp.Matrix([omega_dot_expr]).jacobian(x)  # 状態変数による偏微分
B = sp.Matrix([omega_dot_expr]).jacobian(u)  # 入力変数による偏微分


# === 出力 ===
print("=== 状態変数 omega_fr に関する A 行列 ===")
sp.pprint(A)

print("\n=== 入力変数 [deltaT, deltaL, deltaM, deltaN] に関する B 行列 ===")
sp.pprint(B)
