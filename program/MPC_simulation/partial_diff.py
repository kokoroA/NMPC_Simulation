import sympy as sp

# 記号の定義
x, y, z = sp.symbols('x y z')

# 任意の関数の定義（例）
f = sp.sin(x**2) * sp.exp(y) + x * y * z

# 偏微分の実行
df_dx = sp.diff(f, x)  # xについて偏微分
df_dy = sp.diff(f, y)  # yについて偏微分
df_dz = sp.diff(f, z)  # zについて偏微分

# 結果の表示
print("f(x, y, z) =")
sp.pprint(f)

print("\nPartial derivative with respect to x:")
sp.pprint(df_dx)

print("\nPartial derivative with respect to y:")
sp.pprint(df_dy)

print("\nPartial derivative with respect to z:")
sp.pprint(df_dz)
