import numpy as np
from scipy.linalg import solve_discrete_are
from discritization_euler import discretize_euler

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

C = np.array([
    [1,0,0,0,0],
    [0,1,0,0,0],
    [0,0,0,0,1]
])
Ts = 0.01
Ad,Bd = discretize_euler(A, B, Ts)  # 離散化のためのサンプリング周期を指定

# Ad, Bd, C, Ts を用意
p = C.shape[0]
Aa = np.block([[Ad, np.zeros((Ad.shape[0], p))],
               [-C*Ts, np.eye(p)]])
Ba = np.vstack((Bd, np.zeros((p, Bd.shape[1]))))

Qx   = np.diag([1/0.3**2, 1/0.3**2, 1/3.5**2, 1/3.5**2, 1/0.1**2])
Qeta = np.diag([1/0.3**2, 1/0.3**2, 1/0.1**2])
Qa   = np.block([
    [Qx,                 np.zeros((Qx.shape[0], Qeta.shape[1]))],
    [np.zeros((Qeta.shape[0], Qx.shape[1])), Qeta]
])
R = np.diag([1.7,1.7, 10])

P = solve_discrete_are(Aa, Ba, Qa, R)
K = np.linalg.solve(R + Ba.T@P@Ba, Ba.T@P@Aa)   # (R+B'PB)^{-1}B'PA
Kx, Ki = K[:, :5], K[:, 5:]
print("Kx:", Kx)
print("Ki:", Ki)
