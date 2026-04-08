import numpy as np
l = 0.022
Ct = 1e-8
K = 6.15e-4
m = 0.045
g = 9.81
omega_zero = np.sqrt(m*g/Ct)/2
print("omega_zero:", omega_zero)
CQ = 9.17e-11
Reg = 0.34
D= 0
# D= 1e-8
Ix,Iy,Iz = 4.4e-4,4.4e-4,4e-4

numerator = l * Ct * K * omega_zero * 111.112 * (K**2 + CQ * omega_zero * Reg)
denominator = (D * Reg + K**2)**2 * Ix
Bp = numerator / denominator

numerator = l * Ct * K * omega_zero * 111.112 * (K**2 + CQ * omega_zero * Reg)
denominator = (D * Reg + K**2)**2 * Iy
Bq = numerator / denominator

numerator = 2 * CQ * K * omega_zero * 27.667 * (K**2 + CQ * omega_zero * Reg)
denominator = (D * Reg+ K**2)**2 * Iz
Br = numerator / denominator

print(Bp, Bq, Br)