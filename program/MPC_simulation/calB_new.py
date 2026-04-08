import numpy as np

#---------パラメータ設定---------
g = 9.81                                    # 重力加速度(m/s)
m = 0.0368                                   # 機体質量(kg)
l = 0.044                                 #モーター間の距離(m)
K = 6.15e-4 #3.28e-3                        #トルク定数
Ct = 1e-8                               #推力係数
CQ = 9.17e-11#4.53e-14#3.0e-8                 #トルク係数
D = 0#15.01e-6#0                            #動粘性抵抗係数                             #摩擦トルク
Reg = 0.34                                  #巻線抵抗値
Ix, Iy, Iz = 4.4e-4, 4.4e-4, 4e-4       # 慣性モーメント                          #釣り合いの推力 m*g                       
omega_zero = np.sqrt(m*g/Ct)/2        #釣り合いの角速度
print("omega_zero:", omega_zero)

# numerator = l * Ct * K * omega_zero * 111.112 * (K**2 + CQ * omega_zero * Reg)
numerator = l * Ct * K * omega_zero * 45.44 * (K**2 + CQ * omega_zero * Reg)
denominator = (D * Reg + K**2)**2 * Ix
Bp = numerator / denominator

numerator = l * Ct * K * omega_zero * 45.44 * (K**2 + CQ * omega_zero * Reg)
denominator = (D * Reg + K**2)**2 * Iy
Bq = numerator / denominator

numerator = 2 * CQ * K * omega_zero * 109 * (K**2 + CQ * omega_zero * Reg)
denominator = (D * Reg+ K**2)**2 * Iz
Br = numerator / denominator

print(Bp, Bq, Br)