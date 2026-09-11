# -*- coding: utf-8 -*-
"""2点吊り Iz 実験ログ(Iz_exp{,2,3}.csv)から ねじり振動の周期を推定。
   信号: ヨー角速度 r [rad/s] (gyro z)。実時間10Hz記録(0.1s/行)。"""
import numpy as np, pandas as pd
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

LOG="/sessions/dreamy-cool-sagan/mnt/research_project/program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/tools/log"
FILES=["Iz_exp.csv","Iz_exp2.csv","Iz_exp3.csv"]
OUT="/sessions/dreamy-cool-sagan/mnt/outputs/iz"
DT=0.1  # 10Hz

def load(f):
    df=pd.read_csv(f"{LOG}/{f}",comment='#')
    df=df[np.isfinite(df['r'])].reset_index(drop=True)
    n=len(df); return np.arange(n)*DT, df['r'].values.astype(float)

def fit_band(t,r,fmin=0.5,fmax=3.0):
    """物理帯域(0.5-3Hz)で w,z 格子探索 + a,b,c 線形LSQ。"""
    r=r-r.mean(); best=None
    for fw in np.linspace(fmin,fmax,501):
        w=2*np.pi*fw
        for z in np.linspace(0.0,0.4,41):
            e=np.exp(-z*t)
            M=np.column_stack([e*np.sin(w*t),e*np.cos(w*t),np.ones_like(t)])
            coef,_,_,_=np.linalg.lstsq(M,r,rcond=None)
            sse=np.sum((M@coef-r)**2)
            if best is None or sse<best[0]: best=(sse,w,z,coef)
    sse,w,z,coef=best
    R2=1-sse/np.sum((r-r.mean())**2)
    return 2*np.pi/w,w,z,coef,R2

def strongest_window(t,r,width=8.0):
    r0=r-r.mean(); win=int(2/DT)
    env=pd.Series(np.abs(r0)).rolling(win,center=True).max().values
    ic=np.nanargmax(env); tc=t[ic]
    return max(0,tc-width/2), min(t[-1],tc+width/2)

Ts=[]; rows=[]
fig,axs=plt.subplots(3,1,figsize=(9,8))
for ax,f in zip(axs,FILES):
    t,r=load(f); T,w,z,coef,R2=fit_band(t,r)
    Ts.append(T); rows.append((f,T,R2,z))
    t0,t1=strongest_window(t,r)
    m=(t>=t0)&(t<=t1)
    ax.plot(t[m],r[m],'o-',ms=4,lw=0.7,color='#1f6feb',label='r (yaw rate) measured')
    tt=np.linspace(t0,t1,1000); e=np.exp(-z*tt)
    ax.plot(tt,coef[0]*e*np.sin(w*tt)+coef[1]*e*np.cos(w*tt)+coef[2],'-',color='#d1242f',lw=1.6,
            label=f'fit  T={T:.3f}s ({1/T:.2f}Hz), R²={R2:.2f}')
    ax.set_title(f"{f}  (拡大: 運動最大の{t1-t0:.0f}s区間)",fontsize=10)
    ax.set_xlabel('time [s]'); ax.set_ylabel('r [rad/s]'); ax.grid(alpha=.3); ax.legend(fontsize=8,loc='upper right')
fig.suptitle('Bifilar Iz test — torsional oscillation period (zoom on strongest motion)',fontsize=12)
fig.tight_layout(); fig.savefig(f"{OUT}/Iz_periods.png",dpi=150)

Ts=np.array(Ts)
print("file            T[s]    freq[Hz]  R2")
for f,T,R2,z in rows: print(f"{f:14s}  {T:.3f}   {1/T:.3f}    {R2:.3f}")
print(f"\n周期 3回: {np.round(Ts,3)} s  →  mean={Ts.mean():.3f}s  std={Ts.std(ddof=1):.3f}s  (CV {Ts.std(ddof=1)/Ts.mean()*100:.1f}%)")

print("\n--- Iz = m g D^2 T^2 /(16 pi^2 L) = C(m,L,D) * T^2 ---")
g=9.81; Tm=Ts.mean()
print("   L,D,m を実測で入れると確定。参考の感度:")
for m,L,D in [(0.356,0.10,0.30),(0.356,0.15,0.30),(0.356,0.20,0.40)]:
    C=m*g*D*D/(16*np.pi**2*L); 
    print(f"   m={m} L={L:.2f}m D={D:.2f}m → C={C:.3e}, Iz(meanT)={C*Tm*Tm*1e3:.3f}e-3 kg m^2")
