"""du = u_i - u_base の周波数内訳（docs/figs/20261006_du_spectrum_v8.svg の数値を出す）。
使い方: python3 du_spectrum.py  （Benzen_NMPC/tools で実行。numpy のみ）
決定記録: docs/ai/decisions/20261006_du_above_5hz_85pct_not_reproducible_69pct.md"""
import numpy as np

MOT=['u_fr','u_fl','u_mr','u_ml','u_rr','u_rl']

def load(path):
    cols=None; rows=[]
    for line in open(path, errors='replace'):
        line=line.strip()
        if line.startswith('#cols:'):
            cols=[s.strip() for s in line[len('#cols:'):].split(',')]; continue
        if not line or line.startswith('#'): continue
        p=line.split(',')
        if cols is None or len(p)!=len(cols): continue
        try: rows.append([float(x) for x in p])
        except ValueError: continue
    D=np.array(rows)
    c={n:i for i,n in enumerate(cols)} if cols else {}
    return c,D
import numpy as np, sys, math

FS=100.0
BASE='0919_%s.txt'

def welch(x,nperseg=256,fs=FS):
    x=np.asarray(x,float); n=len(x); nperseg=min(nperseg,n); step=max(nperseg//2,1)
    w=np.hanning(nperseg+1)[:nperseg]; U=(w**2).sum(); segs=[]
    for s in range(0,n-nperseg+1,step):
        seg=x[s:s+nperseg]-x[s:s+nperseg].mean()
        X=np.fft.rfft(seg*w); P=(np.abs(X)**2)/(fs*U); P[1:-1]*=2.0; segs.append(P)
    return np.fft.rfftfreq(nperseg,1/fs), np.mean(segs,axis=0)

def lp(x,fc,fs=FS):
    tau=1.0/(2*math.pi*fc); a=(1/fs)/(tau+1/fs)
    y=np.empty_like(x); acc=x[0]
    for i,v in enumerate(x): acc+=a*(v-acc); y[i]=acc
    z=np.empty_like(y); acc=y[-1]
    for i in range(len(y)-1,-1,-1): acc+=a*(y[i]-acc); z[i]=acc
    return z

def seg(v,t0,t1):
    c,D=load(BASE%v); t=D[:,c['t']]; m=(t>=t0)&(t<t1)
    U=D[np.ix_(m,[c[k] for k in MOT])]; ub=D[m,c['u_base']]
    return t[m], U-ub[:,None], ub

def psd6(du):
    acc=None
    for i in range(6):
        f,P=welch(du[:,i]); acc=P.copy() if acc is None else acc+P
    return f, acc/6

def tailfrac(f,P):
    # fraction of total power at frequencies >= f[i]
    df=f[1]-f[0]
    w=np.full_like(P,df); w[0]*=0.5; w[-1]*=0.5
    e=P*w; tot=e.sum()
    return np.cumsum(e[::-1])[::-1]/tot

t8,du8,ub8 = seg('v8',5.0,10.3)
t6,du6,ub6 = seg('v6',4.0,14.0)
f8,P8 = psd6(du8); f6,P6 = psd6(du6)
T8=tailfrac(f8,P8); T6=tailfrac(f6,P6)

def at(f,T,fc): 
    i=int(np.argmin(np.abs(f-fc))); return T[i]
print('v8 tail:', {fc:round(at(f8,T8,fc)*100,1) for fc in (1,2,3,4,5,10,20)})
print('v6 tail:', {fc:round(at(f6,T6,fc)*100,1) for fc in (1,2,3,4,5,10,20)})
fr=du8[:,0]; fr_lp=lp(fr,4.0)
print('du_fr: rms=%.4f  rms(<4Hz)=%.4f  max|.|=%.4f  max|lp|=%.4f'%(fr.std(),fr_lp.std(),np.abs(fr).max(),np.abs(fr_lp).max()))
