# -*- coding: utf-8 -*-
"""静推力データ (重量-推力計算.xlsx / シート「ヒコロボ」) から K=1 規約の
   Ct・T_max・T(duty) を同定する。

2026-08-28 の修正点:
  1. 入力/出力パスを **リポジトリ相対** にした。
     （旧版は別セッションの絶対パス `/sessions/.../mnt/...` で、そのままでは実行不能だった）
  2. 対象モータの既定を **f1404 Kv2900**（実機のモータ）にした。
     旧版は `F1408 Kv3950` を同定しており、実機と別のモータだった。
  3. スロットル目盛り → duty の換算を **サーボテスタのパルス幅経由** に一般化した。
     旧版は `duty = 目盛り/20` を仮定していたが、ファーム側の duty は
     `1100us + 800us * duty`（`pwm.hpp` の DUTYMIN=1375 / DUTYMAX=2375 カウント）で定義される。
     ベンチのテスタが 1000-2000us を 0-20 目盛りで出すなら、両者は一致しない。
     `--map` でどちらの仮定も試せるようにし、**T_max がこの仮定にどれだけ依存するか**を示す。
  4. 質量・プロペラ・モータ・換算方式を CLI で切り替え可能にした。

使い方（NMPC_simulation の venv で実行）:
    ../../../../NMPC_simulation/.venv/bin/python sysid_thrust.py            # 既定
    python sysid_thrust.py --map ratio --mass 0.300                         # 旧版と同じ仮定
    python sysid_thrust.py --prop GEMFAN-3630-3
    python sysid_thrust.py --list                                           # 使えるモータ/プロペラ一覧
"""
import argparse, os, sys
import openpyxl, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
XLSX = os.path.join(HERE, '..', '重量-推力計算.xlsx')
OUT  = os.path.join(HERE, 'sysid_figs')

# ---- 前提（要確認パラメータ）----
G       = 9.81
NMOTOR  = 6
# スロットル目盛りの全目盛り。ベンチの目盛り 0..FULL_SCALE がテスタ出力 TESTER_MIN..MAX us に対応すると仮定。
FULL_SCALE   = 20.0
TESTER_MIN_US, TESTER_MAX_US = 1000.0, 2000.0   # 一般的なサーボテスタ。★未確認★
# ファーム側 duty の定義（pwm.hpp DUTYMIN=1375 / DUTYMAX=2375 カウント, 1 カウント = 0.8us）
FW_MIN_US,     FW_MAX_US     = 1100.0, 1900.0

ap = argparse.ArgumentParser()
ap.add_argument('--motor', default='f1404 Kv2900')
ap.add_argument('--prop',  default='betaFPV-D90s-3')
ap.add_argument('--mass',  type=float, default=0.356, help='[kg] 既定は nmpc_params.hpp の値')
ap.add_argument('--map',   choices=['pwm', 'ratio'], default='pwm',
                help="pwm: 目盛り→パルス幅→ファーム duty / ratio: duty=目盛り/FULL_SCALE (旧版の仮定)")
ap.add_argument('--sheet', default='ヒコロボ')
ap.add_argument('--list',  action='store_true', help='シート内のモータ/プロペラ組を列挙して終了')
ap.add_argument('--no-plot', action='store_true')
args = ap.parse_args()

wb = openpyxl.load_workbook(XLSX, data_only=True)
ws = wb[args.sheet]; rows = list(ws.iter_rows(values_only=True))
# idx: 2=prop, 5=motor, 6=throttle, 7=rpm, 8=weight_g, 13=omega2 (rad/s)^2, 14=thrust_N
def block(motor, prop):
    out = []; cm = cp = None
    for r in rows[1:]:
        if len(r) > 5 and r[5]: cm = r[5]
        if len(r) > 2 and r[2]: cp = r[2]
        if cm == motor and cp == prop and isinstance(r[7], (int, float)) and isinstance(r[14], (int, float)) and r[7]:
            out.append((r[6], r[7], r[8], r[13], r[14]))  # throttle, rpm, g, omega2, T_N
    return out

def catalog():
    seen = []; cm = cp = None
    for r in rows[1:]:
        if len(r) > 5 and r[5]: cm = r[5]
        if len(r) > 2 and r[2]: cp = r[2]
        if cm and cp and isinstance(r[7], (int, float)) and r[7] and (cm, cp) not in seen:
            seen.append((cm, cp))
    return seen

if args.list:
    print("=== データがある (モータ, プロペラ) の組 ===")
    for m, p in catalog():
        print("  %-16s  %s   (%d 点)" % (m, p, len(block(m, p))))
    sys.exit(0)

d = block(args.motor, args.prop)
if not d:
    print("該当データなし: motor=%r prop=%r。--list で候補を確認してください。" % (args.motor, args.prop))
    sys.exit(1)

thr = np.array([x[0] for x in d], float)
om2 = np.array([x[3] for x in d], float)   # = omega^2 [(rad/s)^2] シート計算値
T   = np.array([x[4] for x in d], float)   # thrust [N]

# --- スロットル目盛り → ファーム duty ---
if args.map == 'ratio':
    duty = thr / FULL_SCALE
    map_desc = "duty = 目盛り/%g  (旧版の仮定)" % FULL_SCALE
else:
    us   = TESTER_MIN_US + (TESTER_MAX_US - TESTER_MIN_US) * (thr / FULL_SCALE)
    duty = (us - FW_MIN_US) / (FW_MAX_US - FW_MIN_US)
    map_desc = ("目盛り→%.0f..%.0fus (テスタ) → duty=(us-%.0f)/%.0f (ファーム)"
                % (TESTER_MIN_US, TESTER_MAX_US, FW_MIN_US, FW_MAX_US - FW_MIN_US))

# 1) Ct : T = Ct * omega^2  (原点通過, 最小二乗)  ← duty 換算に依存しない
Ct    = np.sum(T * om2) / np.sum(om2 * om2)
T_ct  = Ct * om2
r2_ct = 1 - np.sum((T - T_ct) ** 2) / np.sum((T - T.mean()) ** 2)

# 2) T(duty) 2次フィット  T = a*d^2 + b*d + c
a, b, c = np.polyfit(duty, T, 2)
def Tfit(dd): return a * dd * dd + b * dd + c
T_max_full   = Tfit(1.0)          # duty=1 外挿
T_max_tested = T.max()

# 3) ホバー点
T_hover = args.mass * G / NMOTOR
roots = np.roots([a, b, c - T_hover])
roots = [x.real for x in roots if abs(x.imag) < 1e-9 and 0 < x.real < 1.2]
u_hover     = min(roots) if roots else float('nan')
omega_hover = (T_hover / Ct) ** 0.5
slope_hover = 2 * a * u_hover + b   # dT/du at hover → 線形 B_c に入れる実効 T_max

print("=== CONFIG: %s + %s ===" % (args.motor, args.prop))
print("換算: %s" % map_desc)
print("目盛り %g..%g  →  duty %.3f..%.3f   (data points: %d)"
      % (thr.min(), thr.max(), duty.min(), duty.max(), len(d)))
print("機体質量 %.3f kg / %d 発  →  ホバー推力 %.4f N/モータ" % (args.mass, NMOTOR, T_hover))
print("-" * 68)
print("Ct  = %.3e  N*s^2/rad^2  (T=Ct*omega^2, R^2=%.4f)   ※換算に非依存" % (Ct, r2_ct))
print("T(duty) quad: a=%.4f b=%.4f c=%.4f" % (a, b, c))
print("T_max (duty=1 外挿)      = %.3f N   [実測最大 %.3f N @duty=%.3f]"
      % (T_max_full, T_max_tested, duty.max()))
print("ホバー duty              = %.3f   (omega=%.0f rad/s = %.0f rpm)"
      % (u_hover, omega_hover, omega_hover * 60 / (2 * np.pi)))
print("dT/du at hover           = %.3f N/unit-duty   ★線形 B_c に入れる実効 T_max★" % slope_hover)
print("推力マージン T_max/T_hover = %.2f  (6発合計 %.0f g vs 機体 %.0f g)"
      % (T_max_full / T_hover, T_max_full * NMOTOR / G * 1000, args.mass * 1000))

if args.no_plot:
    sys.exit(0)

os.makedirs(OUT, exist_ok=True)
fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
xs = np.linspace(0, om2.max() * 1.05, 100)
ax[0].scatter(om2 / 1e6, T, c='#1f6feb', s=42, zorder=3, label='measured')
ax[0].plot(xs / 1e6, Ct * xs, '-', c='#d1242f', lw=2, label=f'T=Ct·ω²,  Ct={Ct:.2e}\nR²={r2_ct:.4f}')
ax[0].set_xlabel('ω²  [×10⁶ (rad/s)²]'); ax[0].set_ylabel('Thrust  T [N]')
ax[0].set_title('(a) Thrust coefficient  Ct'); ax[0].grid(alpha=.3); ax[0].legend()
dd = np.linspace(0, 1.02, 100)
ax[1].scatter(duty, T, c='#1f6feb', s=42, zorder=3, label='measured')
ax[1].plot(dd, Tfit(dd), '-', c='#2da44e', lw=2, label='quadratic fit')
ax[1].axhline(T_hover, ls=':', c='#888', lw=1.2)
ax[1].scatter([u_hover], [T_hover], c='#d1242f', s=70, zorder=4, label=f'hover (u≈{u_hover:.3f})')
ax[1].plot(dd, T_hover + slope_hover * (dd - u_hover), '--', c='#d1242f', lw=1.4,
           label=f'linear @hover (slope={slope_hover:.2f})')
ax[1].scatter([1.0], [T_max_full], marker='*', c='k', s=120, zorder=4, label=f'T_max≈{T_max_full:.2f} N')
ax[1].set_xlabel('firmware duty  u'); ax[1].set_ylabel('Thrust  T [N]')
ax[1].set_title('(b) Static thrust map  T(duty)'); ax[1].grid(alpha=.3); ax[1].legend(fontsize=8)
fig.suptitle(f'Static thrust system-ID: {args.motor} + {args.prop}  [map={args.map}, M={args.mass} kg]', fontsize=12)
fig.tight_layout()
png = os.path.join(OUT, 'thrust_sysid_%s_%s.png' % (args.motor.replace(' ', '_'), args.map))
fig.savefig(png, dpi=150)
print("saved plot: %s" % os.path.relpath(png, HERE))
