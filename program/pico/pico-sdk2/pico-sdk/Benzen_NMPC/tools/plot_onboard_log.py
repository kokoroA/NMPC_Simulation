# -*- coding: utf-8 -*-
"""機内 100Hz ログ（log_output() の USB ダンプ）を読み込んで PNG を出す。

使い方:
    python3 tools/plot_onboard_log.py log/onboard_0904.txt

シリアルモニタの出力をそのまま保存したファイルでよい（`#` 行は読み飛ばす）。
列は control.cpp の log_output() が出す `#cols:` 行と同じ:
    t,phi,theta,p,q,r,phi_ref,theta_ref,psi_ref,T_ref,u_base,
    u_fr,u_fl,u_mr,u_ml,u_rr,u_rl,iter_us,loop_us,cycles,failsafe,ftc,acc_norm
"""
import sys, os, csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for _f in ('Hiragino Sans', 'AppleGothic', 'Noto Sans CJK JP'):
    if any(_f == x.name for x in fm.fontManager.ttflist):
        matplotlib.rcParams['font.family'] = _f
        break

COLS = ("t,phi,theta,p,q,r,phi_ref,theta_ref,psi_ref,T_ref,u_base,"
        "u_fr,u_fl,u_mr,u_ml,u_rr,u_rl,iter_us,loop_us,cycles,failsafe,ftc,acc_norm").split(',')
FS_NAME = {0: 'none', 1: 'rx_lost(旧)', 2: 'rx_timeout', 3: 'attitude', 4: 'rate', 5: 'overg',
           6: 'rx_failsafe(0x08)', 7: 'rx_framelost(0x04連続)'}


def load(path):
    global COLS
    rows = []
    for line in open(path, encoding='utf-8', errors='replace'):
        line = line.strip()
        if line.startswith('#cols:'):     # ログ自身の列定義を優先する（列追加に追随）
            COLS = [c.strip() for c in line.split(':', 1)[1].split(',')]
            continue
        if not line or line.startswith('#'):
            continue
        f = line.split(',')
        if len(f) != len(COLS):
            continue                      # 途切れた行は捨てる
        try:
            rows.append([float(x) for x in f])
        except ValueError:
            continue
    if not rows:
        sys.exit("読める行がありません: %s" % path)
    a = np.array(rows)
    return {c: a[:, i] for i, c in enumerate(COLS)}


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'log/onboard.txt'
    D = load(path)
    R = 180.0 / np.pi
    t = D['t']
    U = np.vstack([D['u_' + m] for m in ('fr', 'fl', 'mr', 'ml', 'rr', 'rl')])
    print("%d 行 / %.2f s  (%.0f Hz 相当)" % (len(t), t[-1] - t[0], 1.0 / np.median(np.diff(t))))
    fs = D['failsafe']
    if (fs > 0).any():
        i = int(np.argmax(fs > 0))
        print("フェイルセーフ作動: t=%.2f s  原因=%s" % (t[i], FS_NAME.get(int(fs[i]), '?')))
    else:
        print("フェイルセーフ作動なし")
    print("飽和(>=0.94) した時刻の割合 %.1f %%" % (100 * (U >= 0.94).any(0).mean()))

    fig, ax = plt.subplots(5, 1, figsize=(13, 12), sharex=True)
    ax[0].plot(t, D['phi'] * R, lw=1, c='#1f6feb', label='phi')
    ax[0].plot(t, D['phi_ref'] * R, '--', lw=.9, c='#1f6feb', alpha=.5, label='phi_ref')
    ax[0].plot(t, D['theta'] * R, lw=1, c='#d1242f', label='theta')
    ax[0].plot(t, D['theta_ref'] * R, '--', lw=.9, c='#d1242f', alpha=.5, label='theta_ref')
    ax[0].set_ylabel('角度 [deg]'); ax[0].legend(fontsize=8, ncol=4); ax[0].grid(alpha=.3)
    ax[0].set_title('機内 100Hz ログ: %s' % os.path.basename(path))

    ax[1].plot(t, D['p'] * R, lw=1, c='#1f6feb', label='p')
    ax[1].plot(t, D['q'] * R, lw=1, c='#d1242f', label='q')
    ax[1].plot(t, D['r'] * R, lw=1, c='#2da44e', label='r')
    ax[1].plot(t, D['psi_ref'] * R, '--', lw=.9, c='#2da44e', alpha=.5, label='psi_ref')
    ax[1].axhline(300, c='r', ls=':', lw=1); ax[1].axhline(-300, c='r', ls=':', lw=1)
    ax[1].set_ylabel('角速度 [deg/s]'); ax[1].legend(fontsize=8, ncol=4); ax[1].grid(alpha=.3)

    for m, c in zip(('fr', 'fl', 'mr', 'ml', 'rr', 'rl'),
                    ('#d1242f', '#fa8c16', '#2da44e', '#1f6feb', '#8250df', '#57606a')):
        ax[2].plot(t, D['u_' + m], lw=.8, c=c, label=m.upper())
    ax[2].plot(t, D['u_base'], 'k--', lw=1.4, label='u_base')
    ax[2].axhline(0.95, c='r', ls=':', lw=1)
    ax[2].set_ylabel('duty'); ax[2].legend(fontsize=7, ncol=4); ax[2].grid(alpha=.3)

    ax[3].plot(t, D['iter_us'] / 1000, lw=.9, c='#8250df', label='iter_us')
    ax[3].plot(t, D['loop_us'] / 1000, lw=.9, c='#fa8c16', label='loop_us')
    ax[3].axhline(10, c='r', ls=':', lw=1.2)
    ax[3].set_yscale('log'); ax[3].set_ylabel('時間 [ms]')
    ax[3].legend(fontsize=8); ax[3].grid(alpha=.3, which='both')

    ax[4].plot(t, D['cycles'], lw=.9, c='#2da44e', label='cycles_since_reset')
    ax4b = ax[4].twinx()
    ax4b.plot(t, D['acc_norm'], lw=.9, c='#57606a', alpha=.7, label='acc_norm [m/s²]')
    ax4b.set_ylabel('acc_norm [m/s²]')
    ax[4].set_ylabel('cycles'); ax[4].set_xlabel('t [s]')
    ax[4].legend(fontsize=8, loc='upper left'); ax4b.legend(fontsize=8, loc='upper right')
    ax[4].grid(alpha=.3)

    if (fs > 0).any():
        tf = t[int(np.argmax(fs > 0))]
        for a in ax:
            a.axvline(tf, c='r', lw=1.4, ls='--')
        ax[0].text(tf, ax[0].get_ylim()[1] * .8, ' フェイルセーフ', fontsize=9, c='r')

    out = os.path.splitext(path)[0] + '.png'
    fig.tight_layout(); fig.savefig(out, dpi=130)
    print("保存: %s" % out)


if __name__ == '__main__':
    main()
