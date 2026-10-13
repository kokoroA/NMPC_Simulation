"""EKF のずれとソルバの暴れへの対策候補の比較（SIL、ファーム複製に比較用スイッチ）。

    <matplotlib のある python> analyze_fix.py <図の出力ディレクトリ>
"""
import csv, glob, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '20261008_plant'))
sys.path.insert(0, os.path.join(HERE, '..', '20261008'))
from metrics_plant import run_metrics, load   # noqa: E402

SH = os.path.join(HERE, '..', '20261008_shake', 'runs')
R_ = os.path.join(HERE, 'runs')
EKF = [('ekf_base', '現行\n(R そのまま)'), ('ekf_rx10', '加速度計の\n重み 1/10'), ('ekf_rx100', '加速度計の\n重み 1/100'),
       ('ekf_gate', '大きさで判定\n|‖a‖−g|>1'), ('ekf_oracle', '理想の補償\n(比力を除去)'),
       ('combo_suexact', '[Su厳密]\n現行 EKF'), ('combo_suexact_rx10', '[Su厳密]\n重み 1/10'), ('combo_suexact_oracle', '[Su厳密]\n理想の補償')]
SOL = [('stock', '現行\n(TR1/SR2)'), ('rp10', '再構築\n10 周期'), ('suexact', 'Su 毎周期厳密\nH は 100 周期'), ('noupd', '差分更新なし\n(100 周期)')]
CONDS = [('shake', '実機並みの揺れ'), ('gust', '+乱流 0.3 m/s'), ('wind', '+風 1.5 m/s')]


def sol_paths(var, cond):
    if var == 'stock':
        return sorted(glob.glob(os.path.join(SH, {'shake': 'full', 'gust': 'gust', 'wind': 'wind'}[cond] + '_s*.npz')))
    if var == 'rp10' and cond == 'shake':
        return sorted(glob.glob(os.path.join(SH, 'rp10full_s*.npz')))
    return sorted(glob.glob(os.path.join(R_, f'sol_{var}_{cond}_s*.npz')))


KEYS = ['est_err_rms', 'est_err_max', 'hover_true_rms', 'true_err_max', 'roll_true_17', 'pitch_true_17',
        'du_max', 'du_p99', 'du_rms', 'wrong', 'path2']


def agg(rs):
    row = dict(n=len(rs), crash=sum(bool(r['crash']) for r in rs))
    for k in KEYS: row[k] = float(np.nanmean([r[k] for r in rs])) if rs else float('nan')
    row['true_err_max_worst'] = float(np.nanmax([r['true_err_max'] for r in rs])) if rs else float('nan')
    row['est_err_max_worst'] = float(np.nanmax([r['est_err_max'] for r in rs])) if rs else float('nan')
    row['big30'] = sum(r['true_err_max'] > 30 for r in rs)
    return row


if __name__ == '__main__':
    E = {g: [run_metrics(p) for p in sorted(glob.glob(os.path.join(R_, f'{g}_s*.npz')))] for g, _ in EKF}
    S = {(v, c): [run_metrics(p) for p in sol_paths(v, c)] for v, _ in SOL for c, _ in CONDS}
    rows = []
    for g, lab in EKF:
        a = agg(E[g]); a.update(group=g); rows.append(a)
    for (v, c), rs in S.items():
        a = agg(rs); a.update(group=f'{v}_{c}'); rows.append(a)
    keys = ['group', 'n', 'crash', 'big30'] + KEYS + ['true_err_max_worst', 'est_err_max_worst']
    with open(os.path.join(HERE, 'fix_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader()
        for r in rows: w.writerow({k: (round(r[k], 4) if isinstance(r[k], float) else r[k]) for k in keys})
    for r in rows:
        print(f"{r['group']:16s} n={r['n']:2d} crash={r['crash']} >30°={r['big30']} " + ' '.join(f'{k}={r[k]:.3g}' for k in KEYS[:4] + ['du_max', 'wrong', 'path2', 'true_err_max_worst', 'est_err_max_worst']))
    if len(sys.argv) < 2: sys.exit()

    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for fam in ('Hiragino Sans', 'Hiragino Maru Gothic Pro', 'AppleGothic'):
        try:
            matplotlib.font_manager.findfont(fam, fallback_to_default=False); plt.rcParams['font.family'] = fam; break
        except Exception:
            pass
    plt.rcParams['axes.unicode_minus'] = False
    OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
    C = ['#8a8986', '#2a78d6', '#4a3aa7', '#eda100', '#1baf7a', '#b9b8b4', '#7fb3ef', '#8fd6bb']

    # 図12: EKF 対策
    items = [('est_err_rms', '推定 − 真値 rms [deg]'), ('est_err_max', '推定 − 真値 の最大（各本の平均）[deg]'),
             ('hover_true_rms', 'ホバー中の真の姿勢誤差 rms [deg]'), ('true_err_max', '真の姿勢誤差の最大 [deg]')]
    fig, axs = plt.subplots(1, 4, figsize=(22, 5))
    for ax, (k, lab) in zip(axs, items):
        ax.axvline(4.5, color='#52514e', lw=0.6, ls=':')
        for i, (g, gl) in enumerate(EKF):
            v = np.array([r[k] for r in E[g]], float)
            ax.bar(i, np.nanmean(v), color=C[i], alpha=0.85)
            ax.scatter(np.full(v.size, i) + np.linspace(-0.25, 0.25, v.size), v, s=7, color='k', zorder=3)
        ax.set_xticks(range(len(EKF))); ax.set_xticklabels([g[1] for g in EKF], fontsize=7); ax.set_title(lab, fontsize=10)
    fig.suptitle('図12  EKF の「推定が浅くなる」対策（full プラント・実機並みの揺れ＋ジャイロバイアスの漂流 0.001 rad/s/√s、各 12 本）\n'
                 '左 5 本 = 現行ソルバ、右 3 本 = ソルバを「Su 毎周期厳密」にしたもの（ソルバの暴れを除いて EKF の効果だけを見る）。「理想の補償」は比力の成分を完全に除いた上限', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.86]); fig.savefig(os.path.join(OUT, 'fig12_ekf_fixes.png'), dpi=130, bbox_inches='tight'); plt.close(fig)

    # 図13: ソルバ対策
    items = [('true_err_max', '真の姿勢誤差の最大 [deg]'), ('du_max', '|Δduty| 最大'), ('wrong', '逆向き指令の割合'), ('du_rms', '|Δduty| rms')]
    fig, axs = plt.subplots(1, 4, figsize=(20, 5))
    w = 0.2
    for ax, (k, lab) in zip(axs, items):
        for i, (v, vl) in enumerate(SOL):
            for j, (c, cl) in enumerate(CONDS):
                vals = np.array([r[k] for r in S[(v, c)]], float)
                x = j + (i - 1.5) * w
                ax.bar(x, np.nanmean(vals) if vals.size else 0, w, color=C[i], label=vl.replace('\n', ' ') if j == 0 else None)
                if vals.size: ax.scatter(np.full(vals.size, x), vals, s=4, color='k', zorder=3)
                if k == 'true_err_max' and vals.size:
                    nb = int((vals > 30).sum()); nc = sum(bool(r['crash']) for r in S[(v, c)])
                    if nb: ax.text(x, np.nanmax(vals) + 1, f'{nb}', ha='center', fontsize=8, color='#e34948')
        ax.set_xticks(range(len(CONDS))); ax.set_xticklabels([c[1] for c in CONDS]); ax.set_title(lab, fontsize=10)
    axs[0].legend(fontsize=8)
    fig.suptitle('図13  ソルバの暴れへの対策（full プラント、各 12 本）。図の赤数字 = 真の姿勢誤差が 30° を超えた本数\n'
                 '「Su 毎周期厳密」は再構築の頻度（＝停止の頻度）を現行の 100 周期のまま変えずに、Su だけ毎周期作り直す', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.88]); fig.savefig(os.path.join(OUT, 'fig13_solver_fixes.png'), dpi=130, bbox_inches='tight'); plt.close(fig)

    # 図14: 暴れた本の比較（同じ seed・同じ条件で現行と Su 毎周期厳密）
    pairs = [('gust', 4, '乱流 seed 4（現行は墜落）'), ('wind', 0, '風 seed 0（現行は −46°）')]
    fig, axs = plt.subplots(len(pairs), 2, figsize=(16, 4 * len(pairs)), sharex=True)
    for row, (c, s, ttl) in enumerate(pairs):
        for col, v in enumerate(('stock', 'suexact')):
            ps = [p for p in sol_paths(v, c) if p.endswith(f'_s{s}.npz')]
            if not ps: continue
            d, L, cc, P, pc, tP, t, idx = load(ps[0])
            ax = axs[row, col]
            ax.plot(tP, np.degrees(P[:, pc['phi']]), color='#2a78d6', label='φ 真値'); ax.plot(t, np.degrees(L[:, cc['phi']]), '--', color='#2a78d6', lw=1)
            ax.plot(tP, np.degrees(P[:, pc['theta']]), color='#eb6834', label='θ 真値'); ax.plot(t, np.degrees(L[:, cc['theta']]), '--', color='#eb6834', lw=1)
            ax2 = ax.twinx(); U = L[:, 10:16]; ax2.plot(t[1:], np.abs(np.diff(U, axis=0)).max(1), color='#52514e', lw=0.6); ax2.set_ylim(0, 3); ax2.set_yticks([0, 0.5, 1])
            ax.set_ylim(-50, 50); ax.axvspan(7, 9, color='#2a78d6', alpha=0.05); ax.axvspan(11, 13, color='#eb6834', alpha=0.05)
            ax.set_title(f"{ttl} / {dict(SOL)[v].replace(chr(10), ' ')}: {('墜落 ' + str(d['crash'])) if str(d['crash']) else '完走'}", fontsize=10)
            if row == 0 and col == 0: ax.legend(fontsize=8)
    for ax in axs[-1]: ax.set_xlabel('LOG 開始からの時間 [s]')
    fig.suptitle('図14  暴れた 2 本を同じ条件・同じ乱数で: 現行（左）と Su 毎周期厳密（右）。実線 = 真値、破線 = 推定、灰色 = |Δduty|（右軸）', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.95]); fig.savefig(os.path.join(OUT, 'fig14_burst_cases.png'), dpi=130, bbox_inches='tight'); plt.close(fig)
    print('figs', sorted(os.listdir(OUT)))
