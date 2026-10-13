"""run_nmpc_eval.py をまとめて並列実行する（SIL は 1 プロセス 1 インスタンスなので別プロセス）。

    python3 run_jobs.py jobs.txt [-j 4]

jobs.txt の 1 行 = run_nmpc_eval.py の引数。行頭に `SIL_BUILD_DIR=<dir> ` を書くと別ビルドを使う。既に --out があれば飛ばす。
行頭に `SIL_SCRIPT=run_ftc_eval.py ` を書くと別のスクリプトを呼ぶ（2026-10-13、FTC 評価用）。
"""
import os, shlex, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))


def run(line):
    env = dict(os.environ)
    toks = shlex.split(line)
    while toks and '=' in toks[0] and not toks[0].startswith('-'):
        k, v = toks.pop(0).split('=', 1); env[k] = v
    if '--out' in toks and os.path.exists(os.path.join(HERE, toks[toks.index('--out') + 1])):
        return f'skip {toks[toks.index("--out") + 1]}'
    r = subprocess.run([sys.executable, env.get('SIL_SCRIPT', 'run_nmpc_eval.py')] + toks, cwd=HERE, env=env, capture_output=True, text=True)
    return (r.stdout.strip().splitlines() or [''])[-1] + (f'\n  ERR {r.stderr.strip()[-300:]}' if r.returncode else '')


if __name__ == '__main__':
    lines = [l.strip() for l in open(sys.argv[1]) if l.strip() and not l.startswith('#')]
    j = int(sys.argv[sys.argv.index('-j') + 1]) if '-j' in sys.argv else 4
    with ThreadPoolExecutor(j) as ex:
        for out in ex.map(run, lines):
            print(out, flush=True)
