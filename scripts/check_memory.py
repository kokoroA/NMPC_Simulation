#!/usr/bin/env python3
"""外部記憶（docs/ai/）の整合性チェック。AI ツールに依存しない唯一の関門。

なぜ必要か:
  CLAUDE.md / .cursorrules / AGENTS.md 等は「読んでください」とお願いしているだけで、
  読んだか・書いたかを誰も確かめていなかった。LLM に「読ませる」ことは原理的に強制できないが、
  **成果物（decision ファイルがあるか、INDEX に行があるか）は機械的に検証できる。**
  ここを git hook と CI から呼ぶことで、どの AI を使っても同じ関門を通る。

使い方:
  scripts/check_memory.py --staged        # コミット予定の変更を見る（pre-commit 用）
  scripts/check_memory.py --range A..B    # コミット範囲を見る（CI 用）
  scripts/check_memory.py                 # 作業ツリー全体の整合性だけ見る

終了コード:
  0 = エラーなし（警告はあり得る）
  1 = エラーあり（INDEX 漏れ / frontmatter 欠け / リンク切れ）
  ⚠ 「decision が書かれていない」は **警告** に留める。repository_rules.md が
     「typo・フォーマット・リネームなど再現価値のない作業は記録しない」と定めており、
     機械には区別できないため。止めるのではなく気づかせるのが役割。
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DECISIONS = ROOT / "docs" / "ai" / "decisions"
INDEX = DECISIONS / "INDEX.md"

# 記録のトリガーになりうる「コードを触った」とみなすパス
CODE_PREFIXES = ("program/",)
# 記録そのもの・記録に準ずるもの
MEMORY_PREFIXES = ("docs/ai/decisions/", "docs/ai/global_drafts.md", "docs/ai/repository_rules.md")

REQUIRED_FIELDS = ("**日付**", "**種別**", "**タグ**")

errors: list[str] = []
warnings: list[str] = []


def git(*args: str) -> str:
    try:
        return subprocess.run(
            ["git", *args], cwd=ROOT, capture_output=True, text=True, check=True
        ).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


def changed_files(staged: bool, rng: str | None) -> list[str]:
    if staged:
        out = git("diff", "--cached", "--name-only", "--diff-filter=ACMR")
    elif rng:
        out = git("diff", "--name-only", "--diff-filter=ACMR", rng)
    else:
        return []
    return [line for line in out.splitlines() if line]


def check_record_present(files: list[str]) -> None:
    """コードを触ったのに decision が増えていない → 警告."""
    touched_code = [f for f in files if f.startswith(CODE_PREFIXES)]
    touched_memory = [f for f in files if f.startswith(MEMORY_PREFIXES)]
    if touched_code and not touched_memory:
        warnings.append(
            "コード（%s ほか %d 件）を変更していますが docs/ai/decisions/ に追記がありません。\n"
            "    バグ原因の特定 / 設計判断 / パラメータ変更 / 試して失敗した / 30 分以上の調査\n"
            "    のいずれかに当てはまるなら docs/ai/decisions/YYYYMMDD_概要.md を書いてください。\n"
            "    typo・フォーマット・リネームだけなら記録不要です（repository_rules.md §0）。"
            % (touched_code[0], len(touched_code))
        )


def check_index_entries(files: list[str]) -> None:
    """新規/変更された decision が INDEX.md に載っているか."""
    if not INDEX.exists():
        errors.append("docs/ai/decisions/INDEX.md が存在しません。")
        return
    index_text = INDEX.read_text(encoding="utf-8", errors="replace")
    targets = [
        f for f in files
        if f.startswith("docs/ai/decisions/") and f.endswith(".md") and not f.endswith("INDEX.md")
    ]
    for f in targets:
        name = Path(f).name
        if name not in index_text:
            errors.append(
                f"{name} が INDEX.md に載っていません（1 行追記してください）。"
            )


def check_frontmatter(files: list[str]) -> None:
    """decision の必須項目（日付・種別・タグ）が揃っているか."""
    targets = [
        f for f in files
        if f.startswith("docs/ai/decisions/") and f.endswith(".md") and not f.endswith("INDEX.md")
    ]
    for f in targets:
        p = ROOT / f
        if not p.exists():
            continue
        head = "\n".join(p.read_text(encoding="utf-8", errors="replace").splitlines()[:15])
        missing = [k for k in REQUIRED_FIELDS if k not in head]
        if missing:
            errors.append(
                f"{Path(f).name} の冒頭に {', '.join(missing)} がありません"
                "（skills.md §2 のテンプレート参照）。"
            )


LINK_RE = re.compile(r"\[[^\]]*\]\((?!https?:|mailto:)([^)#]+)")


def check_links() -> None:
    """decision 同士の相対リンクが切れていないか（リポジトリ全体を毎回見る）."""
    if not DECISIONS.is_dir():
        return
    for p in sorted(DECISIONS.glob("*.md")):
        text = p.read_text(encoding="utf-8", errors="replace")
        for rel in LINK_RE.findall(text):
            rel = rel.strip()
            if not rel:
                continue
            target = (p.parent / rel).resolve()
            if not target.exists():
                errors.append(f"{p.name} のリンク切れ: {rel}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--staged", action="store_true", help="ステージ済みの変更を対象にする")
    ap.add_argument("--range", dest="rng", help="コミット範囲（例: origin/main..HEAD）")
    ap.add_argument("--strict", action="store_true", help="エラーがあれば終了コード 1")
    args = ap.parse_args()

    files = changed_files(args.staged, args.rng)
    if files:
        check_record_present(files)
        check_index_entries(files)
        check_frontmatter(files)
    check_links()

    for w in warnings:
        print(f"  ⚠ {w}")
    for e in errors:
        print(f"  ✗ {e}")

    if not warnings and not errors:
        print("  ✓ 外部記憶の整合性 OK")

    return 1 if (errors and args.strict) else 0


if __name__ == "__main__":
    sys.exit(main())
