#!/usr/bin/env python3
"""SessionStart hook: 外部記憶（docs/ai/）の存在と開始時ルーチンをコンテキストへ注入する。

このスクリプトは自分の位置（.claude/hooks/）からリポジトリルートを解決するため、
セッションの cwd に依存しない。出力は Claude Code の hook JSON 形式。
失敗しても常に exit 0 で無害な JSON を返し、セッション起動を妨げない。
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DECISIONS = ROOT / "docs" / "ai" / "decisions"

HEADER = """【外部記憶システム（3層）が有効です】

このリポジトリでは docs/ai/ に過去の設計判断・失敗・却下案を蓄積しています。
手順の全文は skills.md、常時参照ルールは docs/ai/repository_rules.md にあります。

■ 着手前に必ず実行（skills.md §1-2）
    grep -ril "<検索語>" docs/ai/decisions/
  実装・修正に手を付ける前に最低 1 回。ヒットしたら全文を読むこと。
  過去に却下された案を、当時から何が変わったかを示さずに再提案しないこと。
  最初の応答で、参照した decision 名（または「関連する過去記録なし」）を 1 行で示すこと。

■ 完了時に必ず実行（skills.md §2）
  バグ原因の特定・設計判断・却下した案・30 分以上かかった調査は
  docs/ai/decisions/YYYYMMDD_概要.md に記録し、INDEX.md に 1 行追記する。
  記録には「なぜ」だけを書く（何を変えたかは git diff が持っている）。
"""


def main() -> None:
    try:
        files = sorted(
            (p.name for p in DECISIONS.glob("*.md") if p.name != "INDEX.md"),
            reverse=True,
        )
    except OSError:
        files = []

    if files:
        listing = "\n".join(f"  - {name}" for name in files[:10])
        body = f"{HEADER}\n■ 直近の decisions（全 {len(files)} 件）\n{listing}\n"
    else:
        body = f"{HEADER}\n■ decisions はまだ 0 件です（前例なしとして扱ってよい）\n"

    json.dump(
        {
            "hookSpecificOutput": {
                "hookEventName": "SessionStart",
                "additionalContext": body,
            },
            "suppressOutput": True,
        },
        sys.stdout,
        ensure_ascii=False,
    )


if __name__ == "__main__":
    try:
        main()
    except Exception:
        sys.stdout.write('{"suppressOutput": true}')
    sys.exit(0)
