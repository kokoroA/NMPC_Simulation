# CLAUDE.md

このリポジトリの AI 向け入口です。**このファイルは意図的に短く保つこと。**

## セッション開始時に必ず読むもの

1. [`docs/ai/repository_rules.md`](docs/ai/repository_rules.md) — アーキテクチャ方針・基本制約・現在の到達点（常時参照レイヤ）
2. [`skills.md`](skills.md) — 検索 → 実行 → 記録 → 昇格の自律運用ルーチン（**必ず従うこと**）

## 過去の判断・失敗を引く

```bash
grep -ril "検索語" docs/ai/decisions/
```

実装・修正に着手する前に最低 1 回は実行する。詳細は `skills.md` §1。

## 記録する

バグ原因の特定・設計判断・却下した案は `docs/ai/decisions/YYYYMMDD_概要.md` に残し、
[`INDEX.md`](docs/ai/decisions/INDEX.md) に 1 行追記する。手順とテンプレートは `skills.md` §2。

汎用的な教訓は [`docs/ai/global_drafts.md`](docs/ai/global_drafts.md) へ昇格させる（`skills.md` §3）。
