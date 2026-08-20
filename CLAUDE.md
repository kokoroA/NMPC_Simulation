# CLAUDE.md

このリポジトリの AI 向け入口です。**このファイルは意図的に短く保つこと。**
実体は `docs/ai/` に置き、ここは import と最小限の指示のみを持ちます。

## 常時読み込み（自動展開）

以下は `@` import によりセッション開始時に自動で内容が展開されます。
**読み飛ばさず、記載されたルーチンと制約に従ってください。**

@skills.md
@docs/ai/repository_rules.md

## 着手前に必ず実行すること

実装・修正に手を付ける前に、Archive レイヤを最低 1 回は検索する（`skills.md` §1-2）。

```bash
grep -ril "検索語" docs/ai/decisions/
```

最初の応答で、参照した decision のファイル名（または「関連する過去記録なし」）を 1 行で示すこと。

## 完了後に必ず実行すること

バグ原因の特定・設計判断・却下した案・30 分以上かかった調査は
`docs/ai/decisions/YYYYMMDD_概要.md` に記録し、
[`INDEX.md`](docs/ai/decisions/INDEX.md) に 1 行追記する（`skills.md` §2）。

汎用的な教訓は [`docs/ai/global_drafts.md`](docs/ai/global_drafts.md) へ昇格させる（`skills.md` §3）。

## 注意

- `.clinerules` / `.cursorrules` は `docs/ai/repository_rules.md` へのシンボリックリンク。
  内容をコピーして二重管理しないこと。
- `program/M5StampFly_NMPC_ADMM/` はネストした別 git リポジトリ。本リポジトリからは変更しない。
