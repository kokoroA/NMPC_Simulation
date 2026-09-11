# 外部記憶の運用を「お願い」から「検証」へ。AI ツールに依存しない関門を git hook と CI に置いた

- **日付**: 2026-09-11
- **種別**: design
- **対象**: `scripts/check_memory.py`, `.githooks/pre-commit`, `.github/workflows/memory-check.yml`, `.cursor/rules/research-memory.mdc`, `docs/ai/repository_rules.md`, `.claude/hooks/session_start_memory.py`
- **タグ**: #外部記憶 #運用 #Cursor #Cline #Copilot #フック #CI #symlink #観測性

## 背景 / 症状

ユーザの問い: **「記憶の参照・記録・昇格を Claude 以外の AI（Cursor など）からも自動的に行うには？」**

棚卸しの結果、**入口の配布はすでに完成していた**（`20260822_multi_tool_entrypoints.md`）:
`AGENTS.md` / `.clinerules` / `.cursorrules` / `.github/copilot-instructions.md` はすべて
`docs/ai/repository_rules.md` への symlink で健在、`.cursor/rules/research-memory.mdc` は
`alwaysApply: true` なので **Cursor には毎回自動注入されていた。**

足りなかったのは 2 つ。

1. **動的な情報（decisions に何があるか）が Claude にしか流れていない。**
   SessionStart フックが最新 10 件を注入しているが、他ツールは静的テキストしか見ていない。
2. **検証が皆無。** 「読んでください」「記録してください」とお願いしているだけで、
   **書いたかどうかを誰も確かめていなかった。**

## 原因 / 判断根拠

### 設計の分かれ目: 「読ませる」は強制できないが「書いたか」は検証できる

LLM が実際に過去記録を読んだかは**原理的に検証不能**である。
一方、**成果物（decision ファイルの有無、INDEX の行、必須項目、リンクの健全性）は機械的に検証できる。**
したがって仕組みを 2 系統に分ける:

- **注入系**（`@` import / `alwaysApply` / フック）= 思い出させる。強制力なし。ツールごとに実装が違う。
- **検証系**（git hook / CI）= 成果物を確かめる。**ツール非依存。全員 git を通るのでここが唯一の共通関門。**

### なぜ INDEX の `@` 参照が「フックより効く」か

Cursor の `alwaysApply: true` ルールは `@ファイル` 参照を展開する。
`@docs/ai/decisions/INDEX.md` を 1 行足すだけで、**フックを使わずに最新の全一覧が毎回入る。**
Cursor Hooks はバージョン依存で当てにできないが、`@` 参照は静的ファイルなので確実に効く。
Cline / Copilot / Codex 向けも同じ理屈で `repository_rules.md` 側に目次リンクを置いた。

### 「記録が無い」は止めない

`repository_rules.md` §0 は「typo・フォーマット・リネームなど再現価値のない作業は記録しない」と
定めており、**機械にはその区別がつかない。** 止めると必ず誤検知で邪魔になり、
`--no-verify` が常態化して仕組み全体が形骸化する。よって**警告に留め、判断は人に残す。**
一方、INDEX 漏れ・必須項目欠け・リンク切れは**誰が見ても間違い**なので commit を止める。

## 対応

| 追加/変更 | 役割 |
|---|---|
| `scripts/check_memory.py` | 検証の実体。INDEX 漏れ / 必須項目欠け / リンク切れ = エラー、記録無し = 警告 |
| `.githooks/pre-commit` | 全ツール共通の関門。`git config core.hooksPath .githooks` で有効化 |
| `.github/workflows/memory-check.yml` | `--no-verify` を抜けても止まる最後の砦。symlink の健全性も検査 |
| `.cursor/rules/research-memory.mdc` | `@docs/ai/decisions/INDEX.md` を追加（Cursor に一覧が毎回入る） |
| `docs/ai/repository_rules.md` | 目次リンク＋`core.hooksPath` の設定手順＋「読んだかは検証できない」旨を明記 |
| `.claude/hooks/session_start_memory.py` | `--plain` を追加。**文言を 1 か所に保つため**、他ツールのフックからも同じ本文を使える |

**却下した案**:
- **Cursor Hooks で注入する**: バージョン依存で、無い環境では何も起きない。
  `@` 参照で同等以上のことができるので、フックは「使えるなら足す」扱いに留めた。
- **記録が無い commit を block する**: 上記のとおり誤検知で形骸化する。
- **各ツール向けに文言をコピーする**: `skills.md` §6 が禁じている（片方が古くなる事故）。
  `--plain` はそのための措置で、コピーを増やさずに再利用できるようにした。

## 検証結果

`scripts/check_memory.py` の動作を実際に作って確かめた:

| テスト | 結果 |
|---|---|
| INDEX 追記漏れ | ✗ 検出、exit 1 |
| 必須項目（日付/種別/タグ）欠け | ✗ 検出、exit 1 |
| コード変更あり・記録なし | ⚠ 警告のみ、exit 0（止めない） |
| 正常時 | ✓ OK、exit 0 |

**初回実行でリンク切れを 1 件検出した**（`20260909_v1_roll_departure_and_nan_lockup.md` の
`../../program/...` は `../../../program/...` が正しい。決定ファイルは `docs/ai/decisions/` に
あるのでルートまでは 3 つ上がる）。修正済み。リンク総数 357 件のうち残りは健全。

`.githooks/pre-commit` は `sh -n` で構文確認。CI の YAML は GitHub 側で検証される。
**CI の実走は未検証**（push して初めて分かる）。

## 次に触る人へ

- **クローンしたら `git config core.hooksPath .githooks` を 1 回実行すること。**
  これを忘れると検証系が丸ごと無効になる。`repository_rules.md` §0 に手順を書いた。
- **この仕組みは「読んだか」を保証しない。** 保証するのは記録の整合性だけである。
  「AI が過去記録を参照した」ことの担保は、今も**最初の応答で decision 名を挙げさせる**という
  自己申告に頼っている。そこは変わっていない。
- 検査を足すなら `scripts/check_memory.py` に関数を 1 つ増やすだけで hook と CI の両方に効く。
  **hook 側と CI 側でロジックを二重に書かないこと。**
- `.cursor/rules` に `@` を足した効果は未測定。Cursor 側で実際に INDEX が読まれているかは、
  次に Cursor を使うときに「参照した decision 名を挙げたか」で確認するとよい。
- 関連: [`20260822_multi_tool_entrypoints.md`](20260822_multi_tool_entrypoints.md)、
  [`20260820_activate_memory_hooks.md`](20260820_activate_memory_hooks.md)、
  [`000000_setup_intelligence.md`](000000_setup_intelligence.md)
