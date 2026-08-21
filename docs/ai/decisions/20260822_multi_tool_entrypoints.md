# 他 AI ツールへの記憶配布：入口を増やし、repository_rules.md 単体で運用が回るようにした

- **日付**: 2026-08-22
- **種別**: design
- **対象**: `docs/ai/repository_rules.md`, `skills.md`, `AGENTS.md`, `.cursor/rules/`, `.github/`
- **タグ**: #meta #memory #cursor #cline #copilot #AGENTS.md #symlink #配布

## 背景 / 症状

「`skills.md` は Cursor や ChatGPT からも参照されるのか」という問いを受けて実態を確認したところ、
**設計意図と実装が食い違っていた**。

`skills.md` §5 には既に
「Cline / Cursor にはフックが無いので **`repository_rules.md` 単体で最低限の運用ルールが
伝わる状態を保つこと**」と書いてあった。にもかかわらず:

```bash
grep -n '着手前\|完了時\|grep -ril' docs/ai/repository_rules.md
# → ヒット 0 件
```

つまり **Cline / Cursor には「プロジェクトの知識」だけが届き、「記録しろ」という
運用ルーチンは一行も届いていなかった**。要件を自分で書いておきながら満たしていなかった。

さらに入口ファイル自体も不足していた（`AGENTS.md` / `.cursor/rules/` /
`.github/copilot-instructions.md` がいずれも存在しない）。

## 原因 / 判断根拠

`skills.md`（手順）と `repository_rules.md`（知識）を綺麗に分離した結果、
**分離の境界が、そのままツール間の到達範囲の境界になってしまった**。
`skills.md` は Claude Code の `@` import でしか展開されないため、
手順を全部そちらへ寄せた時点で他ツールは手順ゼロになる。

「関心事で分離する」ことと「配布経路で分離する」ことは別問題であり、
前者だけを考えて後者を検証していなかったのが原因。

## 対応

### 1. `repository_rules.md` に §0「作業ルーチン」を追加（最重要）

symlink 経由で全ツールに届く唯一のファイルなので、ここに最小限の運用を埋め込んだ。

- 着手前: `grep -ril "検索語" docs/ai/decisions/` を必ず 1 回。検索語の例も併記。
  却下済みの案を「当時から何が変わったか」を示さずに再提案しない。
- 完了後: 記録トリガー（バグ原因 / 設計判断 / **失敗・ボツ** / 30 分以上の調査）と、
  `YYYYMMDD_概要.md` + `INDEX.md` 追記の手順。「なぜ」だけを書く。
- サブモジュールで作業しても decision は親リポジトリに書く旨も再掲。

**`skills.md` は §0 の完全版という位置づけになった。§0 を削ってはいけない。**

### 2. 入口ファイルを追加（実体は 1 つ、他は symlink）

| 追加 | 対象ツール | 形式 |
|---|---|---|
| `AGENTS.md` | Codex / Gemini CLI / Zed など | symlink → `docs/ai/repository_rules.md` |
| `.github/copilot-instructions.md` | GitHub Copilot | symlink → 同上 |
| `.cursor/rules/research-memory.mdc` | Cursor（現行形式） | **実体**（`alwaysApply: true` の frontmatter が必須のため symlink 不可） |

`.cursor/rules/*.mdc` だけは frontmatter が要るので実体を置かざるを得なかった。
ただし**本文にルールを複製せず**、`@docs/ai/repository_rules.md` と `@skills.md` への
参照＋ルーチンの要約に留めることで、二重管理の面積を最小化した。

### 採用しなかった案

- **各ツール向けに repository_rules.md を実体コピーする**: 片方だけ古くなる事故が確実に起きる。
  この構成全体が「二重管理をしない」ことを前提に組まれているので却下。
- **`skills.md` も各ツールへ symlink する**: 入口が肥大し、
  「入口は最小・固定」というキャッシュ最適化の設計思想に反する。§0 の要約で足りる。
- **ChatGPT（web）対応**: リポジトリを読まないため原理的に不可能。手貼り以外の解が無い。

## 検証結果

- `sed -n '1,20p' .cursorrules` で、**symlink 経由で §0 が読める**ことを確認。
- `head -1 AGENTS.md` / `head -1 .github/copilot-instructions.md` が
  いずれも `# Repository Rules — research_project` を返し、リンクが解決することを確認。
- `ls -la` で `AGENTS.md` と `.github/copilot-instructions.md` が
  `lrwxr-xr-x`（symlink）であることを確認。

**未検証**: Cursor / Cline / Copilot が**実際にこれらを読み込むか**は、
各ツールを起動しての確認が必要であり、このセッションでは未確認。
特に **symlink を辿らない実装のツールがあり得る**点は要注意。

**検証済み（副次）**: Claude Code の SessionStart フックは
2026-08-22 のセッション開始時に実際に発火し、decisions 一覧が注入されることを確認した
（[20260820_activate_memory_hooks.md](20260820_activate_memory_hooks.md) の「未検証」を解消）。

## 次に触る人へ

- **`repository_rules.md` の §0 を削らないこと。** Claude Code 以外のツールにとっては
  あれが運用ルーチンの全てである。
- 新しい AI ツールを導入したら、その入口ファイル名を調べて symlink を 1 本足す。
  **実体をコピーしない。**
- あるツールがルーチンを守っていないと感じたら、まず
  「そのツールが symlink を辿れているか」「入口ファイル名が現行仕様か」を疑う
  （`.cursorrules` は旧形式であり、いつ廃止されてもおかしくない）。
- 関連: [000000_setup_intelligence.md](000000_setup_intelligence.md),
  [20260820_activate_memory_hooks.md](20260820_activate_memory_hooks.md)
