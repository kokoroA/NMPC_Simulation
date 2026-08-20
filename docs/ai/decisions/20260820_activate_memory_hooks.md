# 外部記憶の発火機構（@ import + SessionStart フック）を実装

- **日付**: 2026-08-20
- **種別**: design
- **対象**: `CLAUDE.md`, `.claude/settings.json`, `.claude/hooks/session_start_memory.py`, `skills.md`
- **タグ**: #meta #memory #hook #CLAUDE.md #import #setup #workflow

## 背景 / 症状

`000000_setup_intelligence.md` で 3 層記憶と `skills.md` を構築したが、
発火が「AI が `CLAUDE.md` のポインタを読んで、`skills.md` を読む気になる」という
自発性に依存していた。ポインタは指示であって強制ではないため、
忙しいタスクでは開始時ルーチン（decisions の grep）が素通りされる。

記録する仕組みを作っても、**読む側が起動しなければ記憶は存在しないのと同じ**である。

## 原因 / 判断根拠

「入口を最小に保つ」という設計目標と、「入口を確実に読ませる」という運用目標を
同じ手段（＝ポインタを書く）で満たそうとしていたのが誤り。
前者はファイル構造の問題、後者はハーネスの問題なので、別の手段で解く。

## 対応

2 系統で強制注入する構成にした。片方が効かない環境でももう片方が残る。

1. **`@` import**: `CLAUDE.md` に `@skills.md` と `@docs/ai/repository_rules.md` を記述。
   セッション開始時に両ファイルの中身が実際に展開される。
2. **SessionStart フック**: `.claude/settings.json` から
   `.claude/hooks/session_start_memory.py` を起動し、開始時／完了時ルーチンの要約と
   `docs/ai/decisions/` の最新一覧（最大 10 件）を `additionalContext` として注入する。

フックスクリプトの設計上の判断:

- **jq ではなく python3 を使った。** この環境に `jq` が入っておらず（`which jq` → not found）、
  シェルでの JSON エスケープは日本語・改行・引用符が混ざると壊れやすい。
  `json.dump` に任せるほうが安全。python3 は `/usr/bin/python3` に標準で存在する。
- **スクリプトを別ファイルにし、`__file__` からルートを解決した。**
  settings.json にワンライナーを埋め込む案は、cwd 依存になりサブディレクトリで
  セッションを開くと壊れる。呼び出しは `${CLAUDE_PROJECT_DIR:-.}` でフォールバックさせた。
- **例外を握りつぶして常に exit 0 を返す。** 記憶の注入は補助機能であり、
  これが原因でセッションが起動しないのは本末転倒。

### 採用しなかった案

- **`@` import だけにする**: 推奨構成ではあるが、decisions の「最新一覧」を動的に見せられない。
  一覧が見えると grep の検索語を思いつきやすいので、フックを足す価値があると判断した。
- **フックだけにする**: Cline / Cursor にはフックが無く、それらの環境で完全に無防備になる。
- **UserPromptSubmit フックで毎回注入する**: 確実だがトークンを毎ターン消費し、
  「入口を固定してキャッシュを効かせる」という本プロジェクトの設計思想に反する。

## 検証結果

```bash
echo '{}' | python3 .claude/hooks/session_start_memory.py
```

- 有効な JSON が出力され、`additionalContext` に decisions 一覧（当時 1 件）が含まれることを確認。
- `cd /tmp` から絶対パスで実行し、**cwd 非依存**であることを確認。
- `.claude/settings.json` からコマンド文字列を取り出してそのまま実行し、
  `${CLAUDE_PROJECT_DIR:-.}` の展開込みで動作することを確認。
- settings.json の構文と、SessionStart → command フックの入れ子構造を python で検証。

**未検証**: フックが実際にセッション開始時に発火するところは、このセッション内では確認できない
（SessionStart はこのターンの外側で起こるイベントであるため）。
次回セッションの冒頭で decisions 一覧が見えれば発火成功、見えなければ
`/hooks` を一度開いて設定を再読み込みさせる必要がある。
`@` import についても同様に、次回セッションが最初の実地検証となる。

## 次に触る人へ

- フックの文言を変えたら、必ず上記のパイプテストで JSON が壊れていないか確認すること。
- `CLAUDE.md` にルール本文を書き足さないこと。実体は `docs/ai/` に置き、入口は import のみに保つ。
- `.claude/settings.json` は追跡するが `.claude/worktrees/` と `settings.local.json` は
  `.gitignore` 済み。ワークツリーの実体をコミットしないこと。
- 関連: [000000_setup_intelligence.md](000000_setup_intelligence.md)
