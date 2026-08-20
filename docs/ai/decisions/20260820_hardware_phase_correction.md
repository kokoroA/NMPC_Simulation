# 主戦場の訂正：シミュレータ中心 → 実機（Benzen_NMPC）中心へ

- **日付**: 2026-08-20
- **種別**: investigation
- **対象**: `docs/ai/repository_rules.md`, `skills.md`
- **タグ**: #meta #memory #Benzen_NMPC #Pico #RP2350 #submodule #paper #typst #phase

## 背景 / 症状

外部知能基盤の初期構築時、`repository_rules.md` に
**「主戦場 = `program/NMPC_simulation/`（Python シミュレータ）」**と書いた。
これは誤りで、ユーザーからの指摘により以下が実態と判明した。

- 現在は**実機実験フェーズ**であり、主戦場は
  `program/pico/pico-sdk2/pico-sdk/Benzen_NMPC`（Raspberry Pi Pico 2 W ファームウェア）
- 予稿は `documents/論文/日本航空宇宙学会/原稿/paper.typ`（Typst）
- `NMPC_simulation` は「現在の主戦場」ではなく**検証済みの参照実装**

## 原因 / 判断根拠

初期調査でリポジトリを走査した際、**実機コードが視界に入っていなかった**。理由は 2 つ:

1. `Benzen_NMPC` は `program/pico/pico-sdk2/pico-sdk` という**サブモジュール配下**にあり、
   `git ls-tree` にも親の `git status` にも現れない。
   `git check-ignore` は `fatal: Pathspec ... is in submodule` を返す。
2. 作業していた worktree には `pico-sdk` の中身が展開されておらず、
   `program/pico/` には `eigen/` と空の `pico-sdk2/` しか見えなかった。

同様に `documents/論文/日本航空宇宙学会/` も**未追跡**であり、git 由来の情報だけを見ていると
「存在しないもの」として扱われる。

**教訓: git の視界（追跡対象）と、プロジェクトの実際の重心は一致しない。**
特にサブモジュール・未追跡ディレクトリ・worktree の組み合わせでは、
`git ls-files` ベースの調査は静かに嘘をつく。

## 対応

`repository_rules.md` を実機フェーズ向けに全面改訂した。

- §1 フェーズ宣言を「実機実験フェーズ／主戦場は Benzen_NMPC」に変更
- §2 に **「⚠ git 管理の地雷」** 節を新設し、サブモジュール・未追跡の事実を明記
- §3 に実機固有の方針を追加（**絶対モード + `U_ref` = ホバー baseline**、
  実機モータ順 `[FR, FL, MR, ML, RR, RL]`、core1=100Hz / core0=400Hz の分担）
- §4 に実機の制約を追加（単精度 float、`EIGEN_NO_MALLOC`、`N` が最重要ノブ、
  `CMakeLists.txt` の Eigen 絶対パスハードコード）
- §5 到達点を実機の実測値ベースに上書き

`skills.md` は Step 1-3（隣接する一次情報）を表形式にし、実機側の一次情報を先頭に置いた。
検索語の例も実機語彙（`RP2350`, `core1`, `EIGEN_NO_MALLOC`, `sysid` 等）へ差し替えた。
さらに「**サブモジュールで作業しても decision は親リポジトリに書く**」という節を追加した
（記憶が 2 か所に割れると grep で引けなくなるため）。

### 記録した重要な事実（今後の判断の前提）

- **`N` の正は `nmpc_params.hpp` の `N = 17`。** `plan.md` に残る `N=20` は古い記述。
- **実機の入力モードは絶対モード。** シミュの加算合成（PKG-C）は C++ 未実装で、
  これは意図的な差異。「シミュと違う」ことを理由に片方へ寄せてはいけない。
- **直近実測: ソルバ本体 平均 8.7 ms、制御周期全体 10.6 ms** → 10 ms をわずかに超過中。
  最優先課題はこの超過解消（`N` 短縮または周辺処理削減）。
- 隣接 2 発故障はピッチ軸の権限を失うため困難（配置依存の限界）。対向 2 発までは維持可能。

## 検証結果

- `Benzen_NMPC/nmpc_params.hpp`, `CMakeLists.txt`, `docs/NMPC_REPORT.md`, `plan.md` を実読して確認。
- `paper.typ` の「実機構成」「計算時間の実測」「現状の到達点と今後」節から実測値と課題順を確認。
- `CMakeLists.txt` の Eigen パス `/Users/kokoro/Documents/drone_project/pico/eigen` は
  `ls -d` で**実在を確認**（別プロジェクト配下のため、環境移行で壊れる）。
- `git ls-files program/pico` / `git ls-files documents/論文/日本航空宇宙学会` が
  いずれも空であることを確認（＝未追跡）。

**未検証**: `Benzen_NMPC` のビルドは実行していない（この作業では不要なため）。

## 次に触る人へ

- **調査を git ベースだけで済ませないこと。** このリポジトリはサブモジュールと未追跡ディレクトリに
  重要物が入っている。`find` / `ls` で実ファイルを見るまで「無い」と判断しない。
- `repository_rules.md` §5 は上書き更新する節。実機の実測値が変わったら**古い数値を残さない**。
- 実機の数値を変えたら `paper.typ` の記述との整合も確認する（予稿に実測値が載っている）。
- 関連: [000000_setup_intelligence.md](000000_setup_intelligence.md),
  [20260820_activate_memory_hooks.md](20260820_activate_memory_hooks.md)
