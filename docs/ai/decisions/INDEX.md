# Decisions INDEX

`docs/ai/decisions/` の一覧。新しい記録を書いたら 1 行追記する（新しいものを上に）。
本文検索はこの INDEX ではなく `grep -ril "検索語" docs/ai/decisions/` を使うこと。

| 日付 | ファイル | 要約 |
|---|---|---|
| 2026-08-20 | [20260820_hardware_phase_correction.md](20260820_hardware_phase_correction.md) | 主戦場をシミュレータから実機 `Benzen_NMPC` へ訂正。サブモジュール／未追跡のため初期調査で見落としていた |
| 2026-08-20 | [20260820_activate_memory_hooks.md](20260820_activate_memory_hooks.md) | `@` import と SessionStart フックで開始時ルーチンを強制注入。jq 不在のためフックは python3 で JSON を生成 |
| 2026-08-20 | [000000_setup_intelligence.md](000000_setup_intelligence.md) | 3 層記憶（Repository / Archive / Global）と自律運用スキル `skills.md` を構築。入口を固定サイズに保ち過去ログは grep で引く方針を採用 |
