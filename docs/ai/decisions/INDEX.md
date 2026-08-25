# Decisions INDEX

`docs/ai/decisions/` の一覧。新しい記録を書いたら 1 行追記する（新しいものを上に）。
本文検索はこの INDEX ではなく `grep -ril "検索語" docs/ai/decisions/` を使うこと。

| 日付 | ファイル | 要約 |
|---|---|---|
| 2026-08-25 | [20260825_sil_m0_firmware_on_host.md](20260825_sil_m0_firmware_on_host.md) | 実機ファームを無改造でホスト共有ライブラリ化（SIL M0）。実機 ELF から SRAM 実測: NMPC は 75.6% 使用、g_nmpc_ctx が 158 KB |
| 2026-08-25 | [20260825_benzen_new_auto_maneuver_caller.md](20260825_benzen_new_auto_maneuver_caller.md) | 自動旋回3種の呼び出しを 100Hz 姿勢 / 400Hz 推力に接続。finished で手動へ戻す。Flight_mode はモック値 |
| 2026-08-25 | [20260825_benzen_new_wifi_telemetry.md](20260825_benzen_new_wifi_telemetry.md) | Benzen_new に Wi-Fi AP + UDP テレメ（10 Hz, ver=1 PID パケット）。NMPC スタック移植。実機 UDP 受信は未検証 |
| 2026-08-25 | [20260825_benzen_new_pico2w.md](20260825_benzen_new_pico2w.md) | Benzen_new を Pico 2 W（RP2350）向けに移植。UF2 は RP2350-ARM-S。実機飛行は未検証 |
| 2026-08-25 | [20260825_benzen_new_pitch_sign_figure8.md](20260825_benzen_new_pitch_sign_figure8.md) | 前傾を機首下げ（負ピッチ）に修正。8の字を左右各約360°の協調旋回2連続へ変更 |
| 2026-08-24 | [20260824_benzen_new_auto_maneuvers.md](20260824_benzen_new_auto_maneuvers.md) | Benzen_new に開ループ姿勢参照生成（水平旋回・8の字・上昇旋回）を純粋関数として追加。推力は暫定 ratio 補正 |
| 2026-08-22 | [20260822_multi_tool_entrypoints.md](20260822_multi_tool_entrypoints.md) | Cursor/Cline/Copilot 等へ入口を symlink で配布。`repository_rules.md` に §0 ルーチンを埋め、単体で運用が回る状態にした |
| 2026-08-20 | [20260820_hardware_phase_correction.md](20260820_hardware_phase_correction.md) | 主戦場をシミュレータから実機 `Benzen_NMPC` へ訂正。サブモジュール／未追跡のため初期調査で見落としていた |
| 2026-08-20 | [20260820_activate_memory_hooks.md](20260820_activate_memory_hooks.md) | `@` import と SessionStart フックで開始時ルーチンを強制注入。jq 不在のためフックは python3 で JSON を生成 |
| 2026-08-20 | [000000_setup_intelligence.md](000000_setup_intelligence.md) | 3 層記憶（Repository / Archive / Global）と自律運用スキル `skills.md` を構築。入口を固定サイズに保ち過去ログは grep で引く方針を採用 |
