# SIL で現行 Benzen_NMPC を飛ばした: SIL のモータ順不具合を修正し、実機並みの揺れでは低ランク更新の古さが逆向き指令を生むことを用量反応で確認

- **日付**: 2026-10-08
- **種別**: bugfix | investigation
- **対象**: `program/sil_nmpc/`（`api/sil_api.cpp` の `sil_get_duty`、`host/nmpc_flight.py`、`host/run_nmpc_eval.py`、`results/20261008/`）、`Benzen_NMPC/nmpc_params.hpp`（`rti::reset_period`、**変更はしていない**）
- **タグ**: #SIL #Benzen_NMPC #pwm #set_duty #モータ順 #配線 #TR1 #SR2 #rankUpdate #reset_period #フル再構築 #逆向き指令 #スパイク #健全飛行 #core1 #仮想時間

## 背景 / 症状

ユーザ依頼「program/sil/ で現在の Benzen_NMPC を実行して通常飛行ができるか確認して評価分析」。
`program/sil/`（Benzen_new 用）を `program/sil_nmpc/` に複製して Benzen_NMPC をビルドしたところ、
理想条件でロール +15° を入れた瞬間に **θ が 70° まで振れて墜落**（`姿勢 70° 超 t=8.31`）。

## 原因 / 判断根拠

### (1) SIL の不具合（確認済み）

Benzen_NMPC の `pwm.cpp` は Benzen_new から配線を付け替えてある（`set_duty_fr` → slice7 **B**、`set_duty_rr` → slice7 **A**、
`set_duty_fl` → slice1 B、`set_duty_rl` → slice1 A、`set_duty_ml` → slice2 A、`set_duty_mr` → slice2 B）。
`sil_get_duty` は Benzen_new 用（FR=slice7 A …）のままで、プラントに `[RR, ML, RL, FL, FR, MR]` が届いていた。
firmware ログの u_* とプラント入力を 1 tick ずらして突き合わせると**全値が置換で一致**。
**6 発同値のホバーでは原理的に見えない。** 姿勢指令を入れて初めて出る。

### (2) 前回の中間報告の誤り（確認済み）

「6 発同一・100 Hz ログ 0 行 → core1 が回っていない」は誤り。core1 は 400 tick で 100 回起動していた。
同一 duty は雑音ゼロで姿勢が厳密に 0 だから。ログ 0 行は読み出し側が `loop_us≠0` で行を拾ったため（SIL は仮想時間で loop_us/iter_us が常に 0）。

### (3) 実機並みの揺れでの劣化（SIL で確認、実機への当てはめは推測を含む）

- 雑音（ジャイロ 0.03 rad/s・加速度 60 mg・機体外乱 0.5 rad/s/√s）＋出力遅延 10 ms＋再構築据え置き 30 ms で、
  実機ファームそのまま 18 本中 **1 本墜落**、後半ホバー最悪 10.0°、|Δduty| 最大 0.68–0.88。
- 墜落した 1 本は**通常経路（TR1 採用・|c| 閾値内・box 非有効・λ=0）**のまま、ピッチの指令モーメントが**79 周期連続で戻すべき向きと逆**。EKF は真値に 1° 以内。
- `reset_period` だけを 1/10/30/100 に変えたファーム複製で**単調な用量反応**: |Δduty| 最大 0.11 → 0.16 → 0.49 → 0.72、逆向き指令（ロール）3 → 5 → 10 → 17 %、墜落 0/12, 0/18, 0/18, 1/18。
  **30 ms 据え置きを入れても 10 周期 ≫ 100 周期**（late hover 最悪 2.5° vs 10.0°）。
- 実機ファーム内でも、最後の再構築からの周期数とともに逆向き指令が 9 % → 32 % に増える。
- 実機ログ（0919_v5/v6/v8 巡航）は逆向き指令 0–2 % で SIL より穏やか。v8 だけ 80 周期以降で |Δduty| p95 が 0.04 → 0.22（SIL と同じ形）。
- TR1 棄却率は実機 21–24 % に対し SIL 1–3 %（**未解明**）。実機は棄却 8 連続の再構築で実質の再構築間隔が短くなっている可能性（**推測**）。

## 対応

- `sil_get_duty` の表を `pwm.cpp` の `set_duty_*` の写しに修正（コメントに由来を明記）。
- ホスト側に**モータ順の常設検査**（core1 が起動しなかった tick の PWM 出力 ＝ 最新ログ行の u_*。6 発がそろっていない周期だけ数える）。
  修正前の順を `--legacy-order` で再現すると 390/390 周期で検出、修正後 0。床値（地上 0.01・飛行中 0.1）による差は順序ではない。
- ログ行数は LOG の立ち上がりからの core1 起動回数で数える（`LogRow` は static で読めない）。
- 出力遅延・再構築据え置きをプラント側で模擬（core1 は仮想時間ゼロで終わるため）。
- `CMakeLists.txt` の `FW_DIR` を `-DFW_DIR` で差し替え可能にし、`SIL_BUILD_DIR` でホストが別ビルドを掴めるようにした（アブレーション用。実機ファームは無変更）。
- **ファームのパラメータは変えていない。** `reset_period` 短縮は 2026-08-31 に却下済み（`20260831_nmpc_rebuild_gate_degradation.md`：「停止 58 ms のまま頻度だけ増える」）。
  当時から変わった点は N=17→14 で再構築 31–45 ms、SIL で据え置き込みでも 10 周期が良いこと、因果を用量反応で示したこと。**再検討候補として提示するに留めた。**

## 検証結果

- 決定性: 同じ seed の再実行で同じ時刻（t=9.75）に墜落することを確認。
- 理想条件: 完走、ロール 15°/ピッチ 10° の 90 % 到達 1.2 s・行き過ぎなし、TR1 棄却 88 %（雑音ゼロで割線が小さすぎる）→ 実質 9 周期ごと再構築。
- 指標・図: `docs/20261008_sil_nmpc_flight_eval.md`、`docs/figs/20261008_sil/fig01..07`、`program/sil_nmpc/results/20261008/{metrics,ablation_summary}.csv`。
- **実機では未検証。**

## 次に触る人へ

- **SIL を別ファームに向けたら、最初に非対称な指令（ロール/ピッチのステップ）で順序検査を通すこと。** ホバーが飛べても何も保証しない。
- SIL の据え置きは「出力を凍らせるだけ」で、実機のように 100 Hz 周期そのものは消えない。`reset_period` を実機に入れる判断の前に、core1 周期のスキップを SIL で再現すること。
- SIL の雑音は Python シミュで合わせたもので、SIL の実機ファームは実機より荒れる。劣化の**程度**を実機に当てはめる前に、TR1 棄却率（実機 21–24 %）が合うよう雑音を較正すること。
- TR1（Su）と SR2（Hessian）のどちらの近似が主因かは未分離。
- 関連: `20261006_sim_flies_real_crashes_gap_is_in_controller_internals_not_physics.md`（本件はその「制御器の中身」仮説を SIL で裏付けた）、
  `20260831_nmpc_rebuild_gate_degradation.md` / `20260831_rebuild_gate_reverted.md`（cycles_since_reset と姿勢誤差 +0.70）、
  `20260919_tr1_skip_is_self_reinforcing_not_random.md`、`20260825_sil_m0_firmware_on_host.md`（※これらは作業コピー側の decisions にある）。
