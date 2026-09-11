# 耐故障(FTC)モードとフェイルセーフを実装。η は box 制約の縮減と U_ref シフトで表現

- **日付**: 2026-09-01
- **種別**: design | bugfix
- **対象**: `Benzen_NMPC/` control.{cpp,hpp}, nmpc.{cpp,hpp}, radio.{cpp,hpp}, wifi_telemetry.hpp, tools/nmpc_udp_client.py
- **タグ**: #Benzen_NMPC #FTC #耐故障 #eta #box制約 #U_ref #ADMM #フェイルセーフ #SBUS #OverG #REDCIRCLE #LED

## 背景 / 症状

プロポ ch5 で耐故障モードを切り替え、FR モータを止め、NMPC にもそれを認識させたい。
併せて未実装だった受信機フェイルセーフと脱出条件を入れる。

**着手時、ビルドが壊れていた。** `radio.hpp` の `#define REDCIRCLE 5` が `#define FTC 5` に置き換わり、
`control.cpp` の 4 箇所で `REDCIRCLE` が未定義になっていた。

## 原因 / 判断根拠

**確認済み事実。**

- **`REDCIRCLE` はチャンネル番号と `Flight_mode` の値の両方に使われていた。**
  `control.hpp` の `#define REDCIRCLE 3` はコメントアウトされており、
  `Chdata[REDCIRCLE]`（ch5 を読む）と `Flight_mode == REDCIRCLE`（5 と比較）が**たまたま両方 5 で成立**していた。
  `HOVERING`（=8）も同じ構造で、**LED と `Merker_Hovering()` の 2 箇所が死んでいた**（`Flight_mode` が 8 になることはない）。
- **ch5 は `Flight_mode` の選択条件にも入っていた。** 外さないと FTC スイッチでモードまで変わる。
- **SBUS のフラグバイトは `Chdata[16] = sbus_data[23]` として取得済みだが、どこからも参照されていなかった。**
- **OverG のしきい値 800 m/s²（≈81.5 g）は `LSM9DS1_16g`（最大 157、3 軸合成 272）では到達不能**だった。

**設計判断。**

`repository_rules.md §3`「耐故障は η を box 制約の縮減と `U_ref` シフトで表現する。
故障パターンごとの分岐や専用ミキシングは書かない」に従った。

- ADMM の box をモータごとに: `Δu_j ∈ [-u_hover, η_j·U_duty_max - u_hover]`。
  **`η=0` で下限＝上限＝`-u_hover` となり duty が 0 に固定される。**
  これが「NMPC に FR が回らないことを認識させる」の実体。
- `U_ref` シフト: 故障モータ `-u_hover`、健全モータ `+u_hover·n_lost/n_healthy`。
  入れないと `R_u` が肩代わりを罰し続ける。
- **`H = Suᵀ Q̄ Su + R̄` と LLT は box に依存しないため、η 変更でフル再構築（34 ms）は起きない。**

## 対応

計画書 [`docs/20260901_ftc_mode_plan.md`](../../20260901_ftc_mode_plan.md) に決定事項・
フェイルセーフ仕様・実施記録をまとめた。

- 停止モータは **`int8_t Ftc_target_motor`（既定 `MOTOR_FR`）の 1 行**で変更できる。
  `MOTOR_NONE` で「FTC に入っても何も止めない」も選べる。
- η は**一気に 0**（ユーザ判断）。スイッチで ON/OFF 両方向。
- LED は `Arm_flag==2` の分岐の**先頭**に置き、Flight_mode に関係なく FTC 中は必ず
  通常表示⇔全周青（4 Hz）になるようにした。
- テレメトリ `ftc` 列は `NmpcTelPkt` の**未使用 `_pad` を転用**したため、**パケット長 187 は不変**。
  既存のログ解析が壊れない。
- フェイルセーフは**ラッチ式**（`Arm_flag != 2` まで解除しない）。自動復帰させない。

**却下した案**: FR 専用のミキシングを書く。`repository_rules §3` に反し、
他の故障パターンへ広げるたびに分岐が増える。η 配列なら 1 行で対象を変えられる。

**却下した案**: `Flight_mode = FTC` と書く（ユーザ表記どおり）。
`FTC` は radio.hpp のチャンネル番号 5 であり、**今回ビルドを壊した混同そのもの**。
`MODE_FTC = 2` を新設した。

**却下した案**: 加速度しきい値を 120 m/s² 以下にする。
誤検知で飛行中に止まる方が危険なので **150 m/s²（≈15 g）から始め、係留ログを見てから下げる。**

**確定したしきい値（ユーザ判断）**: 加速度 150 m/s² / 姿勢 **60°** / 角速度 300 deg/s / 受信途絶 200 ms。
姿勢 60° は `update_attitude_refs_from_sticks()` の指令 ±54° に対して 6° のマージンを取ったもの。
**指令スケールを広げるときはしきい値も対で見直すこと。**

**フェイルセーフ成立はテレメトリに出す（ユーザ判断）。** `_pad` は `ftc` で使い切ったため
`failsafe` を 1 バイト追加し、**パケット長 187 → 188、`NMPC_UDP_VER` 3 → 4** とした。
ファームとクライアントの版を揃えないと列がずれる。

## 検証結果

- **ビルド成功。** `text 675720 / bss 352224`。
- `static_assert`: `sizeof(NmpcTelPkt) == 188`、`offsetof(ftc)==14` / `offsetof(failsafe)==15` /
  `offsetof(cycles_since_reset)==16`、`NMPC_UDP_VER == 4`。Python の `TEL_FMT` も 188 で一致。
- η→box→duty の算術確認: `u_hover` 0.10–0.78 のすべてで **`η=0` → duty = 0.00**。
- `Flight_mode` をチャンネル定数と比較している箇所が残っていないことを確認。
- **実機での動作確認は一切していない。** 机上（プロペラ無し）での FR 停止、
  LED、`u_fr=0` のテレメ確認、係留での姿勢維持はすべて**未実施**。
- フェイルセーフのしきい値は**実測に基づいていない**。係留ログでの検証が必要。

## 次に触る人へ

- **チャンネル番号とモード値に同じマクロを使わないこと。** 今回の事故の根本原因。
  `radio.hpp` は ch 番号、`control.hpp` は `MODE_*` と分離した。**混ぜると数年後に必ず壊れる。**
- **`cnt_*` や取得済みフラグが「使われていない」パターンに注意。**
  `Chdata[16]`（SBUS フラグ）は grep すると存在するので実装済みに見えるが、参照が無かった。
- 停止するモータを変えるときは **`control.cpp` の `Ftc_target_motor` 1 行だけ**。
  `Motor_eta[]` は自動で作り直される。
- ⚠ **`update_attitude_refs_from_sticks()` のスケールが ±54° の場合、
  フルスティックで脱出条件の 45° に触れる可能性がある。** 感度としきい値は対で決めること。
- 関連: [`20260831_nmpc_n14_confirmed.md`](20260831_nmpc_n14_confirmed.md)、
  [`20260828_nmpc_review_failsafe_tmax.md`](20260828_nmpc_review_failsafe_tmax.md)
