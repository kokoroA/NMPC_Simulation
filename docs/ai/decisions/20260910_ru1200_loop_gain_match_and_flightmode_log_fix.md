# R_u 500→1200（機体ゲイン 2 倍を含めたループゲインで旧に合わせる）、Flight_mode 判定から LOG スイッチを除外

- **日付**: 2026-09-10
- **種別**: tuning | bugfix
- **対象**: `nmpc_params.hpp`(R_u)、`control.cpp`(Flight_mode 判定 1330/1345)
- **タグ**: #Benzen_NMPC #R_u #ループゲイン #GEMFAN #Flight_mode #LOG #LED #FTC

## 背景

[`20260910_v7_prop_change_doubled_plant_gain.md`](20260910_v7_prop_change_doubled_plant_gain.md) の対応案 (a) と LED 修正をユーザが選択。

## (a) R_u = 1200（Q_p = Q_q = 30 据え置き）

合わせる量は「コントローラ × 機体」のループゲイン。旧（0904_v5, D90S）の roll ループ ≈ 0.606e-3 × 393 = 0.24。
新機体（GEMFAN、プラント同定 604–852、代表 700）で同じにするには K角 ≈ 0.34e-3、K率 ≈ 0.27e-3。

ホスト静的計測（角度のみ 8.6°／角速度のみ 20 deg/s、Q_p=30）:

| R_u | K角 | K率 | 率/角 |
|---|---|---|---|
| 500（前回） | 0.570e-3 | 0.503e-3 | 0.87 s |
| 1000 | 0.376e-3 | 0.326e-3 | 0.87 |
| **1200（採用）** | **0.335e-3** | **0.287e-3** | **0.86** |
| 1300 | 0.318e-3 | 0.280e-3 | 0.88 |

角速度側を目標より少し高め（0.287 vs 0.27）に置いた。遅れに対して安定側。
**これは旧と同じ余裕に戻すだけで、40–50 ms 遅れによる 3.8 Hz 限界モードは残る。** 追従は旧同様に鈍い。

## (b) Flight_mode 判定から `Chdata[LOG]` を除外

機構（確認済み）: `Flight_mode = NORMAL / MODE_HOVERING` の条件に `Chdata[LOG] < 200` が入っていたため、
LOG ON の間は更新されない。FTC ブロック（`control.cpp:1363-1381`）は ON で `MODE_FTC` を代入するが
OFF で戻す処理はなく、復帰は NORMAL 分岐頼み。→ LOG ON で FTC を一度触ると `MODE_FTC` が残り、
`led_control()` は `Ftc_mode==0` なので FTC 分岐にも NORMAL 分岐にも入らず消灯。LOG OFF→ON で NORMAL が入って直る。
制御は `Flight_mode` を見ないので表示のみの問題。

対応: 2 箇所の条件から `Chdata[LOG]` を外した。`unique_mission()`（`control.cpp:3070`、宙返り用の未使用関数）にも
`Chdata[LOG] < 200` があるが、呼ばれておらず飛行モードとは無関係なので触っていない。

## 検証結果

- 出荷ソースをホストでビルド: K角 0.335e-3 / K率 0.287e-3。
- 閉ループ（モータ 50 ms、機体効き 2.0–2.5×＝新機体相当、純遅延 1–3 周期）: 収束。
- リプレイ（v4/v6/v7）: `nonfinite=0`、`|U_lin|max` ≤ 5.3。v7 で `lam_reset` が多いのは `t<3.6` の
  受信機 FAILSAFE 区間（`u_hover` が 0.95 に張り付き box が退化）で、飛行区間では発生しない（下記確認）。
- `cmake --build build_fresh` 成功、`text 679360 / bss 422140`。CRLF/LF 整合。
- **実機未検証。**

## 次に触る人へ

- ⚠ `u_hover` が 0.95 に張り付くと `du_max = 0` で box が退化し λ が毎周期リセットされる。
  受信機 FAILSAFE 中はモータが止まっているので実害はないが、スロットル最大付近で飛ぶと同じ状態になる。
  `T_ref_coe` の見直し（[`20260909_v1_roll_departure_and_nan_lockup.md`](20260909_v1_roll_departure_and_nan_lockup.md) 参照）と併せて対処。
- 受信機アンテナの点検を先に。
