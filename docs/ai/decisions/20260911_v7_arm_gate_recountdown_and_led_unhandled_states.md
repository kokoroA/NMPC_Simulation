# v7: R_u は無罪。モータが回らないのは離陸ゲートの 2.5 秒カウントダウンが 15 回やり直された結果。LED は `led_control()` の未処理状態で凍る

- **日付**: 2026-09-11
- **種別**: bugfix | investigation
- **対象**: `Benzen_NMPC/control.cpp`(led_control, loop_400Hz, ekf_drift_update, Flight_mode 更新), `tools/0911_v7.csv`, `tools/0911_100Hz_v7.csv`
- **タグ**: #Benzen_NMPC #LED #WS2812 #Arm_flag #LockMode #arm_gate #Flight_mode #MODE_FTC #デッドゾーン #観測性 #ログ

## 背景 / 症状

`R_u` を 620 → 1000 にして飛ばそうとした。ユーザ報告:

> アームして飛行状態に遷移（LED が normal になったのを確認）した後、ディスアームを送っても
> **LED はディスアームの色にならない**。Pico のオンボード LED はアーム時点灯・ディスアーム時点滅なので
> モード遷移はしている。**アーム状態でスロットルを与えてもモータが回らない。**
> 100Hz ログを取る際のプロポのコマンドは正しく受信できていた。

## 原因 / 判断根拠

### (1) `R_u` は無罪（確認済み）

`control.cpp` の mtime は **09/10 16:54** で、v4/v5/v6 を飛ばしたものと同一。
`git status` にもソースの変更なし。ログヘッダの差分も `R_u=620 → 1000` と `reinit=1 → 15` だけ。
**コスト重みが LED やアーム状態機械を壊すことはあり得ない。**

### (2) ⚠ このログは原理的にディスアーム状態を記録できない（重要）

100Hz ログもベンチログも `angle_control()` の中で書かれる。そして
**`angle_control()` は `Arm_flag==1/2` の `sem_release` でしか回らない**（control.cpp:255-257 の既知事項）。

つまり **両ログは「アーム中の区間だけを継ぎ接ぎしたもの」**であり、
`arm` 列が全行 2 なのは「ディスアームしなかった証拠」ではなく **記録できないだけ**。
「100Hz ログでプロポのコマンドは正しく受信できていた」も、**アーム中の話に限定される。**
ここを取り違えると診断が丸ごと狂う。

### (3) モータが回らない直接原因は離陸ゲート（確認済み）

100Hz ログ 16.2 s のうち、**NMPC が走ったのは t=9.25–9.85 の 0.6 秒（59 サンプル、3.6 %）だけ。**
その区間では duty が 0.28 まで上がっており、モータは回る状態だった。

それ以外の全区間で `iter_us = 0`、かつ 6 個の duty が **ぴったり `Disable_duty = 0.1`** に張り付いている。
**`T_ref = 3.47`（フルスティック）でも 0.1 のまま。**

```
nmpc_run = (T_ref * T_ref_coe >= Flight_duty);           // 3.47*0.7 = 2.43 >> 0.05 → true のはず
if (!g_arm_gate_ok && Bench_mode == BENCH_OFF) nmpc_run = false;   // ← ここしかない
```

`T_ref*0.7` が `Flight_duty=0.05` を 2 桁上回っているので、**残る原因は `g_arm_gate_ok == 0` だけ。**

### (4) ゲートの判定条件自体は通っていた。**閉じ直されていた**（確認済み）

| | ゲート条件（両軸 <1 deg/s）を満たす割合 | 最長連続 |
|---|---|---|
| **v7（飛べず）** | **96.7 %** | **9.60 s** |
| v6（飛べた） | 40.3 % | 2.45 s |
| v5 | 57.0 % | 3.24 s |

**v7 は v6 より圧倒的に条件が良い**（機体が卓上で静止していたため）。判定が厳しくて開かないのではない。

`g_arm_gate_ok` を 0 にする場所は 2 か所しかない。

1. `ekf_reinit_on_arm()`（control.cpp:1102）— **アームのたびに呼ばれ、`g_arm_gate_t0` も now に戻す**
2. `loop_400Hz()` の `Arm_flag==2 && LockMode==2 && lock_com()==1`（control.cpp:735）— ディスアーム操作の開始時

そして **`#arm_gate: reinit=15`**（v6 は 1）。**アーム手順が 15 回完了している。**
再初期化のたびに **`warm 1.5 s + hold 1.0 s = 2.5 s` のカウントダウンがやり直される**
（`wait_ms=2491` が実測値）。

**機構**: 「アーム → 2.5 秒はモータが死んでいる → 何も起きないのでディスアーム → 再アーム → またカウントダウン」
というループに入ると、**永遠にモータは回らない。** 15 回の再アームはこれで説明がつく。

### (5) LED が凍る経路は 2 つある（コード上は確定、実機での成立は未確認）

`led_control()`（control.cpp:537-608）は **`else` を持たない if-else チェーン**で、
**どの条件にも当たらないと WS2812 に一切書かずに抜ける＝直前の色のまま固まる。**
一方 `board_led_request()` は `Arm_flag` 各分岐の中から呼ばれ続けるので、
**「色は変わらないがオンボード LED だけ変化する」**という報告と完全に一致する。

未処理の状態は 2 つ:

**(a) `Arm_flag == 4`（ログ出力状態）に分岐が無い。**
`grep -c "Arm_flag == 4"` は 1（`loop_400Hz` の分岐のみ）。`led_control()` には無い。
state 4 は `logdata_out_com()`（ch5 低＋スティック 4 隅）で入り、`motor_stop()` し、
`rgbled_switch(led)` を自前で叩く。抜けるのは `log_output()` が `#end` を出し切ったとき
（control.cpp:2497 で `Arm_flag=3, LockMode=0`）。**v7 の CSV は両方 `#end` で終わっているので実際に入って抜けている。**

**(b) `Arm_flag==2 && Flight_mode==MODE_FTC && Ftc_mode==0`。**
`led_control()` の分岐は `Ftc_mode` / `Flight_mode==NORMAL` / `Flight_mode==MODE_HOVERING` の 3 つで、
**`Flight_mode==MODE_FTC` かつ `Ftc_mode==0` の組み合わせに当たる枝が無い。**

そしてこの組み合わせは**実際に作れる**。`Flight_mode` の更新（control.cpp:1479-1495）は

```
if      (Chdata[HOVERING] < 200) Flight_mode = NORMAL;
else if (Chdata[HOVERING] > 500) Flight_mode = MODE_HOVERING;
// 200-500 は**どちらにも当たらず Flight_mode を書かない**
```

**`Chdata[HOVERING]` に 200–500 の未処理デッドゾーンがある。** 一方 FTC ブロックは
`if (Ftc_mode) Flight_mode = MODE_FTC;` と書くだけで、**OFF 時の復帰を上の NORMAL/HOVERING 分岐が
毎周期書き直すことに明示的に依存している**（control.cpp:1511-1512 のコメント）。
HOVERING スイッチがデッドゾーンに居ると**その前提が崩れ、`MODE_FTC` が居座る。**

⚠ `Flight_mode` も `LockMode` も `g_arm_gate_ok` も `Ftc_mode` も **どのログにも出ていない**ので、
(a)(b) のどちらが起きたかは**このログからは決定できない。**

## 対応

**コード変更はまだ行っていない**（原因が観測不能な変数にあるため、まず観測性を足すべき）。

**却下した案**: `R_u` を 620 に戻して様子を見る。(1) のとおり `R_u` は無関係で、切り分けにならない。

## 検証結果

(1)(2)(3)(4) はログとソースから確定。(5) はコード上の欠陥としては確定だが、
**今回の飛行で実際にどちらを踏んだかは未確認。**

## 次に触る人へ

**先に観測性を足すこと。今回の診断に必要だった状態変数が 1 つもログに出ていない。**

1. **ベンチログ（10Hz）に `Flight_mode` / `LockMode` / `Ftc_mode` / `g_arm_gate_ok` /
   `Chdata[HOVERING]` / `Chdata[FTC]` を追加する。** 6 列で足りる。
2. **`led_control()` の末尾に `else` を足す**（例: 紫の高速点滅）。
   **「どの枝にも当たらなかった」が正常色に化けて見えるのが最悪**で、今回それが起きた疑いが濃い。
3. **`Chdata[HOVERING]` のデッドゾーンを潰す**: `> 350` で HOVERING、それ以外は NORMAL の
   単一しきい値にする。3 位置スイッチなら中央を明示的に扱う。
4. **離陸ゲートのカウントダウンを見えるようにする。** 現状 `Arm_flag==2 && !g_arm_gate_ok` は
   赤 5 Hz 点滅だが、(5) の未処理状態に当たるとその表示すら出ない。
   起動時ヘッダに加えて **`#arm_gate: waiting ...` を 1 秒ごとに printf** するか、
   `g_arm_gate_ok` をベンチログに出す（=1 で解決）。
5. **運用上の注意**: **アームのたびに 2.5 秒はモータが回らない。** 待たずに再アームすると
   カウントダウンがやり直しになり、いつまでも回らない。v7 の `reinit=15` はこの症状そのもの。
- 関連: [`20260910_arm_gate_and_ekf_reinit_ru800.md`](20260910_arm_gate_and_ekf_reinit_ru800.md)、
  [`20260910_ru1200_loop_gain_match_and_flightmode_log_fix.md`](20260910_ru1200_loop_gain_match_and_flightmode_log_fix.md)、
  [`20260831_nmpc_flight_led_yellow.md`](20260831_nmpc_flight_led_yellow.md)
