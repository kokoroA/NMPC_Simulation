# SIL M1: センサ注入の符号・単位を検証し、EKF の追従を確認

- **日付**: 2026-08-25
- **種別**: investigation
- **対象**: `program/sil/host/{sensors.py,rc.py,test_m1_sensors.py}`, `program/sil/shim/shim.cpp`
- **タグ**: #SIL #M1 #EKF #センサ #符号 #地磁気 #校正 #Phi_bias #ヨー #Benzen_new

## 背景 / 症状

M0 でファームはホストで動くようになったが、「ビルドが通り時間が進む」だけで、
**センサの符号・単位が正しいかは未検証**だった。ここが狂うと EKF が静かに破綻し、
「なぜか飛ばない」という形でしか症状が出ない。

## 原因 / 判断根拠（踏んだ罠を全部残す）

### 1. core1 の完了ハンドシェイクが壊れていた（M0 の実装バグ）

`sil_core1_wait_idle()` は「semaphore が待ち状態か」で判定していたが、
`sem_release` 直後は core1 がまだ起きておらず**待ちフラグが立ったまま**。
そのため即座に「完了」と誤判定し、**EKF が一度も走らないまま core0 だけ進んでいた**。

→ カウンタ方式へ変更。`sem_acquire_blocking` の入口で `acquire_entry_count` を増やし、
「acquire に再突入した = 直前の 1 周期が完了した」を完了条件とする。
tick 開始時にスナップショットを取り、release が起きた場合のみカウンタの増加を待つ。

### 2. 実機の printf が stdout を溢れさせる

`control.cpp:958` の `printf("%d\n", Chdata[REDCIRCLE])` が **400 Hz で出力**される。
実機では USB シリアルへ流れるだけだが、ホストでは洪水になる。
→ `sil_set_stdout(0)` を既定にした。**`freopen` は fd 1 ごと差し替えてしまい
ホスト側 (Python) の出力まで消える**ので、`stdout` ポインタだけを退避・差替する方式にした。

### 3. スロットル低の間、実機は姿勢ゼロ点を取り続ける（仕様であってバグではない）

`control.cpp:1319` `if (T_ref / BATTERY_VOLTAGE < Flight_duty)` の中で
`Phi_bias = Phi; Theta_bias = Theta; Psi_bias = Psi;` を毎周期実行している（地上での姿勢トリム）。
このため**スロットルを上げない限り `Phi - Phi_bias` は常に 0**。
最初これを「EKF が動いていない」と誤診した。`Flight_duty = 0.05`、
`T_ref = 0.7 * BATTERY_VOLTAGE * (Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN)` より
**スロットル比 0.072 以上**で姿勢が生きる（モータが回るのは `Disable_duty = 0.1` → 比 0.143 以上）。

### 4. 角速度 0 のまま姿勢だけ動かすと EKF には「ジャイロバイアス」に見える

階段状に姿勢だけ変えると、10° の傾斜に対し 12° までオーバーシュートし、
20 秒かけてようやく 10.5° まで戻る。これは**入力が物理的に矛盾している**ため。
→ テストは `euler_rates_to_body_rates()` で**整合した角速度を伴う滑らかな回転**を与える形にした。
結果、誤差 0.01° で一致した。

### 5. 地磁気はハードアイアン校正を通る

`control.cpp` が `v = zoom*(R@m0 - center)`, `m = normalize(Rᵀ@v)` を掛けるため、
合成値は**校正を逆算して**作る必要がある。`sensors.py` の `_mag_calibration_inverse()`。
往復テストで一致を確認済み。

## 対応

- `host/sensors.py`: 物理状態 → 生値の変換（符号・単位・地磁気校正の逆算）
- `host/rc.py`: `Chdata[18]` 生成と `lock_com()` アーム操作の合成
- `host/test_m1_sensors.py`: M1 検証スイート
- `shim/shim.cpp`: 完了ハンドシェイクをカウンタ方式へ修正
- `api/sil_api.cpp`: `sil_set_stdout()`, `sil_get_debug()`, `sil_get_sched_counts()` 追加

## 検証結果

**M1 全項目 PASS。**

| 項目 | 結果 |
|---|---|
| 水平静止 | phi=+0.0000deg, theta=+0.0003deg |
| Arm 状態遷移 1→3→2 | `lock_com()` ジェスチャで到達 |
| ロール ±10deg | 誤差 ±0.01deg |
| ピッチ ±10deg | 誤差 ±0.01deg |
| 複合 (+8,-6)deg | 誤差 0.00deg |
| ジャイロ p/q/r ±30deg/s | 完全一致（**Z 軸反転を含む符号が正しい**） |

センサ注入の妥当性も確認: ロール20°で `Ay=3.354`(=g·sin20°), `Az=9.215`(=g·cos20°)。

**未検証**: 閉ループ（M2 以降）。ESC 遅れ・振動・実機ノイズは未モデル。

## 次に触る人へ

- ⚠ **ヨー角は推定されていない。** `control.cpp:1256` で
  `Psi = Xn_est_3`（ライントレース用カルマンの「角度ズレ」）に上書きされ、
  地磁気由来の `Psi = atan2(e12, e11)` は**破棄されている**。
  `Kalman_holizontal` は `Flight_mode==LINETRACE && i2c_connect==1` でしか呼ばれない。
  → SIL で psi が 0 のままなのは**正しい**。
  通常飛行は `control.cpp:1350` の `Rref = Psi_ref` により
  **Psi_ref をヨー角速度指令として直接使う**ので、自動旋回・8の字（M5）に影響しない。
- ⚠ **姿勢を動かすときは必ず角速度も整合させること**（`euler_rates_to_body_rates`）。
  怠ると EKF の収束が桁違いに遅くなり、制御性能の評価を誤る。
- ⚠ SIL でテストするときは**スロットルを上げてから**姿勢を見ること（上記 3）。
- `Phi_bias` の累積は 2000〜6000 の 4000 サンプルだが `KALMANWAIT`(6000) で割っている。
  水平なら影響しないが、傾いた地面でアームすると **2/3 に過小評価**される。未修正。
- 関連: [20260825_sil_m0_firmware_on_host.md](20260825_sil_m0_firmware_on_host.md)
