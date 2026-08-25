# SIL M0: 実機ファームをホストで共有ライブラリとしてビルド

- **日付**: 2026-08-25
- **種別**: design
- **対象**: `program/sil/`（新設）, `docs/sil_simulator_plan.md`
- **タグ**: #SIL #Benzen_new #Benzen_NMPC #shim #pico-sdk #ctypes #計算資源 #ELF

## 背景 / 症状

実機に書き込む C/C++ をシミュレーションで試す環境が無く、
自動旋回・8の字・上昇旋回（`20260824_benzen_new_auto_maneuvers.md`）は
**実機軌道が未検証のまま**残っていた。墜落リスクを負わずに検証する手段が必要。

## 原因 / 判断根拠

- `control.cpp` は 2,167 行あり、PWM IRQ と `multicore_launch_core1` に直結している。
  ロジックだけ切り出す改造は差分が大きく、**実機と SIL でコードが乖離する**ため却下。
- 実際に使われている pico-sdk API は **35 関数程度**しかなく、shim で肩代わり可能と判明。
- `pwm.cpp` の `set_duty_*` は `pwm_set_chan_level` を呼ぶだけなので、
  **本物をそのまま使い shim 側で level を読み戻す**方がクリップ処理まで忠実に再現できる。

## 対応

`program/sil/` を新設。制御ロジックは**一行も変更していない**。

- `shim/`: pico-sdk 代替（`pico/`, `hardware/`, TinyUSB, 生成 PIO ヘッダ）。
  include パスで本物より先に解決させる。
- `hal_sil/sensor_sil.cpp`: `sensor.cpp` のみ差し替え。関数レベルで生値を注入。
- `api/sil_api.{h,cpp}`: Python 用 C ABI。実機 `main()` の初期化順を再現。
- `host/sil_bridge.py`: ctypes バインディング。
- `host/resource_report.py`: 実機 ELF から Flash/SRAM を厳密算出。

### 設計上の判断

- **仮想時間**: `time_us_32()` はホストが明示的に進める。`sleep_ms` はホストを待たせない。
  これで「同じ入力 → 同じログ」の決定性が得られる。
- **core1 は実スレッド**: `angle_control()` は `while(1)` なので 1 回だけ呼べない。
  ファーム無改造で済ませるため `semaphore_t` を condition variable で実装し、
  **完了ハンドシェイク**（core1 が次の acquire に戻るまで待つ）で決定性を確保した。
- **`EIGEN_NO_MALLOC` をホストでも有効化**: 実機と同じ制約を課し、動的確保を早期に検出する。

### 却下した案

- **ロジックを別ライブラリに切り出す**: 実機コードと乖離するため。SIL の存在価値が消える。
- **QEMU/エミュレータで ARM バイナリを走らせる**: RP2350 の周辺モデルが無く工数過大。
  計算時間の忠実度が本当に足りなくなった時点で再検討する（計画書 §9-4）。

## 検証結果

- `cmake --build build` で `libbenzen_sil.dylib`（172,848 B）の生成に成功。
- 疎通テスト: `sil_init()` 成功、`Arm_flag=1`。
  8 tick で仮想時間が **正確に 20,000 us 進行**（400Hz × 2500us）。
  `Arm_flag==1` のため `motor_stop()` が効き、duty が全て下限 **0.01**（`set_duty_*` のクリップ値）。
  → **実機と同じ振る舞いをしている。**

### 実機 ELF の資源実測（`resource_report.py`）

| ファーム | SRAM | 最大シンボル |
|---|---|---|
| `Benzen_new` | 236,760 B / 520 KB（44.5%） | `Logdata` 191,560 B |
| `Benzen_NMPC` | **402,456 B / 520 KB（75.6%）** | `Logdata` 191,560 B, **`g_nmpc_ctx` 162,016 B** |

**未検証**: 閉ループ挙動（M2 以降）。センサ符号・単位の正しさ（M1）はまだ確認していない。
現時点で確認したのは「ビルドが通り、初期化が走り、時間が進む」ところまで。

## 次に触る人へ

- ⚠ **`pwm.cpp` のスライス名は実際のモータ位置と一致していない。**
  `Slice_num_front`(7)=FR/RR, `Slice_num_middle`(2)=FL/RL, `Slice_num_rear`(1)=MR/ML。
  `api/sil_api.cpp` の対応表を正とすること。名前を信じると出力が入れ替わる。
- `Temp` / `Temp2` / `IntPol` は `sensor.cpp` ではなく `control.cpp` が定義している。
  `sensor_sil.cpp` で定義すると重複シンボルになる。
- ファーム関数は **C++ リンケージ**。`sil_api.cpp` で `extern "C"` に入れるとリンクできない。
- **`Logdata` が 187 KB** ある。NMPC で SRAM が逼迫したら最初の削減候補。
  制御に不要なログバッファであり、削れば `N` を伸ばせる。
- 次は **M1（センサ符号・単位の検証）**。ここを飛ばすと EKF が静かに破綻する。
- 関連: [20260824_benzen_new_auto_maneuvers.md](20260824_benzen_new_auto_maneuvers.md),
  [20260825_benzen_new_pico2w.md](20260825_benzen_new_pico2w.md)
