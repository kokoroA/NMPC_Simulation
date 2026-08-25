# 実機ファーム SIL シミュレータ 実装計画書

- **作成日**: 2026-08-25
- **対象**: `Benzen_new`（PID）を第一段階、`Benzen_NMPC` を第二段階
- **目的**: **実機に書き込むのと同一の C/C++ ソース**を PC 上で動かし、
  手動操縦・モード遷移・自動機動を飛ばす前に検証できるようにする

---

## 0. 方針（確定事項）

| 論点 | 決定 |
|---|---|
| センサ模擬の深さ | **関数レベル**。`imu_mag_data_read()` / `get_Altitude()` を差し替える |
| 構成 | **ファーム = 共有ライブラリ(.so) / プラント・可視化・ログ = Python** |
| 対象ファーム | **`Benzen_new`（PID）から。** HAL 境界は NMPC と共通化しておく |
| 操縦入力 | **WebUI 仮想スティック + キーボード** |

**中核となる考え方**: `control.cpp` / `pid.cpp` / `ekf.cpp` / `auto_maneuvers.cpp` は**一行も変更しない**。
変えるのは「その下」だけ。実機とSILで**同じ制御コードが動いている**ことが、この環境の価値の全て。

---

## 1. アーキテクチャ

```
┌──────────────────── ブラウザ ────────────────────┐
│  3D 可視化(three.js)  仮想スティック/キーボード   │
│  リアルタイムグラフ    モードスイッチ             │
└───────────────▲───────────────────┬──────────────┘
                │ WebSocket         │ 操作入力
┌───────────────┴───────────────────▼──────────────┐
│  Python シミュレータホスト (FastAPI)              │
│  ┌────────────┐  ┌──────────┐  ┌──────────────┐  │
│  │ PlantModel │  │NoiseModel│  │ Logger       │  │
│  │(既存資産)  │  │(既存資産)│  │ (Parquet/CSV)│  │
│  └─────▲──────┘  └────┬─────┘  └──────────────┘  │
│        │ duty[6]      │ 物理量→センサ生値         │
│  ┌─────┴──────────────▼─────────────────────┐    │
│  │      ctypes 経由の tick 駆動               │    │
│  └─────────────────▲───────────────────────┘    │
└────────────────────┼─────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────┐
│  libbenzen_sil.so                                 │
│  ┌─────────────────────────────────────────────┐  │
│  │ 【無改変】control.cpp / pid.cpp / ekf.cpp    │  │
│  │           auto_maneuvers.cpp                 │  │
│  └─────────────────────────────────────────────┘  │
│  ┌─────────────────────────────────────────────┐  │
│  │ 【SIL シム】pico-sdk 代替 + センサ/PWM/RC   │  │
│  └─────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────┘
```

---

## 2. HAL 境界（置換対象の確定表）

調査の結果、**差し替えが必要なのは以下だけ**。制御ロジック本体には触れない。

| 実機モジュール | ハード依存 | SIL での置換 |
|---|---|---|
| `sensor.cpp` | LSM9DS1 (SPI0), VL53L1X (I2C) | `imu_mag_data_read()` / `get_Altitude()` を**空実装**にし、Python から生値配列へ直接注入 |
| `pwm.cpp` | PWM 400 Hz IRQ, `set_duty_*` | `set_duty_*` は値を保持するだけ。IRQ は SIL スケジューラが `loop_400Hz()` を呼ぶ形に |
| `radio.cpp` | UART0 100 kbaud, `Chdata[18]` | UART 初期化を no-op 化し、`Chdata[]` へ WebUI 由来の値を注入 |
| `board_led` / `rgbled` / `WS2812` | CYW43, PIO | すべて no-op（状態だけ記録して WebUI へ流す） |
| `i2c.cpp` | I2C | no-op |
| `wifi_telemetry.cpp` | CYW43 + lwIP | **ホスト UDP ソケットで同一プロトコルを実装**（後述） |
| pico-sdk 全般 | `time_us_32`, `sleep_ms`, `sem_*`, `multicore_*` | ホスト実装（仮想時間ベース） |

### 2.1 センサ注入の契約（**最重要・符号に注意**）

`sensor_read()` は `imu_mag_data_read()` の結果に**符号反転を掛けてから**制御量にしている:

```c
Ax = -acceleration_mg[0] * GRAV * 0.001;   //  X 反転
Ay = -acceleration_mg[1] * GRAV * 0.001;   //  Y 反転
Az =  acceleration_mg[2] * GRAV * 0.001;   //  Z そのまま
Wp =  angular_rate_mdps[0] * π/180/1000;   //  そのまま
Wq =  angular_rate_mdps[1] * π/180/1000;   //  そのまま
Wr = -angular_rate_mdps[2] * π/180/1000;   //  Z 反転
Mx0 = -magnetic_field_mgauss[0];
My0 =  magnetic_field_mgauss[1];
Mz0 = -magnetic_field_mgauss[2];
```

したがって Python 側は**機体座標系の物理量に逆変換を掛けて「センサ生値」を作る**必要がある。
単位も生値側に合わせること:

| 配列 | 単位 | Python 側の変換 |
|---|---|---|
| `acceleration_mg[3]` | **milli-g** | `[-ax, -ay, az] / GRAV * 1000` |
| `angular_rate_mdps[3]` | **milli-deg/s** | `[p, q, -r] * 180/π * 1000` |
| `magnetic_field_mgauss[3]` | **milli-gauss** | `[-mx, my, -mz]` |
| `distance` (VL53L1X) | **mm**（`uint16_t`） | 高度 [m] × 1000（要クリップ） |

> ⚠ ここを間違えると EKF が静かに破綻し、原因究明に時間を溶かす。**M1 の最優先検証項目**。

### 2.2 時間統制の再現

実機のタイミング構造:

- `loop_400Hz()` = PWM IRQ ハンドラ（core0, 400 Hz）。`pico_copter.hpp` で `MAINLOOP` として定義。
- `angle_control()` = core1 の `while(1)`。`sem_acquire_blocking(&sem)` で待ち、
  `loop_400Hz()` 側の `sem_release(&sem)` で 100 Hz 起動される。

**SIL での再現方針**: `angle_control()` は無限ループなので「1 回だけ呼ぶ」ことができない。
ファームを改変せずに済ませるため、**core1 をホストスレッドとして実際に起動**し、
`semaphore_t` をホストの condition variable で実装する。

さらに **決定性を確保するため、シムに完了ハンドシェイクを追加**する
（`sem_release` 後、core1 が次の `sem_acquire` に戻るまでホストをブロックする）。
これにより「400 Hz を 4 回 → 100 Hz を 1 回」が毎ステップ厳密に揃い、
同じ入力からは常に同じログが出る。

### 2.3 モータ出力の契約

`set_duty_*(float duty)` は `[0.01, 0.97]` にクリップしてから PWM カウントへ写す。
**SIL はクリップ後の正規化 duty（0〜1）を保持**し、Python のプラントへ渡す。
`DUTYMIN 1375 / DUTYMAX 2375` の PWM カウント換算は SIL では不要
（ただし ESC 特性を再現する段階では、この非線形性を Python 側で掛ける）。

---

## 3. ディレクトリ構成（新規）

```
program/sil/
├── CMakeLists.txt              # libbenzen_sil.so をビルド
├── shim/                       # pico-sdk 代替
│   ├── pico/stdlib.h           # time_us_32, sleep_ms, ...
│   ├── pico/multicore.h        # multicore_launch_core1 → std::thread
│   ├── pico/sync.h             # semaphore_t → condition_variable
│   ├── hardware/{pwm,irq,spi,uart,i2c,clocks}.h
│   └── shim.cpp                # 実体 + 仮想時間
├── hal_sil/                    # センサ・アクチュエータの SIL 実装
│   ├── sensor_sil.cpp          # imu_mag_data_read / get_Altitude を空実装化
│   ├── pwm_sil.cpp             # set_duty_* → 保持
│   └── radio_sil.cpp           # Chdata[] 注入
├── api/
│   ├── sil_api.h               # Python から叩く C ABI
│   └── sil_api.cpp
├── host/                       # Python 側
│   ├── sil_bridge.py           # ctypes バインディング
│   ├── plant.py                # NMPC_simulation の PlantModel を再利用
│   ├── sensors.py              # 物理量 → センサ生値（§2.1 の変換）
│   ├── logger.py               # Parquet/CSV
│   ├── server.py               # FastAPI + WebSocket
│   └── udp_bridge.py           # 既存 PidTelPkt 互換の UDP 送出
└── web/
    ├── index.html
    ├── viz3d.js                # three.js
    ├── input.js                # 仮想スティック + キーボード
    └── charts.js
```

### C ABI（`sil_api.h`）

```c
void  sil_init(void);
void  sil_set_imu(const float acc_mg[3], const float gyr_mdps[3], const float mag_mgauss[3]);
void  sil_set_range_mm(uint16_t mm);
void  sil_set_rc(const uint16_t ch[18]);
void  sil_tick_400hz(void);        // loop_400Hz を 1 回。内部で core1 同期も完了させる
void  sil_get_duty(float duty[6]); // [FR, FL, MR, ML, RR, RL]
void  sil_get_telemetry(PidTelPkt *out);
void  sil_advance_time_us(uint32_t us);
```

---

## 4. フェーズ分割

| フェーズ | 目標 | 完了条件（検証可能な形で） |
|---|---|---|
| **M0** | ビルドを通す | `libbenzen_sil.so` が生成され、`sil_init()` がクラッシュしない。Eigen の include パスを解決 |
| **M1** | センサ注入 → EKF が正しく動く | **静止姿勢**を注入して `Phi/Theta/Psi ≈ 0`。既知の傾きを注入して EKF 出力が一致（§2.1 の符号検証） |
| **M2** | 閉ループが成立 | ホバー duty を与えて機体が落ちない。ステップ角度指令に対し PID が収束する |
| **M3** | ログ + オフライン描画 | 全状態を Parquet に記録し、既存 matplotlib で時系列描画。1 分飛行分が欠落なく取れる |
| **M4** | WebUI（可視化 + 操縦） | 3D 姿勢がリアルタイム追従。キーボード/仮想スティックで手動ホバリング可能 |
| **M5** | 自動機動の検証 | `auto_maneuvers` の水平旋回・8の字・上昇旋回を SIL で飛ばし、軌道形状を確認 |
| **M6**（後続） | NMPC 対応 | 同じ HAL で `Benzen_NMPC` をビルド。周期 10 ms 超過の最適化検証に使う |

**M5 が本命**です。`20260824_benzen_new_auto_maneuvers.md` / `20260825_benzen_new_pitch_sign_figure8.md`
の「実機軌道は未検証」を、墜落リスクなしで潰すのがこの環境の第一の投資対効果。

---

## 5. ログ設計

**方針: 取れるものは全部取る。間引きは後段で行う。**

| 系統 | レート | 内容 |
|---|---|---|
| 真値（プラント） | 400 Hz | 位置・速度・姿勢（quat/Euler）・角速度・各モータ推力 |
| センサ生値 | 400 Hz | 注入した `acc_mg` / `gyr_mdps` / `mag_mgauss` / `distance`（ノイズ込み） |
| ファーム内部 | 100 Hz | `PidTelPkt` 全フィールド（`Phi/Theta/Psi`, `*_ref`, `P/Q/R`, `*_com`, duty×6, `T_ref`, `loop_us`） |
| PID 内部 | 100 Hz | 各軸の P/I/D 項の内訳（`pid.cpp` にフックを 1 本追加） |
| 操作入力 | 発生時 | `Chdata[18]` の変化、モード遷移イベント |

形式は **Parquet**（列指向・圧縮・pandas 直読み）を主、CSV をオプション。
`run_YYYYMMDD_HHMMSS/` にメタ情報（ファームの git hash、機体パラメータ、乱数シード）を同梱し、
**再現可能性を担保**する。

---

## 6. WebUI

### 3D 可視化
three.js。ヘキサ X 配置の簡易モデル（アーム 6 本 + プロペラディスク）。
モータ duty をプロペラの色/透明度に反映。姿勢は quaternion で直接駆動。
軌跡はトレイルライン、参照姿勢はゴースト表示。

### 操縦入力（キーボード + 仮想スティック）

| キー | 機能 | 対応 ch |
|---|---|---|
| `W` / `S` | スロットル 増/減 | CH3 |
| `↑` / `↓` | エレベータ（ピッチ） | CH2 |
| `←` / `→` | エルロン（ロール） | CH1 |
| `A` / `D` | ラダー（ヨー） | CH4 |
| `Space` | Arm / Disarm | — |
| `1`〜`5` | 飛行モード切替 | MODE_SW |
| `R` | シミュレーション リセット |  — |

キーはレート入力（押している間だけ増減、離すとセンタリング）とし、
生成した値を `radio.hpp` のレンジ（`CH1MIN 368` 〜 `CH1MAX 1680` 等）に写して `Chdata[]` へ注入する。
仮想スティックは同じ内部値を操作する別 UI として実装（キーボードと排他ではなく合流させる）。

> モード遷移そのものはユーザー側で実装される想定のため、SIL は
> **「`Chdata[]` に正しい値を届ける」ところまで**を責務とする。

---

## 7. 既存 UDP プロトコルの活用

`wifi_telemetry.hpp` に `PidTelPkt`（`magic 0xBEEF` / `ver 1` / CMD 5000 / TEL 5001）が既に定義済み。
**SIL も同じパケットを同じポートに投げる**ことで、以下が無料で手に入る:

- `tools/pid_udp_client.py` が**実機・SIL 両対応**になる（デバッグ手順が一本化）
- WebUI を後で実機に繋ぎ替えられる（SIL で作った可視化がそのまま実機テレメトリビューアになる）

**この互換性は意図的に維持すること。** パケット定義を SIL 都合で拡張したくなったら、
`ver` を上げて別型を足す（v1 を壊さない）。

---

## 8. リスクと注意点

| リスク | 対策 |
|---|---|
| **センサ符号・単位の取り違え**（最大のリスク） | M1 を独立したマイルストンにし、既知姿勢での EKF 一致を先に確認する |
| `control.cpp` が 2,167 行あり未使用パス（LINETRACE, I2C, servo）が多い | まず該当機能を無効化してビルドを通し、後から必要分だけ有効化 |
| Eigen の include パスが別プロジェクトの絶対パス | ホストビルドではシステム Eigen を使い、パスを CMake 変数化 |
| プラントのモータ順とファームの `set_duty_*` の対応 | `[FR, FL, MR, ML, RR, RL]`。`plan.md` の対応表と突き合わせて M2 で検証 |
| SIL で動いても実機で飛ぶとは限らない | ESC 遅れ・電圧降下・振動を段階的に足す。**SIL を「飛ぶ保証」でなく「落ちる原因を先に見つける道具」と位置づける** |

## 9. 計算量・計算速度・計算資源の再現

**Benzen_NMPC では制御周期 10 ms 超過が最大の課題**であり、SIL でこれを試せることが必須要件。
ただし「ホストで測った時間」は実機時間ではないので、3 つに分けて扱う。

### 9-1. 計算資源（メモリ）— **厳密に取れる。実装済み**

SIL はホスト上で動くため、SIL プロセスのメモリ消費は実機と無関係。
実機資源は**実機向けにクロスビルドした ELF を解析する**のが唯一正しい方法。
`program/sil/host/resource_report.py` が `arm-none-eabi-size` / `nm` でこれを行う。

実測値（2026-08-25 時点）:

| | SRAM | 内訳の上位 |
|---|---|---|
| `Benzen_new` | 236,760 B / 520 KB（**44.5%**） | `Logdata` 191,560 B |
| `Benzen_NMPC` | 402,456 B / 520 KB（**75.6%**） | `Logdata` 191,560 B, **`g_nmpc_ctx` 162,016 B** |

**`g_nmpc_ctx` が 158 KB** で、これが `N` に直接比例して増減する本体。
`Logdata`（187 KB）は制御に不要なログバッファであり、**メモリが逼迫したらここが最初の削減候補**。

### 9-2. 計算量（アルゴリズム的仕事量）— アーキテクチャ非依存

SIL ビルドに**ワークカウンタ**を仕込み、1 周期あたりの以下を数える:

- `full_rebuild_sx_su` の実行回数と行列サイズ
- TR1 / SR2 の低ランク更新回数
- Cholesky 更新の回数
- **ADMM の反復回数**（飽和時に増えるため、最悪ケースの指標になる）
- 各フェーズの浮動小数点演算数

これは CPU に依存しないので、**`N` を変えたときの仕事量の変化率**を正確に出せる。
「`N=17→13` で仕事量が何 % 減るか」はこれで答えられる。

### 9-3. 計算時間（実機の壁時計時間）— **較正モデルで推定する**

ホストは x86_64 の GHz 級、実機は Cortex-M33 150 MHz・単精度。単純な倍率では換算できない。
そこで**実測値に対して較正**する:

1. 実機の既存ベンチログから `iter`（ソルバ本体）と `loop`（周期全体）を取る
   → 現状 `N=17` で **ソルバ 8.7 ms / 周期全体 10.6 ms**
2. 同条件の SIL で 9-2 のワークカウンタを取る
3. `t_hw ≈ Σ_phase c_phase × work_phase` の係数 `c_phase` をフィットする
4. **`N` を変えた実機実測を数点取って検証する**（外挿の妥当性確認）

> ⚠ **正直に言うべき限界**: この方法が信頼できるのは**相対的な変化の予測**まで。
> キャッシュ挙動・メモリ帯域・分岐予測は再現していないので、絶対時間の予測には誤差が残る。
> 「`N` を下げれば 10 ms に入るか」の判断材料としては使えるが、
> **最終確認は必ず実機のベンチログで行うこと。**

### 9-4. さらに高忠実度が要る場合（将来オプション）

実際の ARM バイナリをサイクル近似エミュレータ（Renode 等）で走らせる方法がある。
命令レベルで正確になるが、RP2350 の周辺モデルを自作する必要があり工数が大きい。
9-3 の較正モデルで判断がつかなくなった時点で検討する。

## 10. 未決事項（実装中に決めればよいもの）

- ESC の一次遅れ・推力非線形をどこまで入れるか（M2 では線形、M5 で同定値を反映）
- 風外乱モデル（`NMPC_simulation` の外乱シナリオを流用できるか要確認）
- リアルタイム実行と高速バッチ実行の切替（同じコアで倍速係数を持たせる想定）
