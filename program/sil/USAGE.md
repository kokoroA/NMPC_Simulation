# SIL 使い方ガイド

実機に書き込むのと**同一の C/C++ ソース**を PC 上で飛ばすための環境です。
`control.cpp` / `pid.cpp` / `ekf.cpp` / `auto_maneuvers.cpp` は**一行も変更していません**。
変えているのは pico-sdk とセンサドライバの「下回り」だけです。

- 設計の背景: [`docs/sil_simulator_plan.md`](../../docs/sil_simulator_plan.md)
- 判断の経緯: `docs/ai/decisions/2026082*_sil_*.md`

---

## 0. 前提: Python 環境

**`program/NMPC_simulation/.venv` を使います。** システムの `python3` には numpy が入っておらず動きません。

```bash
cd program/sil
PY=../NMPC_simulation/.venv/bin/python
$PY -c "import numpy, matplotlib; print(numpy.__version__, matplotlib.__version__)"
# -> 2.0.2 3.9.4
```

以降このガイドでは `$PY` をこの Python として書きます。毎回打つのが面倒なら:

```bash
alias silpy="$PWD/../NMPC_simulation/.venv/bin/python"
```

### この venv を使う理由

| | 理由 |
|---|---|
| numpy | プラント（機体ダイナミクス）とセンサ変換に必須 |
| matplotlib | ログの描画に必須。日本語表示は `Hiragino Sans` を自動選択 |
| 既存資産と同じ | `NMPC_simulation` のシミュレータと同じ環境なので、将来資産を流用しやすい |

**追加インストールは不要です。** SIL は FastAPI も three.js も pandas も使いません
（依存を増やさない方針。WebUI は標準ライブラリ + Server-Sent Events、3D は Canvas 2D 自前描画）。

### venv が壊れた／無いとき

```bash
cd ../NMPC_simulation
python3.12 -m venv .venv          # 3.9 以上なら可
.venv/bin/pip install numpy matplotlib
```

> `PyQt5` / `PySide2` は `NMPC_simulation` 側の GUI 用で、SIL は使いません。

---

## 1. ビルド

```bash
cd program/sil
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

生成物は `build/libbenzen_sil.dylib`（Linux では `.so`）。

| オプション | 用途 |
|---|---|
| `-DEIGEN_DIR=<path>` | Eigen が標準の場所に無いとき |
| `-DFW_DIR=<path>` | ファームの場所を変えたとき（既定は `../pico/pico-sdk2/pico-sdk/Benzen_new`） |

### 疎通確認

```bash
$PY -c "import sys;sys.path.insert(0,'host');from sil_bridge import Sil;s=Sil();[s.tick() for _ in range(8)];print(s.time_us,'us',s.duty())"
# -> 4020000 us [0.01, 0.01, ...]   ← 8 tick で 20,000us、Arm_flag=1 なので duty は下限
```

---

## 2. WebUI で手動操縦する

```bash
$PY host/webserver.py --port 8420
```

ブラウザで `http://127.0.0.1:8420/` を開きます。
起動時にウォームアップ 6000 tick（実時間 15 秒相当）を自動で早送りするので、すぐ操作できます。

### 手順

1. `Space`（または「アーム」ボタン）でアーム → 状態が **「飛行中」** になる
2. `W` を押してスロットルを上げる
   - **0.045 未満** … モータ停止
   - **0.128 付近** … ホバー（`T_max=2.04 N` の線形仮定での計算値）
   - **0.428 以上** … duty が上限に張り付く
3. 方向舵で姿勢を操作

### キー割り当て

| 操作 | キー | 備考 |
|---|---|---|
| スロットル | `W` / `S` | 保持される（離しても戻らない） |
| エルロン（ロール） | `←` / `→` | 離すとセンタリング |
| エレベータ（ピッチ） | `↑` / `↓` | 離すとセンタリング |
| ラダー（ヨー角速度） | `A` / `D` | 離すとセンタリング |
| アーム | `Space` | |
| 姿勢リセット | `R` | プラントの姿勢だけ戻す |
| 一時停止 | `P` | |

タップでも押しっぱなしでも効きます。マウス／タッチの仮想スティックも同じ入力に合流します。

### 画面の見方

| 要素 | 内容 |
|---|---|
| 3D ビュー | ドラッグで回転、ホイールでズーム。プロペラの色 = duty（青→赤）、赤い線 = 機首 |
| 状態テーブル | 「真値 / 推定」の並びで、プラントの真値と EKF 推定を比較できる |
| モータ duty | 6 本のバー。姿勢補正で差がつく様子が見える |
| 時系列 | 直近 20 秒。実線 = 実測、破線 = 目標 |

### オプション

```bash
$PY host/webserver.py --port 8420 --speed 2.0   # 2 倍速
```

---

## 3. バッチ実行（ログと図）

手で飛ばさず、決められたシーケンスを走らせてログを取ります。

```bash
$PY host/run_m3_demo.py --csv    # 手動操作相当: ステップ・複合舵・突風・スロットル掃引
$PY host/run_m5_demo.py          # 自動機動 3 種: 水平旋回 → 8の字 → 上昇旋回
```

`runs/run_YYYYMMDD_HHMMSS/` に出力されます:

| ファイル | 内容 |
|---|---|
| `data.npz` | **全 44 列・400 Hz** の生ログ（float32 圧縮。CSV の約 1/8） |
| `meta.json` | ファームのソースハッシュ・機体パラメータ・ファーム定数（**再現用**） |
| `overview.png` | 姿勢 / EKF誤差 / 角速度 / duty / 推力の時系列 |
| `saturation.png` | duty の上下限張り付き率 |
| `trajectory.png` | 軌跡（開ループ・参考値） |
| `data.csv` | `--csv` 指定時のみ |

### 既存ログの再描画・読み込み

```bash
$PY host/plots.py                      # 最新のラン
$PY host/plots.py runs/run_2026...     # 指定したラン
```

```python
import sys; sys.path.insert(0, "host")
import logger
data, meta = logger.load(logger.latest_run())
print(data["phi_true"].shape, meta["firmware_source_sha256"])
```

---

## 4. 検証スイート

**ファームを変更したら必ず回してデグレを確認します。**

```bash
$PY host/test_m1_sensors.py         # センサ符号・単位と EKF の追従
$PY host/test_m2_closed_loop.py     # 姿勢閉ループ（ステップ・突風）
$PY host/test_m3_logging.py         # ログの完全性と描画
$PY host/test_m4_webui.py           # WebUI（別プロセスでサーバを起動）
$PY host/test_m5_auto_maneuvers.py  # 自動機動（旋回・8の字・上昇旋回）
```

全て終了コード 0 で PASS、1 で FAIL です。

---

## 5. 実機ファームを変更したときの反映

SIL は `Benzen_new/` の**ソースを直接コンパイル**しています（コピーは持ちません）。

| 変更内容 | 必要な作業 |
|---|---|
| `.cpp` / `.hpp` の**中身**を編集 | **`cmake --build build` だけ** |
| **新しい .cpp を追加** | `CMakeLists.txt` の `FW_SOURCES` に追記 |
| **新しい pico-sdk API** を使用 | `shim/` に関数を追加（未定義シンボルでリンクエラーになる） |
| `control.cpp` の**定数**を変更 | `host/firmware_consts.py` も直す（写しなので） |

反映されたかはソース内容ハッシュで確認できます（**ファームは git 管理外**のため）:

```bash
$PY -c "import sys;sys.path.insert(0,'host');from logger import _firmware_source_hash;from pathlib import Path;print(_firmware_source_hash(Path('../pico/pico-sdk2/pico-sdk/Benzen_new')))"
```

> ⚠ `control.cpp` は **CRLF** です。テキストモードで書き戻すと全行が差分化します。
> **編集後は必ずバックアップとの全差分を確認してください**（過去に中断された編集で
> ミキサ 55 行が消えた事故があります）。

---

## 6. ゲインのチューニング検討

実機の `control_init()` を書き換えずに、SIL 上だけで PID を振れます。

```bash
$PY host/tune_roll_ti.py          # ロール Ti を既定候補で掃引
$PY host/tune_roll_ti.py 5 3 2    # 値を指定
```

コードからは:

```python
cl.sil.set_pid(cl.sil.PID_PHI, kp=1.0, ti=5.0, td=0.0)
# which: PID_PHI / PID_THETA / PID_PSI / PID_P / PID_Q / PID_R
```

> ⚠ **SIL で変えても実機には反映されません。** 良い値が決まったら
> `control.cpp:699-701` を手で直してください。

> ⚠ `Ti` は**積分時間 [s]** で、**大きいほど積分が弱い**。角度ループは `h=0.01s` なので、
> `Ti` が maneuver の時間スケール（秒オーダー）より桁違いに大きいと積分は効きません。
> `100000` も `10000` も「積分なし」と同じです。

---

## 7. 設定を変える場所

| 変えたいもの | 場所 |
|---|---|
| 機体パラメータ（`T_max`・慣性・空力・モータ遅れ・推力カーブ） | `host/plant.py` の `PlantParams` |
| 実機定数の写し（`Disable_duty` 等） | `host/firmware_consts.py` |
| 地磁気の伏角 | `host/sensors.py` の `SensorModel(mag_ned=...)` |
| PID ゲイン（SIL 上のみ） | `cl.sil.set_pid(...)` |
| 飛行シーケンス | `host/run_m3_demo.py` / `run_m5_demo.py` を複製して編集 |

### 自作スクリプトの雛形

```python
import sys; sys.path.insert(0, "host")
import numpy as np, rc
from sim import ClosedLoop

DEG = np.pi / 180
cl = ClosedLoop()
cl.arm()                                        # ウォームアップ + lock_com ジェスチャ
cl.set_rc(rc.with_throttle(rc.neutral(), 0.13)) # ホバー付近
cl.run_seconds(2.0)

cl.set_rc(rc.with_roll_ref(rc.with_throttle(rc.neutral(), 0.13), 10 * DEG))
cl.run_seconds(4.0)
print(cl.plant.phi / DEG)                       # 真値
print(cl.sil.state()["phi"] / DEG)              # EKF 推定

cl.sil.set_flight_mode(cl.sil.MODE_AUTO_FIGURE_EIGHT)   # 自動機動
cl.run_seconds(30.0)

cl.save_run(note="自分の試験")                    # runs/ に保存
```

---

## 8. トラブルシューティング

| 症状 | 原因と対処 |
|---|---|
| `ModuleNotFoundError: numpy` | システムの python3 を使っている。`../NMPC_simulation/.venv/bin/python` を使う |
| `RuntimeError: Sil は 1 プロセスに 1 つだけ` | ファームがグローバル状態を持つため。**条件を変えるならプロセスを分ける**（`tune_roll_ti.py` が実装例） |
| ビルドで `undefined symbol` | ファームが新しい pico-sdk API を使い始めた。`shim/` に足す |
| `Eigen が見つかりません` | `-DEIGEN_DIR=<path>` を渡す |
| 図の文字が豆腐 | 日本語フォントが無い環境。警告が出るが処理は続行する |
| モータが回らない | スロットル比 0.045 未満。`common_duty >= Disable_duty(0.1)` が必要 |
| 姿勢が目標に追従しない | スロットルが低すぎる。`common_duty >= Flight_duty(0.05)` 未満だと毎周期 `Phi_bias = Phi` で姿勢ゼロ点を取り直す |
| 姿勢がいつまでも収束しない | **プラントの姿勢を直接書き換えた**可能性。外乱は `cl.disturb_rates()` で角速度に与える |
| 高度が発散する | 仕様。位置・速度は**開ループ積分**で制御にフィードバックしていない。`T_max` 較正前は特に大きく出る |

---

## 9. 既知の制約（結論を出す前に読むこと）

- **`T_max = 2.04 N` は未検証の較正値。** 実機のホバー duty を測って更新しないと、
  自動機動の軌道がまるごとずれます。`thrust_exponent` も併せて決める必要があり、
  **duty-推力を 2 点以上**測らないと確定しません。
- **並進ドラッグを入れていません。** 上昇旋回の上昇量は過大に出ます。相対的な傾向の確認まで。
- **ESC 一次遅れ・電圧降下・機体振動・センサノイズは未モデル**（`motor_tau = 0`）。
  実機は SIL より必ず悪くなります。**保守側から入ってください。**
- **ヨー角は推定されていません。** `control.cpp:1256` で `Psi = Xn_est_3`
  （ライントレース用カルマンの角度ズレ）に上書きされます。SIL で `psi` が 0 なのは正しい挙動です。
  通常飛行は `Rref = Psi_ref` でヨー**角速度**指令として使うため、自動機動には影響しません。
- **センサはレジスタレベルでは模擬していません。** ドライバの設定ミス（ODR・レンジ・スケール）は
  SIL では再現できません。
- SIL は**「飛ぶ保証」ではなく「落ちる原因を先に見つける道具」**です。
  最終確認は必ず実機で行ってください。
