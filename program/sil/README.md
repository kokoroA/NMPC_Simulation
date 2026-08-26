# SIL — 実機ファームをホストで動かすシミュレーション基盤

実機に書き込むのと**同一の C/C++ ソース**を PC 上で動かす。
`control.cpp` / `pid.cpp` / `ekf.cpp` / `auto_maneuvers.cpp` は一行も変更しない。

計画書: [`docs/sil_simulator_plan.md`](../../docs/sil_simulator_plan.md)

## 実機ファームを変更したときの反映手順

SIL は `Benzen_new/` の **ソースを直接コンパイル**している（コピーは持たない）。
したがって:

| 変更内容 | 必要な作業 |
|---|---|
| `control.cpp` / `pid.cpp` / `ekf.cpp` / `auto_maneuvers.cpp` の中身を編集 | **`cmake --build build` だけ** |
| **新しい .cpp を追加**した | `CMakeLists.txt` の `FW_SOURCES` に足す |
| **新しい pico-sdk の API** を使い始めた | `shim/` に該当関数を足す（未定義シンボルでリンクエラーになる） |
| `control.cpp` の**定数**（`Disable_duty` 等）を変えた | あわせて `host/firmware_consts.py` も直す（写しなので） |

反映されたかは**ソース内容ハッシュ**で確認できる（ファームは git 管理外なので）:

```bash
../NMPC_simulation/.venv/bin/python -c "import sys;sys.path.insert(0,'host');from logger import _firmware_source_hash;from pathlib import Path;print(_firmware_source_hash(Path('../pico/pico-sdk2/pico-sdk/Benzen_new')))"
```

⚠ `control.cpp` は **CRLF**。編集はバイナリモードで行い、**編集後は必ず全差分を確認**すること
（過去に中断された編集でミキサ 55 行が消えた）。

## ビルド

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Eigen が標準の場所に無い場合は `-DEIGEN_DIR=<path>` を渡す。
ファームの場所を変えたい場合は `-DFW_DIR=<path>`。

## 疎通確認

```bash
python3 -c "import sys; sys.path.insert(0,'host'); from sil_bridge import Sil; s=Sil(); [s.tick() for _ in range(8)]; print(s.time_us, s.duty())"
```

## デモ飛行とログ

```bash
../NMPC_simulation/.venv/bin/python host/run_m3_demo.py --csv    # 手動操作のデモ
../NMPC_simulation/.venv/bin/python host/run_m5_demo.py          # 自動機動 3 種
```

`runs/run_YYYYMMDD_HHMMSS/` に保存される:

| ファイル | 中身 |
|---|---|
| `data.npz` | 全 44 列・400 Hz の生ログ（float32 圧縮。CSV の 1/8） |
| `meta.json` | ファームの git hash・機体パラメータ・ファーム定数（再現用） |
| `overview.png` | 姿勢/EKF誤差/角速度/duty/推力の時系列 |
| `saturation.png` | duty の上下限張り付き率 |
| `trajectory.png` | 軌跡（開ループ・参考） |

既存ログの再描画:

```bash
../NMPC_simulation/.venv/bin/python host/plots.py            # 最新
../NMPC_simulation/.venv/bin/python host/plots.py runs/run_...
```

## WebUI（M4）

```bash
../NMPC_simulation/.venv/bin/python host/webserver.py --port 8420
```

ブラウザで `http://127.0.0.1:8420/` を開く。起動時にウォームアップ(6000 tick)を
自動で早送りするので、すぐ `Space` でアームできる。

**依存ゼロ**で実装している（FastAPI も three.js も使わない）:
標準ライブラリの `http.server` + Server-Sent Events、3D は Canvas 2D に自前投影。
オフラインでも動く。

| 操作 | キー |
|---|---|
| スロットル | `W` / `S`（タップでも押しっぱなしでも可） |
| エルロン | `←` / `→` |
| エレベータ | `↑` / `↓` |
| ラダー（ヨー角速度） | `A` / `D` |
| アーム | `Space` |
| 姿勢リセット | `R` |
| 一時停止 | `P` |

マウス／タッチの仮想スティックも同じ入力に合流する。3D ビューはドラッグで回転、
ホイールでズーム。`--speed` で実時間倍率を変えられる。

## ゲインのチューニング検討

実機の `control_init()` を書き換えずに、SIL 上だけで PID ゲインを振れる。

```bash
../NMPC_simulation/.venv/bin/python host/tune_roll_ti.py          # ロール Ti 掃引
../NMPC_simulation/.venv/bin/python host/tune_roll_ti.py 5 3 2    # 値を指定
```

コードからは:

```python
cl.sil.set_pid(cl.sil.PID_PHI, kp=1.0, ti=5.0, td=0.0)
```

⚠ **SIL で変えても実機には反映されない。** 良い値が見つかったら
`control.cpp:699-701` を手で直すこと。

## 検証スイート

```bash
../NMPC_simulation/.venv/bin/python host/test_m1_sensors.py       # センサ符号・単位と EKF
../NMPC_simulation/.venv/bin/python host/test_m2_closed_loop.py   # 姿勢閉ループ
../NMPC_simulation/.venv/bin/python host/test_m3_logging.py       # ログと描画
../NMPC_simulation/.venv/bin/python host/test_m4_webui.py         # WebUI（別プロセスで起動）
../NMPC_simulation/.venv/bin/python host/test_m5_auto_maneuvers.py # 自動機動（旋回・8の字・上昇旋回）
```

⚠ Python は `NMPC_simulation/.venv`（numpy・matplotlib 入り）を使う。
システムの python3 には numpy が無い。

## 実機の計算資源レポート

SIL はホスト上で動くため、**メモリ消費はホストのものであって実機のものではない**。
実機資源は実機向け ELF を解析して得る。

```bash
python3 host/resource_report.py ../pico/pico-sdk2/pico-sdk/Benzen_NMPC/build/pico_copter.elf
```

## 構成

| ディレクトリ | 役割 |
|---|---|
| `shim/` | pico-sdk の代替。include パスで本物より先に解決させる |
| `hal_sil/` | センサの SIL 実装（`sensor.cpp` の差し替え） |
| `api/` | Python から叩く C ABI |
| `host/` | ctypes バインディング、資源レポート |

## 注意点

- **1 プロセスに 1 インスタンス**。ファームはグローバル変数で状態を持つ。
- **仮想時間**。`time_us_32()` は SIL が明示的に進める。`sleep_ms` はホストを待たせない。
- **core1 は実スレッド**。`angle_control()` は `while(1)` なので 1 回だけ呼ぶことができない。
  `sem` をホストの condition variable で実装し、完了ハンドシェイクで決定性を確保している。
- **`pwm.cpp` は本物を使う**。`set_duty_*` のクリップ（0.01〜0.97）を実機と同じに保つため。
- ⚠ `pwm.cpp` のスライス名は実際のモータ位置と一致していないが、**実機はこの結線で飛んでいる**。
  変更しないこと。`api/sil_api.cpp` の対応表が実際の結線を反映している。
- ⚠ **プラントの状態を直接書き換えないこと。** 外乱は `ClosedLoop.disturb_rates()` で角速度に与える。
  姿勢だけ動かすと EKF がジャイロバイアスと誤解釈し、制御を誤診する。
- ⚠ **`PlantParams.T_max` は未検証の較正パラメータ。** 実機のホバー duty を測ったら必ず更新すること。
- 実機で使えるスロットル帯は **0.200〜0.428**（`firmware_consts.py` 参照）。
  始動と同時に duty 0.444 へ跳ぶ。
