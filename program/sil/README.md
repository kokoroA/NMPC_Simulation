# SIL — 実機ファームをホストで動かすシミュレーション基盤

実機に書き込むのと**同一の C/C++ ソース**を PC 上で動かす。
`control.cpp` / `pid.cpp` / `ekf.cpp` / `auto_maneuvers.cpp` は一行も変更しない。

計画書: [`docs/sil_simulator_plan.md`](../../docs/sil_simulator_plan.md)

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
../NMPC_simulation/.venv/bin/python host/run_m3_demo.py --csv
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

## 検証スイート

```bash
../NMPC_simulation/.venv/bin/python host/test_m1_sensors.py       # センサ符号・単位と EKF
../NMPC_simulation/.venv/bin/python host/test_m2_closed_loop.py   # 姿勢閉ループ
../NMPC_simulation/.venv/bin/python host/test_m3_logging.py       # ログと描画
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
