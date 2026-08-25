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
- ⚠ `pwm.cpp` のスライス名は実際のモータ位置と一致していない
  （`Slice_num_front`=FR/RR, `_middle`=FL/RL, `_rear`=MR/ML）。`api/sil_api.cpp` の対応表を正とする。
