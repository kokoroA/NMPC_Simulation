# Benzen_NMPC ビルド手順

Raspberry Pi Pico 2 W（RP2350）向け NMPC ファームウェア `pico_copter` のビルドと書き込み。

- 実装解説: `program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/docs/NMPC_REPORT.md`
- パラメータの正: `Benzen_NMPC/nmpc_params.hpp`（現行 `NX = 8`, `N = 12`）
- ホスト上の SIL ビルドは別物 → [`program/sil/USAGE.md`](../program/sil/USAGE.md)

---

## 1. 前提

| 項目 | このマシンでの実測（2026-08-28） |
|---|---|
| プロジェクト | `program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/` |
| ターゲット | **Pico 2 W**（`PICO_BOARD=pico2_w`, `PICO_PLATFORM=rp2350-arm-s`） |
| pico-sdk | 同ディレクトリの親 `pico-sdk`（version **2.1.1**） |
| CMake | 3.13 以上（実測 3.31.5, `/usr/local/bin/cmake`） |
| クロスコンパイラ | `arm-none-eabi-gcc` 10.3.1（`/usr/local/bin/arm-none-eabi-gcc`） |
| Eigen | CMakeLists に **絶対パス固定**: `/Users/kokoro/Documents/drone_project/pico/eigen` |
| ビルドタイプ | 未指定時 `Release` |
| ジェネレータ | Unix Makefiles |

`pico-sdk2` 直下に `Benzen_NMPC` はない。正しい場所は **`pico-sdk/Benzen_NMPC`**。

`PICO_BOARD` は `CMakeLists.txt` で `pico2_w` に FORCE 済み。RP2040（Pico 1）向けにはならない。

### Eigen パス（環境移行で壊れる）

```cmake
include_directories(/Users/kokoro/Documents/drone_project/pico/eigen)
```

（`CMakeLists.txt` 23 行目）

このマシンでは実在する。別環境では `program/pico/eigen` などに合わせて CMakeLists を直す。コメントアウトされた Linux パス `/home/student01/pico/pico-sdk/eigen` は旧環境の名残。

### 初回 cmake が遅い理由

PATH に `picotool` が無いと、CMake が **picotool 2.1.1 をソースから取ってビルドする**。初回はネットワークが必要。警告:

> No installed picotool with version 2.1.1 found - building from source

2 回目以降は `build_fresh/_deps/` に残る。

---

## 2. ビルドコマンド（推奨）

既存の `build/` は 2026-07-15 の成果物（予稿・SIL の ELF 解析に使ったバイナリ）なので、**消さない**。新規は `build_fresh` を使う。

```bash
export PICO_SDK_PATH="/Users/kokoro/Documents/research_project/program/pico/pico-sdk2/pico-sdk"

cd "$PICO_SDK_PATH/Benzen_NMPC"

cmake -S . -B build_fresh -DPICO_SDK_PATH="$PICO_SDK_PATH"

cmake --build build_fresh --target pico_copter -j8
```

`-DPICO_SDK_PATH=...` は環境変数を出さずに渡す書き方。どちらでもよい。

### 再ビルド（ソース変更後）

```bash
cmake --build /Users/kokoro/Documents/research_project/program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/build_fresh --target pico_copter -j8
```

CMakeLists を変えたら `cmake -S . -B build_fresh -DPICO_SDK_PATH=...` からやり直す。

### クリーン再構成

```bash
rm -rf build_fresh
# その後「2. ビルドコマンド」を最初から
```

`build/` を `rm -rf` しないこと（7/15 の参照 ELF が消える）。

---

## 3. 成功時の確認

| 確認 | 期待値 |
|---|---|
| 成果物 | `build_fresh/pico_copter.uf2` |
| UF2 family ID | `0xe48bff57`（RP2350-ARM-S） |
| ボード | cmake ログに `Target board (PICO_BOARD) is 'pico2_w'` |

family ID の確認:

```bash
python3 - <<'PY'
import struct, pathlib
p = pathlib.Path("build_fresh/pico_copter.uf2")
magic0, magic1, flags, addr, psz, blockno, nblocks, family = struct.unpack_from("<8I", p.read_bytes())
print(f"family={family:#010x} size={p.stat().st_size}")
# 0xe48bff57 = RP2350-ARM-S。0xe48bff56 なら RP2040 なので Pico 2 W には入れない
PY
```

2026-08-28 の実測（`N=17`, Wi-Fi テレメ有効）:

| | 値 |
|---|---|
| 構成 | 約 7 s（picotool キャッシュあり） |
| コンパイル | 約 86 s（`-j8`） |
| `arm-none-eabi-size` | text 674936 / bss **402456**（SRAM 520 KB の約 75.6%） |
| UF2 | 約 1.3 MB |

bss 402,456 B は SIL 調査 `20260825_sil_m0_firmware_on_host.md` の 7/15 ELF と同じ桁。主因は `Logdata` と `g_nmpc_ctx`。

上記は旧構成の履歴値。2026-09-20 の `NX=8,N=12` は Pico実機の
`iter_us/loop_us` をまだ取得していないため、旧値を性能値として流用しない。

2026-09-20 の `NX=8,N=12` freshクロスビルド:

| | 値 |
|---|---|
| `arm-none-eabi-size` text | 691,344 B |
| data | 0 B |
| bss | **417,720 B** |

クロスビルドと静的メモリは合格。100 Hz期限はPico上の`iter_us/loop_us`実測待ち。

---

## 4. 書き込み

対象は **Pico 2 W のみ**。Pico 1（RP2040）にこの UF2 を入れるとブートローダが拒否する。

### 方法 A: BOOTSEL + ドラッグ（確実）

1. BOOTSEL を押しながら USB 接続（またはリセット）
2. マウントされたボリューム（macOS では `RP2350`）へ `pico_copter.uf2` をコピー
3. 自動で再起動する

```bash
cp build_fresh/pico_copter.uf2 /Volumes/RP2350/
```

### 方法 B: ビルドが落とした picotool

PATH の `picotool` は無い。CMake が `build_fresh/_deps/picotool-build/picotool` に作る。

```bash
./build_fresh/_deps/picotool-build/picotool load -f build_fresh/pico_copter.uf2
```

`-f` は実行中の Pico を BOOTSEL へ落とす。失敗したら方法 A。

---

## 5. 起動後の確認

USB シリアル（`pico_enable_stdio_usb` 有効 / UART 無効）:

```
#wifi: AP 'Benzen_NMPC' IP 192.168.4.1 CMD:5000 TEL:5001
```

| | 値 |
|---|---|
| AP SSID | `Benzen_NMPC` |
| パスワード | `benzen2026` |
| Pico IP | `192.168.4.1` |
| UDP | CMD 5000 / TEL 5001 |

PC クライアント: `Benzen_NMPC/tools/nmpc_udp_client.md`

既定ビルドは `NMPC_WIFI_TELEM=1`, `NMPC_BENCH_SERIAL_LOG=0`（CSV は USB ではなく Wi-Fi）。

---

## 6. CMake が固定しているもの

`CMakeLists.txt` から。ビルドのたびに指定し直す必要はない。

| 定義 | 値 | 意図 |
|---|---|---|
| `PICO_BOARD` | `pico2_w` | Pico 2 W / CYW43 |
| `SYS_CLK_KHZ` | `125000` | PWM `wrap=3124, clkdiv=100` が 400 Hz になる前提 |
| `PICO_STACK_SIZE` | `0x8000`（32 KB） | core0 の Hessian warmup が 2 KB 既定スタックを溢す |
| `PICO_CORE1_STACK_SIZE` | `0x1000`（4 KB） | RP2350 `SCRATCH_X` 上限。`nmpc_step` は BSS scratch |
| `EIGEN_NO_MALLOC` | 定義 | 制御ループで動的確保禁止 |
| `NMPC_WIFI_TELEM` | `1`（pico2_w 時） | AP + UDP テレメ |
| `NMPC_BENCH_SERIAL_LOG` | `0`（Wi-Fi 時） | USB CSV を切る |
| `NMPC_BENCH_LOG` | `0` | 机上ベンチ（参照固定・常時 NMPC）はオフ |
| `NMPC_TR1_VERIFY` | `0` | 定期フル再構築なし |
| `DEBUG_SKIP_LOCK_COM` | `0` | |

姿勢 NMPC の切替は CMake ではなく `control.cpp` 先頭の `#define USE_NMPC`。コメントアウトすると 2 段 PID に戻る。

リンク: `pico_cyw43_arch_lwip_threadsafe_background`（Wi-Fi AP）。CYW43 非対応ボードではシリアルベンチ側に落ちるが、現行 CMake は pico2_w 固定なので通常は通らない。

---

## 7. つまずきやすい点

1. **`PICO_SDK_PATH` 未指定** → `pico_sdk_import.cmake` が FATAL_ERROR。必ず `-D` か `export` する。
2. **Eigen の絶対パス** → 環境を移すと `#include <Eigen/Dense>` が死ぬ。CMakeLists 23 行目を直す。
3. **古い CMake cache** → 別マシンのパスが残っていると構成が壊れる。`build_fresh` を消して再構成。
4. **RP2040 UF2 を Pico 2 W に書く** → family `0xe48bff56` は受け付けない。本プロジェクトは `0xe48bff57`。
5. **`N` を変える** → `nmpc_params.hpp` が正。`plan.md` の `N=20` は古い。変えたら計算時間と SRAM を実測し直す。
6. **SIL と混同しない** → `program/sil` の `cmake -S . -B build` はホスト共有ライブラリ。UF2 は出ない。

---

## 8. 関連

- 主戦場訂正（サブモジュール配下である理由）: `docs/ai/decisions/20260820_hardware_phase_correction.md`
- Pico 2 W 移植の先行例（`Benzen_new`）: `docs/ai/decisions/20260825_benzen_new_pico2w.md`
- SRAM 内訳: `docs/ai/decisions/20260825_sil_m0_firmware_on_host.md`
- 本調査の記録: `docs/ai/decisions/20260828_benzen_nmpc_build.md`
