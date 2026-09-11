# Benzen_NMPC

Raspberry Pi Pico 2 W 上の Hexa-X 姿勢 NMPC ファームウェア。

ビルド手順の正は親リポジトリの [`docs/benzen_nmpc_build.md`](../../../../../docs/benzen_nmpc_build.md)。

```bash
export PICO_SDK_PATH="/Users/kokoro/Documents/research_project/program/pico/pico-sdk2/pico-sdk"
cd "$PICO_SDK_PATH/Benzen_NMPC"
cmake -S . -B build_fresh -DPICO_SDK_PATH="$PICO_SDK_PATH"
cmake --build build_fresh --target pico_copter -j8
```

書き込むファイルは `build_fresh/pico_copter.uf2`（Pico 2 W 専用）。
