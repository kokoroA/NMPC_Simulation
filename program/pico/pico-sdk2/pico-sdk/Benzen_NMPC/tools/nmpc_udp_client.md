# nmpc_udp_client.py 使い方

Pico 2 W の Wi-Fi AP 経由で NMPC ベンチテレメトリを受信する PC 側クライアントです。  
USB シリアルの `debug_print_nmpc_bench()` と**同じ CSV 形式**（10 Hz）で表示・保存できます。

## 概要

| 項目 | 値 |
|------|-----|
| スクリプト | `tools/nmpc_udp_client.py` |
| Pico IP | `192.168.4.1` |
| コマンド UDP | `5000`（REGISTER / SET_COST） |
| テレメトリ UDP | `5001`（ベンチ CSV） |
| AP SSID | `Benzen_NMPC` |
| AP パスワード | `benzen2026` |
| UDP プロトコル ver | **5**（`wifi_telemetry.hpp` / `nmpc_udp_client.py` の `NMPC_UDP_VER`）。パケット 196 B |

ファームウェア側では `NMPC_WIFI_TELEM=1` が有効なビルド（pico2_w）が必要です。  
現在の既定設定では USB シリアル CSV（`NMPC_BENCH_SERIAL_LOG=0`）は無効で、Wi-Fi テレメのみが有効です。

## 前提条件

1. Pico 2 W にファームウェアを書き込み、起動する
2. USB シリアルで次のログが出ることを確認する

   ```
   #wifi: AP 'Benzen_NMPC' IP 192.168.4.1 CMD:5000 TEL:5001
   ```

3. PC の Wi-Fi を Pico の AP に接続する
   - SSID: `Benzen_NMPC`
   - パスワード: `benzen2026`
   - PC の IP が `192.168.4.x` になること（Pico は `192.168.4.1`）

4. Python 3 が使えること（標準ライブラリのみで動作）

##手動IP
IP アドレス
192.168.4.2
サブネットマスク
255.255.255.0
ルーター
192.168.4.1

## 実行場所

プロジェクトルート（`Benzen_NMPC/`）から実行するのがおすすめです。

```bash
cd Benzen_NMPC
python3 tools/nmpc_udp_client.py
```

`tools/` ディレクトリ内から実行する場合:

```bash
cd Benzen_NMPC/tools
python3 nmpc_udp_client.py
```

## 基本コマンド

### 接続確認のみ

```bash
python3 tools/nmpc_udp_client.py --check
```

PC が `192.168.4.x` の IP を持っているか確認します。

成功例:

```
OK: この PC は 192.168.4.2 (Pico 192.168.4.1 へ到達可能なはず)
```

### リアルタイムテレメ受信

```bash
python3 tools/nmpc_udp_client.py
```

動作の流れ:

1. Pico へ `REGISTER` を UDP 5000 で送信
2. `REGISTER_ACK` を受信
3. UDP 5001 でテレメトリ待受（10 Hz）

出力例:

```
REGISTER_ACK from 192.168.4.1:5000 seq=0
テレメトリ待受 UDP 5001
#nmpc_bench CSV (10Hz) iter_us_limit=10000 (100Hz)
#reset_cause: 0=none 1=init 2=external 3=period 4=sr2_bad_c 5=sr2_unhealthy
#path: 0=tr1/sr2 1=reset@start 2=reset@after_sr2
#cols: seq,arm,nmpc,phi,theta,p,q,r,...
16683,2,1,0.0386,-0.0009,...
```

停止は `Ctrl+C`。

### リアルタイムグラフ表示（`--show-graph`）

テレメトリを受信しながら matplotlib でリアルタイムプロットします（約 10 Hz 更新）。

```bash
python3 tools/nmpc_udp_client.py --show-graph
```

表示内容（5 段）:

1. **姿勢** — phi / theta（実測）と phi_ref / theta_ref（スティック目標）
2. **角速度** — p / q / r と psi_ref（スティックヨー目標）
3. **モータ入力** — u_fr 〜 u_rl、u_base、T_ref（右軸 [V]）
4. **グループ化モータ** — roll 左右 / pitch 前後 / yaw CW・CCW の平均
5. **ソルバ** — iter_us と cost

オプション:

| オプション | 説明 |
|----------|------|
| `--graph-window SEC` | 横軸の表示幅（既定 60 秒） |
| `--print-csv` | グラフ表示中も CSV を標準出力に出す |
| `--log out.csv` | グラフと同時にファイル保存可能 |

```bash
# 30 秒分のウィンドウで表示
python3 tools/nmpc_udp_client.py --show-graph --graph-window 30

# グラフ + コスト変更 + ログ保存
python3 tools/nmpc_udp_client.py --show-graph --set-cost R_u=20 --log out.csv
```

グラフウィンドウを閉じるとクライアントも終了します。`matplotlib` が必要です。

```bash
pip install matplotlib
```

### CSV に保存

```bash
python3 tools/nmpc_udp_client.py --log out.csv
```

保存したファイルは `plot_nmpc_log.py` でグラフ化できます。

```bash
python3 tools/plot_nmpc_log.py out.csv
```

### NMPC コスト重みを変更

```bash
python3 tools/nmpc_udp_client.py --set-cost Q_phi=200 R_u=20
```

`--set-cost` 指定時は **コスト表示が自動で有効** になり、テレメトリで反映を確認できます。

```bash
python3 tools/nmpc_udp_client.py --set-cost Q_phi=200 R_u=20 --show-cost
```

### コスト重みの表示・反映確認（`--show-cost`）

テレメトリ UDP パケット内の `nmpc_cost_get_active()` 相当の値を表示します（**変化時のみ**）。

```bash
python3 tools/nmpc_udp_client.py --show-cost
```

出力例:

```
16683,2,1,0.0386,-0.0009,...
# cost_active seq=16683 Q_phi=200.0000 Q_theta=100.0000 Q_p=10.0000 Q_q=10.0000 Q_r=5.0000 R_u=20.0000 terminal_scale=10.0000
```

`--set-cost` と併用した場合:

```
SET_COST 送信: {'Q_phi': 200.0, 'R_u': 20.0}
# cost EXPECT (core1 apply 後にテレメで確認): Q_phi=200.0000 ... R_u=20.0000 ...
ACK (受付確認): {...}
  ※ ACK の Q/R は apply 前の active 値です。反映確認はテレメの # cost_active を参照してください。
...
# cost_active seq=16703 Q_phi=200.0000 ... R_u=20.0000 ...
# cost APPLIED: テレメトリが SET_COST と一致しました
```

| キー | 意味 |
|------|------|
| `Q_phi` | ロール角コスト |
| `Q_theta` | ピッチ角コスト |
| `Q_p` | ロール角速度コスト |
| `Q_q` | ピッチ角速度コスト |
| `Q_r` | ヨー角速度コスト |
| `R_u` | 制御入力コスト |
| `terminal_scale` | 終端コスト倍率 |

複数指定可能:

```bash
python3 tools/nmpc_udp_client.py --set-cost Q_phi=1 Q_theta=1 R_u=20 terminal_scale=10
```

## オプション一覧

| オプション | 説明 |
|------------|------|
| `--check` | ネットワーク接続だけ確認して終了 |
| `--pico-ip IP` | Pico の IP アドレス（既定: `192.168.4.1`） |
| `--wait SEC` | AP 接続を最大 SEC 秒待ってから開始 |
| `--set-cost KEY=VAL ...` | コスト重みを Pico に送信（コスト表示も自動 ON） |
| `--show-cost` | テレメトリの active コスト重みを表示（変化時） |
| `--show-graph` | リアルタイムグラフ表示 |
| `--graph-window SEC` | グラフ横軸の表示幅 [s]（既定 60） |
| `--print-csv` | `--show-graph` 時も CSV を標準出力 |
| `--log CSV` | テレメトリを CSV ファイルに保存 |
| `--duration SEC` | SEC 秒後に自動終了（0 = 無限） |

## 使用例

```bash
# AP 接続を 30 秒待ってから開始
python3 tools/nmpc_udp_client.py --wait 30

# 30 秒間だけ記録
python3 tools/nmpc_udp_client.py --log test.csv --duration 30

# コスト変更しながら記録・反映確認
python3 tools/nmpc_udp_client.py --set-cost Q_phi=200 R_u=20 --log out.csv

# コスト重みだけ監視（ベンチ CSV も表示）
python3 tools/nmpc_udp_client.py --show-cost

# リアルタイムグラフ
python3 tools/nmpc_udp_client.py --show-graph
```

## 出力 CSV の列

USB シリアルログと同一の 49 列です（ver 4 で `ftc` / `failsafe`、ver 5 で `rx_flags` / `rx_fl_run` / `rx_fl_cnt` を追加）。
`rx_flags` は SBUS フラグバイトの生値で、`0x04`=frame lost、`0x08`=receiver failsafe。
`failsafe` の原因コードは 0=正常 / 2=受信途絶 / 3=姿勢 / 4=角速度 / 5=過大G / 6=SBUS failsafe / 7=frame lost 連続（1 は廃止）。

```
seq, arm, nmpc,
phi, theta, p, q, r,
phi_ref, theta_ref, psi_ref, T_ref, u_base,
iter_us, loop_us, cost, reset, iter_max,
reset_cause, path, tr1, cycles,
cnt_tr1, cnt_tr1_ok, cnt_tr1_skip,
cnt_rst_period, cnt_rst_ext, cnt_sr2_c, cnt_sr2_llt, cnt_rst_after_sr2,
u_fr, u_fl, u_mr, u_ml, u_rr, u_rl,
d_fr, d_fl, d_mr, d_ml, d_rr, d_rl
```

補足:

- `reset_cause`: 0=none, 1=init, 2=external, 3=period, 4=sr2_bad_c, 5=sr2_unhealthy
- `path`: 0=tr1/sr2, 1=reset@start, 2=reset@after_sr2
- サンプリングレートは約 10 Hz（`seq` は 100 Hz 制御ループのカウンタ）

## トラブルシューティング

### `NG: 192.168.4.x の IP が見つかりません`

- PC の Wi-Fi が `Benzen_NMPC` に接続されているか確認
- 別の Wi-Fi（自宅ルーター等）に繋がっていないか確認
- `--check` で状態を確認

### `REGISTER_ACK なし` / `5秒間テレメトリなし`

- Pico が起動・動作中か（USB シリアルで `#wifi: AP ...` を確認）
- ファームウェアが `NMPC_WIFI_TELEM=1` でビルドされているか確認
- Pico 側で NMPC が動作しているか（`Arm_flag=2` 等）
- **UDP ver 不一致**: クライアント `NMPC_UDP_VER` と `wifi_telemetry.hpp` の `NMPC_UDP_VER` を揃える（現在は **5**）。ファームウェア変更後は必ず再ビルド・再書き込みする
- USB シリアルに `#wifi: client 192.168.4.x registered, tel -> UDP 5001` が出るか確認（出なければ REGISTER 未登録）

### `テレメ ver=... が不一致` / サイズ不一致の警告

- ファームウェアが古い（例 ver=4 / 188B）のにクライアントだけ新しい（ver=5 / 196B）状態
- `Benzen_NMPC` を再ビルドして Pico に書き込み、更新済みの `nmpc_udp_client.py` を使う

```bash
# ビルド例（build ディレクトリは環境に合わせる）
cd Benzen_NMPC/build && cmake .. && make -j4
# 書き込み後、クライアント実行
python3 tools/nmpc_udp_client.py
```

### `Network is unreachable`

- 先に `--check` で接続状態を確認
- AP 接続直後は `--wait 30` で待機してから実行

```bash
python3 tools/nmpc_udp_client.py --wait 30
```

### USB シリアルに CSV が出ない

pico2_w の既定ビルドでは `NMPC_BENCH_SERIAL_LOG=0` のため、シリアル CSV は無効です。  
テレメトリは本スクリプト経由で受信してください。シリアルも有効にしたい場合は `CMakeLists.txt` で `NMPC_BENCH_SERIAL_LOG=1` に変更して再ビルドします。

## 関連ファイル

| ファイル | 内容 |
|----------|------|
| `wifi_telemetry.cpp` | Pico 側 AP / UDP サーバ |
| `wifi_telemetry.hpp` | パケット定義・ポート番号 |
| `control.cpp` | ベンチスナップショット・テレメ送信 |
| `plot_nmpc_log.py` | ログ CSV のグラフ化 |
| `nmpc_udp_graph.py` | リアルタイムグラフ (`--show-graph`) |
