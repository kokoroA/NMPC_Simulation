# plot_nmpc_log.py 使い方

NMPC ベンチログ（`nmpc_bench` CSV）を読み込み、PNG グラフを一括生成するスクリプトです。  
`nmpc_udp_client.py --log` や USB シリアルの `debug_print_nmpc_bench()` で保存した CSV を可視化できます。

## 概要

| 項目 | 値 |
|------|-----|
| スクリプト | `tools/plot_nmpc_log.py` |
| 入力形式 | `nmpc_bench` CSV（10 Hz サンプリング） |
| 出力形式 | PNG 画像（5 枚） |
| 時間軸 | `seq` を 100 Hz 制御ループとして秒換算 |

## 前提条件

1. Python 3 が使えること
2. 次のパッケージがインストールされていること

```bash
pip install matplotlib pandas
```

3. 入力ファイルが `nmpc_bench` CSV 形式であること（後述）

## ログファイルの入手方法

| 方法 | 例 |
|------|-----|
| Wi-Fi テレメ保存 | `python3 tools/nmpc_udp_client.py --log out.csv` |
| USB シリアル保存 | シリアルモニタ出力を `log/nmpc_1.log` などに貼り付け・リダイレクト |
| 既存ログ | `log/nmpc_*.log`, `tools/out.csv` など |

詳細は [nmpc_udp_client.md](nmpc_udp_client.md) を参照してください。

## 実行場所

プロジェクトルート（`Benzen_NMPC/`）から実行するのがおすすめです。

```bash
cd Benzen_NMPC
python3 tools/plot_nmpc_log.py log/nmpc_12.log
```

`tools/` ディレクトリ内から実行する場合:

```bash
cd Benzen_NMPC/tools
python3 plot_nmpc_log.py out.csv
```

## 基本コマンド

### 既定ログをプロット

引数を省略すると `log/nmpc_1.log` を読み込みます。

```bash
python3 tools/plot_nmpc_log.py
```

### 指定ファイルをプロット

```bash
python3 tools/plot_nmpc_log.py log/nmpc_12.log
python3 tools/plot_nmpc_log.py tools/out.csv
```

### 出力先を指定

```bash
python3 tools/plot_nmpc_log.py log/nmpc_12.log -o /tmp/plots
```

### 画面表示も行う

PNG 保存に加え、matplotlib ウィンドウでグラフを表示します。

```bash
python3 tools/plot_nmpc_log.py log/nmpc_12.log --show
```

## オプション一覧

| オプション | 説明 |
|------------|------|
| `log_file` | 入力ログファイル（省略時: `log/nmpc_1.log`） |
| `-o`, `--output DIR` | PNG 出力ディレクトリ（省略時: `log/plots/<ログ名>/`） |
| `--show` | グラフを画面表示する |

## 実行時の標準出力

スクリプトは読み込み後、次のサマリを表示します。

```
Loaded 342 rows from log/nmpc_12.log
Time range: 23.78 - 58.20 s
NMPC active samples: 340
Reset events: 2
Saved plots to log/plots/nmpc_12/
```

## 出力グラフ（5 枚）

既定の出力先は `log/plots/<ログ名>/` です（例: `log/nmpc_12.log` → `log/plots/nmpc_12/`）。

| ファイル | 内容 |
|----------|------|
| `01_attitude_rates.png` | 姿勢角（phi / theta と参照）と角速度（p / q / r） |
| `02_nmpc_timing_cost.png` | ソルバ時間（iter_us / loop_us）、コスト、状態フラグ（nmpc / arm） |
| `03_motor_inputs.png` | 6 モータ duty（u_fr 〜 u_rl）と NMPC デルタ（d_* = u - u_base） |
| `04_diagnostics.png` | 診断カウンタ（tr1 / reset / sr2 関連） |
| `05_attitude_control_overlay.png` | 姿勢・角速度とグループ化モータ入力の重ね合わせ |

### リセットイベントの表示

`reset > 0` のサンプル位置に赤い縦線が引かれ、`reset_cause` に応じたラベルが付きます。

| reset_cause | ラベル |
|-------------|--------|
| 0 | none |
| 1 | init |
| 2 | external |
| 3 | period |
| 4 | sr2_bad_c |
| 5 | sr2_unhealthy |

### モータグループ（Figure 5）

Hexa-X 配置（x: 前+, y: 右+）に基づき、姿勢制御と対応するモータ群の平均 duty を重ねて表示します。

| 軸 | グループ | モータ |
|----|----------|--------|
| Roll | 右 / 左 | FR, MR, RR / FL, ML, RL |
| Pitch | 前 / 後 | FR, FL / RR, RL |
| Yaw rate | CW / CCW | FR, ML, RR / FL, MR, RL |

右軸には各グループ平均に加え、`u_base`（破線）も表示されます。

## 入力 CSV 形式

### ヘッダー行（`#` で始まる行）

```
#nmpc_bench CSV (10Hz) iter_us_limit=10000 (100Hz)
#reset_cause: 0=none 1=init 2=external 3=period 4=sr2_bad_c 5=sr2_unhealthy
#path: 0=tr1/sr2 1=reset@start 2=reset@after_sr2
#cols: seq,arm,nmpc,phi,theta,p,q,r,...
```

- `#cols:` 行は読み飛ばされます（列定義はスクリプト内の `COLUMNS` と `control.cpp` で一致）
- `iter_us_limit=...` がメタデータに含まれる場合、Figure 2 に制限時間の水平線が描画されます

### データ行（42 列）

```
seq,arm,nmpc,phi,theta,p,q,r,phi_ref,theta_ref,psi_ref,T_ref,u_base,
iter_us,loop_us,cost,reset,iter_max,reset_cause,path,tr1,cycles,
cnt_tr1,cnt_tr1_ok,cnt_tr1_skip,cnt_rst_period,cnt_rst_ext,cnt_sr2_c,cnt_sr2_llt,cnt_rst_after_sr2,
u_fr,u_fl,u_mr,u_ml,u_rr,u_rl,
d_fr,d_fl,d_mr,d_ml,d_rr,d_rl
```

補足:

- サンプリングは約 10 Hz（制御ループ 100 Hz の `seq` を 10 行に 1 回記録）
- 時間軸 `time_s = seq / 100`
- 列数が不足・超過する行は自動でパディング / 切り詰めされます

## 使用例

```bash
# UDP クライアントで記録 → グラフ化
python3 tools/nmpc_udp_client.py --log tools/out.csv --duration 60
python3 tools/plot_nmpc_log.py tools/out.csv

# 出力先を指定してプロット
python3 tools/plot_nmpc_log.py log/nmpc_12.log -o results/nmpc_12

# 画面表示しながら確認
python3 tools/plot_nmpc_log.py log/nmpc_12.log --show
```

## トラブルシューティング

### `ファイルが見つかりません: ...`

- パスが正しいか確認（プロジェクトルートからの相対パスを推奨）
- `log/nmpc_1.log` が無い場合は、引数で実在する CSV / ログを指定する

### `データ行がありません: ...`

- ファイルに CSV データ行が含まれているか確認
- ヘッダー行（`#`）だけでデータが無い場合に発生します

### `ModuleNotFoundError: No module named 'matplotlib'` / `pandas`

```bash
pip install matplotlib pandas
```

### グラフが真っ白 / 値がおかしい

- ログが途中で切れていないか確認
- `#nmpc_bench CSV` ヘッダー付きの正しい形式か確認
- 列のずれがある場合は `nmpc_udp_client.py` やファームウェアのバージョン不一致を疑う

## 関連ファイル

| ファイル | 内容 |
|----------|------|
| `control.cpp` | `debug_print_nmpc_bench()` — CSV 出力元 |
| `nmpc_udp_client.py` | Wi-Fi テレメ受信・CSV 保存 |
| [nmpc_udp_client.md](nmpc_udp_client.md) | UDP クライアントの使い方 |
| `log/nmpc_*.log` | 過去のベンチログ例 |
| `docs/NMPC_REPORT.md` | NMPC 実装・性能レポート |
