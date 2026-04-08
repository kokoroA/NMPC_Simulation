# research_project

小型ドローン（M5Stamp Fly）向けの **非線形モデル予測制御（NMPC）** と **ADMM による制約付き最適化** を中心とした研究用リポジトリです。机上シミュレーション（Python）、組み込みファームウェア（C++/PlatformIO）、および理論・実装方針のメモをまとめています。

## ディレクトリ構成

| パス | 内容 |
|------|------|
| [`memo/研究メモ/`](memo/研究メモ/) | 制御アルゴリズム・今後の実装方針などの研究メモ（Markdown） |
| [`program/M5StampFly_NMPC_ADMM/`](program/M5StampFly_NMPC_ADMM/) | M5Stamp Fly 向け飛行制御ファームウェア（NMPC を ADMM で解く実装） |
| [`program/MPC_simulation/`](program/MPC_simulation/) | MPC/NMPC 関連の Python シミュレーション・行列生成・ヘッダ出力 |

## 研究メモ（概要）

- **NMPC の計算フロー** — RTI（Real-Time Iteration）と準ニュートン法（TR1/SR2）による予測行列・ヘッセ行列の更新など（[`rti_tr1_sr2_nmpc_control_flow.md`](memo/研究メモ/rti_tr1_sr2_nmpc_control_flow.md)）
- **飛行制御・FTC への発展案** — モーター制約の動的書き換え、Q 行列の切り替え、計算負荷削減の方向性など（[`flight_control_ftc_evolution.md`](memo/研究メモ/flight_control_ftc_evolution.md)）

## 組み込みファームウェア（`program/M5StampFly_NMPC_ADMM`）

従来の角度・角速度 2 重 PID に代わり、**制約付き MPC を ADMM で解く** Stamp Fly 向けプロジェクトです。ベースは [StampFly2024June](https://github.com/M5Fly-kanazawa/StampFly2024June) 系の構成で、ビルドは [PlatformIO](https://platformio.org/) を想定しています。

詳細は同ディレクトリの [`README.md`](program/M5StampFly_NMPC_ADMM/README.md) および [`ADMM_MPC_CONTROL_LAW.md`](program/M5StampFly_NMPC_ADMM/ADMM_MPC_CONTROL_LAW.md) を参照してください。

## シミュレーション（`program/MPC_simulation`）

離散化・ヤコビアン生成・Runge–Kutta などを用いた MPC/NMPC 関連の Python スクリプトと、組み込み向けに出力される `.h` ファイルが含まれます。依存関係の例は [`requirements.txt`](program/MPC_simulation/requirements.txt)（例: `sympy`）を参照してください。

## ライセンス・帰属

サブプロジェクトに個別の `LICENSE` やサードパーティライブラリの利用がある場合は、各 `program/` 配下の README・ライセンス表記に従ってください。
