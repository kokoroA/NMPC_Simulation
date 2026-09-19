# TR1 skip の連続上限を 5 → 8 にし、`|c|` / `norm_sq` / `skip_run` を 100 Hz ログへ出した

- **日付**: 2026-09-19
- **種別**: tuning | investigation
- **対象**: `Benzen_NMPC/nmpc_params.hpp`(rti::tr1_skip_max_run), `nmpc.hpp`/`nmpc.cpp`(NmpcStepDiag, tr1_update_sx_su), `control.cpp`(logging, log_output, DATANUM)
- **タグ**: #Benzen_NMPC #TR1 #tr1_skip_max_run #tr1_c_max #フル再構築 #ログ列追加 #DATANUM

## 背景

[`20260919_tr1_skip_is_self_reinforcing_not_random.md`](20260919_tr1_skip_is_self_reinforcing_not_random.md) の
対策 D（観測）と A（キャップ 5 → 8）の実装。

## 対応

### A: `tr1_skip_max_run` 5 → 8

見積り: フル再構築が飛行時間の **11.2 % → 3.6 %**（ブロック下三角後の 22 ms 想定）。
⚠ 代償は **最大 70 ms の Sx/Su 停滞**（旧 cap=5 では 40 ms）。
8 回目の周期は `admm_solve()` の前に `do_full_rebuild()` が走るので、**その周期の解は新しい Sx/Su で出る**。
よって「古いまま解く」周期は最大 7 回 = 70 ms。

**却下案との関係**: 2026-09-10 が却下したのは「捨てるだけで上限なし」で、理由はリプレイでの
**27 周期連続スタール**。8 はその範囲に入らない。加えて 0910 自身の論理

> 36 ms の無制御は…「ヘシアンが 1 周期古い」害より桁違いに大きい

がキャップにもそのまま当てはまる。フル再構築は `loop_us` 実測 41,967 us ＝ **4 制御周期ぶん
duty が ZOH で凍る**のに対し、キャップ超過は **x0 も解も毎周期新しい**。

### D: 3 列追加（`tr1_c` / `secant_nsq` / `tr1_skip_run`）

`NmpcStepDiag` に `last_tr1_c` / `last_secant_norm_sq` / `last_tr1_skip_run` を追加し、
`tr1_update_sx_su()` の 3 つの出口すべてで記録して 100 Hz ログへ出す。

⚠ **なぜ必要か**: `tr1_c_max = 1e6` は 2026-09-10 の**ホストリプレイ**で決めた値だが、
そのリプレイは再構築を 1.14 回/秒と予測したのに**実機は 2.6〜2.8 回/秒**だった。
**ホストが実機を再現できていない以上、実機の `|c|` 分布を見ずに閾値を触ってはいけない。**

`c = (residualᵀ Q̄ residual) / norm_sq²` なので、`c` 単体ではなく **分母 `norm_sq` も併せて**見ないと
「残差が大きいのか」「分母が小さいのか」を切り分けられない。だから 2 列とも出す。

### 副作用: リング長 21.8 s → 19.8 s

`DATANUM` 30 → 33。`LOGDATANUM = 65280` は据え置きなので `LOGROWS = 65280/33 = 1978 行 = 19.8 秒`。
**SRAM の追加はゼロ。**

### 保険: 実際に書いた列数をヘッダに出す

⚠ 列を足して `DATANUM` を直し忘れると **次の行へはみ出してログ全体が壊れる**（サイレントに）。
`g_log_cols_written` に `logging()` が書いた `k` を残し、`#nmpc_log ... cols=33 written=33` の形で
吸い出しヘッダに出すようにした。**`cols` と `written` が違ったらログを信用しないこと。**

## 検証結果

- 列番号のコメントは 1〜33 で連続し、`DATANUM = 33` と一致。
- ビルド: `text 687360 → 687688`、`bss 422772 → 422776`。
- ⚠ **実機未検証。**

## 次に触る人へ

次の飛行で見る順番:

1. **`#nmpc_log ... cols=33 written=33`** — 違っていたらそのログは無効。
2. **`solver_code / 10 == 1` の回数**（再構築）。11.2 % → 3.6 % を狙っている。
   減っていなければキャップが効いていないので `tr1_skip_run` 列を見る。
3. **`tr1_c` のヒストグラム。** `1e6` のすぐ上に密集していれば閾値の問題、
   桁違い（1e8 以上）なら閾値をいじっても無駄で、残差そのものを疑う。
4. **`secant_nsq`。** `tr1_c` が大きいときにこれが小さいなら「分母が消えて `c` が 2 乗で膨らむ」経路。
5. **姿勢追従の悪化**（`phi/theta` と参照の差、`p/q` の振動）。⚠ **悪化していたら 6 へ戻すこと。**

- ⚠ **`tr1_c_max` は今回も触っていない。** 上げると 0909 の線形化点発散が戻る
  （`20260910_sr2_bad_c_rebuild_storm.md`）。
- 関連: [`20260919_ulin_guard_output_decoupled_and_box_relative.md`](20260919_ulin_guard_output_decoupled_and_box_relative.md)（同じ便で検証する対策 1・3）
