# `LOG` を ch7・`SERVO` を ch4 へ。⚠ ch7 を空けるため `LINETRACE` を廃止し、残っていた参照を始末した

- **日付**: 2026-09-16
- **種別**: bugfix | design
- **対象**: `Benzen_NMPC/radio.hpp`, `control.cpp`（`logdata_out_com` / `logging` / PID 経路 / `unique_mission`）
- **タグ**: #radio #SBUS #Chdata #LOG #SERVO #LINETRACE #しきい値 #ビルド破損

## 背景

送信機の割り当て変更に合わせ **`LOG 7` / `SERVO 4`** へ。
同日の [`20260916_swap_log_and_servo_channels.md`](20260916_swap_log_and_servo_channels.md)（`LOG 4` / `SERVO 6`）を**上書きする**。

## 原因 / 判断根拠

### (1) ⚠ 着手時点でツリーがビルド不能だった

`radio.hpp` が編集途中で、**`#define SERVO` が値なし**、**`#define LINETRACE 7` がコメントアウト済み**。
一方で参照は残っていたため 4 件のコンパイルエラー:

```
control.cpp:1643,1645: expected primary-expression before ']' token   （Chdata[SERVO]）
control.cpp:3595:      'LINETRACE' was not declared in this scope
```

### (2) ⚠ `LOG 7` は旧 `LINETRACE 7` と共存できない

`LINETRACE` は **チャンネル番号と Flight_mode 値の二役**を兼ねていた（元コードの作り）。
`LOG` を 7 に入れる以上、廃止するしかない。
モード設定側（`Flight_mode = LINETRACE`）は既に全てコメントアウト済みで、
**`Flight_mode` に 7 が入る経路は存在しない**ことを確認した。

残っていた参照は 2 か所:

| 箇所 | 状態 | 対応 |
|---|---|---|
| `control.cpp:2247` `if (Flight_mode != LINETRACE)` | PID 経路（`USE_NMPC` ではコンパイルされない）。だから今回のエラーには出なかったが、PID ビルドは壊れる | リテラル `7` に置換。条件は常に真のままで**挙動は不変** |
| `control.cpp:3595` `Chdata[LINETRACE]` | `unique_mission()` 内。**この関数はどこからも呼ばれていない**（プロトタイプも呼び出しも無し） | 関数ごと `#if 0` で無効化 |

⚠ 隣の `else if (Flight_mode == 100)` は、以前から誰かが LINETRACE 比較をリテラルへ逃がした跡。
**「この分岐はもう使われていない」という判断は既に一度下されていた。**

### (3) しきい値の名前がチャンネル移動で嘘になる

`logging()` / `logdata_out_com()` の LOG 判定は `(CH5MAX + CH5MIN) / 2` を使っていた。
値としては正しい中央値（1003）だが、**`CH5` は「ch5 の較正値」という名前**で、
LOG が ch4 → ch7 と動いた今は誤解を招く。

→ `radio.hpp` に **`SW_MID`**（チャンネル非依存のスイッチ中央しきい値）を定義して置き換えた。

⚠ 中央しきい値を選ぶ方針自体は前回と同じ。2 ポジション・3 ポジション・ボリュームの
どれでも効くので、スイッチ種別が変わっても壊れない。
旧コードの `Chdata[LINETRACE] < 200` から、**ch7 は 200 を下回るところまで落ちる**と推測でき、
中央しきい値でも問題ない。

## 対応

| 箇所 | 変更 |
|---|---|
| `radio.hpp` | `#define LOG 7` / `#define SERVO 4`、`LINETRACE` は廃止（理由をコメント）、`SW_MID` を新設 |
| `control.cpp:1130` | `logdata_out_com`: `(CH5MAX+CH5MIN)*0.5` → `SW_MID` |
| `control.cpp:2673` | `logging`: `(CH5MAX+CH5MIN)/2` → `SW_MID` |
| `control.cpp:2247` | `Flight_mode != LINETRACE` → `!= 7`（挙動不変） |
| `control.cpp:3594-3615` | `unique_mission()` を `#if 0` で無効化 |

### 触らなかったもの

- **`servo_control()` の `(SERVO_MAX+SERVO_MIN)/2` = 1024**: `SERVO` が ch4 に戻ったので、
  これは**元々の組み合わせ**（ch4 × 1024）。変える理由がない
- `Chdata[FTC]`（ch5, `>500`/`<200`）: 今回の変更と無関係

## 検証結果

- **ビルド成功、警告ゼロ**。`text 684560 / bss 422428`
- 有効行の `Chdata[...]` 参照を全数確認 — すべてマクロ経由、生インデックスの直書きは無し
- `LINETRACE` の残存参照は **`#if 0` の中の 1 件のみ**（ビルドが通ることで非コンパイルを確認）
- `unique_mission` はプロトタイプも呼び出しも存在しないことを確認
- しきい値: `SW_MID` = (1638+368)/2 = **1003**。
  記録（`> 1003`）と吸い出し（`< 1003`）が排他で、「スイッチ上で記録 → 下ろして吸い出し」は維持
- ⚠ **実機未検証**

## 次に触る人へ

- ⚠ **次の電源投入で `Chdata[4]` と `Chdata[7]` の生値を必ず確認すること。**
  `radio.cpp` に各チャンネルの `printf` がコメントアウトで残っている。
  ch7 の OFF 側が 1003 を確実に下回り、ON 側が上回ることを目で見てから飛ばす
- ⚠ **ch7 が 3 ポジションスイッチなら「一番上だけ記録 ON」になる。** 中段で記録したいなら
  `SW_MID` ではなく 200 側のしきい値に戻す必要がある
- ⚠ **`LINETRACE` は完全に廃止した。** ライントレースを復活させるなら、
  チャンネル番号と Flight_mode 値を分けるところから作り直すこと（二役だったのが今回の衝突の原因）
- しきい値の流儀は依然バラバラ（`SW_MID` / `>500` / `<200` が混在）。
  **チャンネルを動かすときはマクロの入れ替えだけでは済まない**
- 関連: [`20260916_swap_log_and_servo_channels.md`](20260916_swap_log_and_servo_channels.md)（本記録が上書き）
