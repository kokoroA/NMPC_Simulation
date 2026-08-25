# SIL M4: WebUI と、停止判定の単位不整合の修正

- **日付**: 2026-08-25
- **種別**: bugfix
- **対象**: `Benzen_new/control.cpp`（実機）, `program/sil/{host/webserver.py,web/*}`
- **タグ**: #SIL #M4 #WebUI #SSE #停止判定 #Disable_duty #Flight_duty #common_duty #未追跡

## 背景 / 症状

M2/M3 で「モータが回り始めた瞬間に duty 0.444 へ跳び、ホバー点(0.285)を超える」
という問題を報告していた。原因は**停止判定の単位が合っていない**ことだった。

```c
if (T_ref / BATTERY_VOLTAGE < Disable_duty)   // 電圧比 と duty を比較していた
```

`T_ref / BATTERY_VOLTAGE` は電圧比、`Disable_duty` は duty。次元が違うものを比べていた。

## 対応

### 1. 停止判定を duty 基準に統一（ユーザー指示）

```c
const float common_duty = T_ref * T_ref_coe;  // 姿勢ゼロ時の PWM
if (common_duty < Disable_duty) {             // 0.1
  motor_stop();
  ...
}
```

効果:

| | 修正前 | 修正後 |
|---|---|---|
| モータ始動スロットル比 | 0.200 | **0.045** |
| 始動時の duty（下限） | 0.444 | **0.100** |
| ホバー(duty 0.285)到達可否 | **不可** | **可**（スロットル比 0.128） |

SIL で検証: スロットル 0.128 で高度を維持、0.15 で +4.8 m/s 上昇。

> ⚠ `control.cpp` は **CRLF** ファイル。Python のテキストモードで書き戻すと
> 全行が LF に変換されて差分が 2282 行になる。**バイナリモードで置換すること。**
> 実際に一度やらかして復元した。

### 2. WebUI（依存ゼロ）

この venv に FastAPI も websockets も three.js も無い。**可視化のために依存を増やさない**方針で:

- サーバ: 標準ライブラリの `http.server` + **Server-Sent Events**（ブラウザ標準、ライブラリ不要）
- 3D: **Canvas 2D に自前投影**（three.js を CDN から読むとオフラインで壊れる）
- 入力: キーボード + 仮想スティック → `POST /input`（30 Hz）

起動時にウォームアップ 6000 tick を最高速で早送りしてから実時間ペースに移る
（実時間だと 15 秒待たされるため）。

**キーは「タップ」と「押しっぱなし」の両方で効かせた。**
押しっぱなしのみだと、素早い操作や自動テストで一切動かない（実際に踏んだ）。

## 検証結果

**M4 全項目 PASS。** M2/M3 も回帰なし。

| 項目 | 結果 |
|---|---|
| 起動時ウォームアップ早送り | Arm_flag=3, t=16.2s |
| 静的配信 / `/state` | OK |
| POST でアーム | arm=2 |
| エルロン → 目標ロール → 追従 | phi=6.93 / ref=8.62 deg |
| SSE | 20 件/秒 |
| `common_duty` 0.177 で回る | 修正の効果を確認 |
| `common_duty` 0.044 で停止 | 閾値どおり |

## 発見（未解決）

### A. 姿勢制御の閾値だけ旧基準のまま残っている

停止判定は `common_duty` 基準になったが、**姿勢ゼロ点の取り直しを止める判定は未修正**:

```c
control.cpp:1319  if (T_ref / BATTERY_VOLTAGE < Flight_duty)   // 0.05 のまま
```

結果、**2 つの閾値が別基準**になった:

| | 条件 | スロットル比 |
|---|---|---|
| モータ始動（修正済） | `common_duty >= 0.1` | **0.045** 以上 |
| 姿勢制御が生きる（未修正） | `T_ref/BATTERY >= 0.05` | **0.100** 以上 |

→ **スロットル比 0.045〜0.100 の帯域では、モータは回るが `Phi_bias = Phi` を
毎周期実行し続けるため姿勢制御が効かない。** SIL で実際に、この帯域では
ロール 72° から復帰しないことを確認した。

**実機で試す前に、この帯域を潰すか判断が要る。**
`Flight_duty` も `common_duty` 基準（例: `common_duty < Flight_duty*BATTERY*T_ref_coe`
相当、あるいは単純に `common_duty < 0.1`）に揃えるのが素直だが、
**ユーザー指示は停止判定のみだったので Flight_duty には手を付けていない。**

### B. ファームがどの git にも追跡されていない

`Benzen_new` も `Benzen_NMPC` も、**pico-sdk サブモジュール内の未追跡ディレクトリ**である
（`git ls-files` が両方 0 件）。親リポジトリからも、サブモジュールからも追跡されていない。

→ M3 のログが記録していた `firmware_git` は **pico-sdk 自体の HEAD** であり、
ファームの版を全く特定できていなかった。**再現性の主張が成立していなかった。**

修正: ファームの `*.cpp/*.hpp/*.c/*.h` の**内容ハッシュ (sha256 先頭16桁)** を記録する形に変えた
(`firmware_source_sha256`)。未追跡である以上、これが実質の版識別子になる。
`pico_sdk_git` は参考情報として別キーに退避した。

## 次に触る人へ

- ⚠ **`control.cpp` は CRLF。** 編集はバイナリモードで行うこと。
- ⚠ **ファームは版管理されていない。** 変更前にバックアップを取ること。
  SIL のログには `firmware_source_sha256` が残るので、後から突き合わせは可能。
- ⚠ **A の帯域（スロットル 0.045〜0.100）は実機で危険。** 判断を保留したまま飛ばさないこと。
- WebUI は `python3 host/webserver.py --port 8420`。`--speed` で実時間倍率を変えられる。
- 関連: [20260825_sil_m3_logging.md](20260825_sil_m3_logging.md),
  [20260825_sil_m2_closed_loop.md](20260825_sil_m2_closed_loop.md)
