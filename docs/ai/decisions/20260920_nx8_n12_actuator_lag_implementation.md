# NX=8 / N=12 の一次遅れ実装では observer を400 Hzにつき一度だけ進める

- **日付**: 2026-09-20
- **種別**: implementation
- **対象**: `Benzen_NMPC/nmpc_model.*`, `nmpc_params.hpp`, `nmpc.cpp`, `control.cpp`, `wifi_telemetry.*`, `tools/nmpc_udp_client.py`, `program/NMPC_simulation/nmpc_simulation.py`
- **タグ**: #Benzen_NMPC #NX8 #N12 #一次遅れ #exactZOH #observer #multicore #telemetry

## 背景 / 判断

6モータ状態をそのまま持つと計算量とSRAMを増やす一方、共通時定数なら姿勢軸に必要な情報は
`a=B_attω` の3状態へ厳密に縮約できる。したがって
`x=[φ,θ,p,q,r,a_p,a_q,a_r]`、`tau_act=45 ms`、アクチュエータ部 exact ZOH を採用した。
`tau_act` は同定済み真値ではないので、初版では `Q_a=0` としゲイン調整を混ぜない。

一次遅れを予測モデルへ入れた状態で従来のDC6を残すと同じ実効遅れを二重に補償するため、
`delay_comp_steps=0` を初期値とした。入力最適化次元とフル再構築時間を抑えるため、
中間のNX=8/N=14を経て最終値をN=12とした。

## 実装中に判明した理由付き修正

`motor_stop()` 内でobserverを更新すると、`failsafe_check()`と400 Hz出力分岐の双方から
同じ周期に呼ばれた場合、ゼロ入力への減衰が2回進む。その結果、停止時だけ見かけの時定数が
半分になる。observer更新は副作用を持つ`motor_stop()`から外し、400 Hz出力段の末尾で
最終PWM duty（停止時はゼロ）を使って**周期ごとに必ず一度**だけ実行する。

Python側は健全時の共通baselineなら`B_att·1=0`で省略できるが、故障時のbaselineは
モータごとに異なり省略できない。NX=8でもrolloutと線形化には`Δu+u_baseline`を渡す。

ホストの`EIGEN_RUNTIME_NO_MALLOC`検査により、Eigen標準の`LLT::rankUpdate()`が
内部で`Matrix<Scalar,Dynamic,1>`を確保することが判明した。固定サイズの`HMat`を使っていても
この一時ベクトルは動的確保されるため、実機の`EIGEN_NO_MALLOC`契約に反する。
そこで既存の`sr2_p/sr2_q`を作業領域として破壊的に使う固定長Cholesky
update/downdateへ置換した。通常周期の計算量O(NU_TOTAL²)は変わらない。

## 検証結果

- NX=8/N=14のPico 2 Wクロスビルドは成功。
- NX=8/N=14 map: `.text=0x62e64`, `.data=0x3440`, `.bss=0x6def8`。
  data+bssは463,672 Bで、RP2350 SRAM 520 KiB内に収まる。
- 最終NX=8/N=12のfreshクロスビルド成功:
  `text=691,344 B`, `data=0 B`, `bss=417,720 B`。
- ホストC++回帰はT1–T6とsolver finite/SPD/box/ADMM/no-mallocをすべてPASS。
- Python NX=8サニティチェックは全項目PASS。
- Pico実測時間、通電・飛行試験は未検証。

## 次に触る人へ

- 起動ヘッダで`NX=8,N=12,NX_TOTAL=96,NU_TOTAL=72,tau_act=0.045,delay_comp=0`を確認する。
- プロペラなしで`iter_us/loop_us`、map、`a_hat`の時定数を測るまで飛行へ進まない。
- `tau_act`を変更する場合は35/45/60 ms格子を再実行する。
- `a_hat`はRPM推定ではなくPWM指令observerなので、未知故障の実推力低下は直接観測できない。
