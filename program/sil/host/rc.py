"""RC チャンネル（Chdata[18]）の生成。

radio.hpp の定義:
    CH1: 368..1680   CH2: 368..1680   CH3: 368..1638   CH4: 368..1638
control.cpp の使い方:
    Chdata[0]=ラダー  Chdata[1]=エレベータ  Chdata[2]=スロットル  Chdata[3]=エルロン
"""
from __future__ import annotations

CH_MIN = [368, 368, 368, 368, 368, 368]
CH_MAX = [1680, 1680, 1638, 1638, 1638, 1638]

RUDDER, ELEVATOR, THROTTLE, AILERON = 0, 1, 2, 3


def _mid(i: int) -> int:
    return (CH_MIN[i] + CH_MAX[i]) // 2


def neutral() -> list[int]:
    """スティック中立・スロットル最低。"""
    ch = [0] * 18
    ch[RUDDER] = _mid(0)
    ch[ELEVATOR] = _mid(1)
    ch[THROTTLE] = CH_MIN[2]
    ch[AILERON] = _mid(3)
    for i in range(4, 6):
        ch[i] = _mid(i)
    return ch


def arm_gesture() -> list[int]:
    """lock_com() を成立させるスティック位置。

    control.cpp lock_com():
        Chdata[2] < CH3MIN+400 (=768) かつ Chdata[0] > CH1MAX-400 (=1280)
        かつ Chdata[3] < CH4MIN+400 (=768) かつ Chdata[1] > CH2MAX-400 (=1280)
    を 50 回超連続で満たすと state=1。解除すると 0 に戻る。
    """
    ch = neutral()
    ch[THROTTLE] = CH_MIN[2]      # 368  < 768
    ch[RUDDER] = CH_MAX[0]        # 1680 > 1280
    ch[AILERON] = CH_MIN[3]       # 368  < 768
    ch[ELEVATOR] = CH_MAX[1]      # 1680 > 1280
    return ch


def with_throttle(ch: list[int], ratio: float) -> list[int]:
    """スロットルを 0..1 の比率で設定した複製を返す。"""
    out = list(ch)
    lo, hi = CH_MIN[2], CH_MAX[2]
    out[THROTTLE] = int(lo + (hi - lo) * max(0.0, min(1.0, ratio)))
    return out
