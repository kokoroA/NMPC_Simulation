"""実機ファーム(Benzen_new)の定数をホスト側に写したもの。

**ここは control.cpp / radio.hpp の写しであり、勝手に変えてはいけない。**
実機を変えたらここも直し、乖離させるなら理由を decisions に残すこと。
"""

# --- control.cpp -----------------------------------------------------------
BATTERY_VOLTAGE = 7.4        # control.hpp: #define BATTERY_VOLTAGE (7.4)
T_REF_COE = 0.6              # control.cpp:112  float T_ref_coe = 0.6
DISABLE_DUTY = 0.1           # control.cpp:149  モータが回り始める閾値
FLIGHT_DUTY = 0.05           # control.cpp:150  姿勢ゼロ点取得を止める閾値
MIN_DUTY = DISABLE_DUTY      # control.cpp:1153 minimum_duty = Disable_duty
MAX_DUTY = 0.95              # control.cpp:1152

# control.cpp:995（rate_control 内・400Hz）が実効。
#   T_ref = 0.5 * BATTERY_VOLTAGE * (Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN)
#   T_ref = T_ref * (1 + g_thrust_delta_ratio)
# control.cpp:833 の 0.7 係数は高度制御ブロック側で、こちらに上書きされる。
T_REF_THROTTLE_GAIN = 0.5

# set_duty_*() のクリップ（pwm.cpp）
DUTY_CLIP_MIN = 0.01
DUTY_CLIP_MAX = 0.97

# --- 便利関数 --------------------------------------------------------------


def t_ref_from_throttle(ratio, thrust_delta_ratio=0.0):
    """スロットル比(0..1) -> T_ref。"""
    return T_REF_THROTTLE_GAIN * BATTERY_VOLTAGE * ratio * (1.0 + thrust_delta_ratio)


def base_duty_from_throttle(ratio, thrust_delta_ratio=0.0):
    """姿勢補正なしの各モータ duty（ミキサの T_ref*T_ref_coe 項）。"""
    return t_ref_from_throttle(ratio, thrust_delta_ratio) * T_REF_COE


def motors_running(ratio, thrust_delta_ratio=0.0):
    """モータが回るか（T_ref/BATTERY >= Disable_duty）。"""
    return t_ref_from_throttle(ratio, thrust_delta_ratio) / BATTERY_VOLTAGE >= DISABLE_DUTY


def attitude_live(ratio, thrust_delta_ratio=0.0):
    """姿勢ゼロ点の取り直しが止まるか（T_ref/BATTERY >= Flight_duty）。"""
    return t_ref_from_throttle(ratio, thrust_delta_ratio) / BATTERY_VOLTAGE >= FLIGHT_DUTY


# --- 使えるスロットル帯 ----------------------------------------------------
# 2026-08-25 の control.cpp 修正で停止判定が単位整合した:
#   旧: T_ref / BATTERY_VOLTAGE < Disable_duty   (電圧比と duty を比較していた)
#   新: common_duty = T_ref * T_ref_coe;  common_duty < Disable_duty
# これにより duty 下限が 0.444 -> 0.100 に下がり、ホバー点が到達可能になった。
_DUTY_PER_RATIO = T_REF_THROTTLE_GAIN * BATTERY_VOLTAGE * T_REF_COE   # = 2.22
THROTTLE_MOTORS_ON = DISABLE_DUTY / _DUTY_PER_RATIO                   # = 0.045
THROTTLE_DUTY_SATURATES = MAX_DUTY / _DUTY_PER_RATIO                  # = 0.428
DUTY_FLOOR = DISABLE_DUTY                                             # = 0.100
