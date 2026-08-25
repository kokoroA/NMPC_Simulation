"""libbenzen_sil への ctypes バインディング。

実機ファーム(Benzen_new)をホスト上のプロセスとして駆動する。
"""
from __future__ import annotations

import ctypes
import sys
from pathlib import Path

_LIB_CANDIDATES = ("libbenzen_sil.dylib", "libbenzen_sil.so")


def _find_library(build_dir: Path | None = None) -> Path:
    base = build_dir or (Path(__file__).resolve().parent.parent / "build")
    for name in _LIB_CANDIDATES:
        p = base / name
        if p.exists():
            return p
    raise FileNotFoundError(f"SIL ライブラリが見つかりません: {base}")


class Sil:
    """実機ファームの 1 インスタンス。

    注意: グローバル変数を持つ C++ コードなので、1 プロセスに 1 つだけ。
    """

    def __init__(self, build_dir: Path | None = None, verbose: bool = False) -> None:
        self._lib = ctypes.CDLL(str(_find_library(build_dir)))
        self._bind()
        self._lib.sil_set_stdout(ctypes.c_int(1 if verbose else 0))
        rc = self._lib.sil_init()
        if rc != 0:
            raise RuntimeError(f"sil_init() 失敗: rc={rc}")

    def _bind(self) -> None:
        L = self._lib
        f3 = ctypes.c_float * 3
        L.sil_init.restype = ctypes.c_int
        L.sil_set_stdout.argtypes = [ctypes.c_int]
        L.sil_set_imu.argtypes = [f3, f3, f3]
        L.sil_set_range_mm.argtypes = [ctypes.c_uint16]
        L.sil_set_rc.argtypes = [ctypes.c_uint16 * 18]
        L.sil_set_arm_flag.argtypes = [ctypes.c_uint8]
        L.sil_get_duty.argtypes = [ctypes.c_float * 6]
        L.sil_get_state.argtypes = [ctypes.c_float * 24]
        L.sil_get_arm_flag.restype = ctypes.c_uint8
        L.sil_get_flight_mode.restype = ctypes.c_uint8
        L.sil_get_time_us.restype = ctypes.c_uint64
        L.sil_get_last_loop_ns.restype = ctypes.c_uint64
        L.sil_get_last_core1_ns.restype = ctypes.c_uint64

    # --- 入力 -------------------------------------------------------------
    def set_imu(self, acc_mg, gyr_mdps, mag_mgauss) -> None:
        f3 = ctypes.c_float * 3
        self._lib.sil_set_imu(f3(*acc_mg), f3(*gyr_mdps), f3(*mag_mgauss))

    def set_range_mm(self, mm: int) -> None:
        self._lib.sil_set_range_mm(ctypes.c_uint16(int(mm)))

    def set_rc(self, channels) -> None:
        arr = (ctypes.c_uint16 * 18)()
        for i, v in enumerate(channels[:18]):
            arr[i] = int(v)
        self._lib.sil_set_rc(arr)

    def set_arm_flag(self, v: int) -> None:
        self._lib.sil_set_arm_flag(ctypes.c_uint8(v))

    # --- 実行 -------------------------------------------------------------
    def tick(self) -> None:
        """400Hz を 1 tick 進める（仮想時間 +2500us）。"""
        self._lib.sil_tick_400hz()

    # --- 出力 -------------------------------------------------------------
    STATE_FIELDS = (
        "phi", "theta", "psi",
        "phi_ref", "theta_ref", "psi_ref",
        "p", "q", "r",
        "Pref", "Qref", "Rref",
        "P_com", "Q_com", "R_com",
        "T_ref", "T_stick", "altitude",
        "u_fr", "u_fl", "u_mr", "u_ml", "u_rr", "u_rl",
    )

    def duty(self) -> list[float]:
        """[FR, FL, MR, ML, RR, RL]"""
        arr = (ctypes.c_float * 6)()
        self._lib.sil_get_duty(arr)
        return list(arr)

    def state(self) -> dict[str, float]:
        arr = (ctypes.c_float * 24)()
        self._lib.sil_get_state(arr)
        return dict(zip(self.STATE_FIELDS, arr))

    @property
    def arm_flag(self) -> int:
        return self._lib.sil_get_arm_flag()

    @property
    def flight_mode(self) -> int:
        return self._lib.sil_get_flight_mode()

    @property
    def time_us(self) -> int:
        return self._lib.sil_get_time_us()

    DEBUG_FIELDS = ("Ax","Ay","Az","Mx","My","Mz","Xe0","Xe1","Xe2","Xe3","MN","ME","MD","Phi_raw","Theta_raw","Psi_raw",
                    "Phi_bias","Theta_bias","Psi_bias","T_ref")

    def debug(self):
        arr = (ctypes.c_float * 20)()
        self._lib.sil_get_debug(arr)
        return dict(zip(self.DEBUG_FIELDS, arr))

    def sched_counts(self):
        """(core1 起動回数, core1 待ち突入回数)"""
        rel = ctypes.c_uint64(); acq = ctypes.c_uint64()
        self._lib.sil_get_sched_counts(ctypes.byref(rel), ctypes.byref(acq))
        return rel.value, acq.value

    @property
    def last_loop_ns(self) -> int:
        """直近 tick の core0 実行時間（ホスト実測 ns。実機時間ではない）"""
        return self._lib.sil_get_last_loop_ns()

    @property
    def last_core1_ns(self) -> int:
        return self._lib.sil_get_last_core1_ns()
