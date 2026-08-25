"""SIL の実行ログ。

方針: **取れるものは全部取る。間引きは後段でやる。**

形式は `.npz`（numpy 標準の圧縮列指向）を主とする。
計画書では Parquet と書いたが、この環境の venv に pandas / pyarrow が無く、
**依存を増やさない方を優先**した。`.npz` は numpy だけで読め、
列ごとに float32 で保存するので実用上のサイズも近い。
CSV は `--csv` で追加出力できる（人が覗く用）。

1 実行 = 1 ディレクトリ:
    runs/run_YYYYMMDD_HHMMSS/
        data.npz    列データ
        meta.json   再現に必要な情報（ファームの git hash、機体パラメータ等）
        data.csv    （任意）
"""
from __future__ import annotations

import json
import subprocess
from dataclasses import asdict, is_dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

DEFAULT_ROOT = Path(__file__).resolve().parent.parent / "runs"


def _firmware_source_hash(fw_dir: Path) -> dict:
    """ファームのソース内容ハッシュ。

    ⚠ `Benzen_new` / `Benzen_NMPC` は **どの git にも追跡されていない**
    （pico-sdk サブモジュール内の未追跡ディレクトリ）。
    したがって git hash ではファームの版を特定できない。
    代わりに**ソースの内容ハッシュ**を取る。これが実質的な版識別子になる。
    """
    import hashlib
    h = hashlib.sha256()
    files = []
    for pat in ("*.cpp", "*.hpp", "*.c", "*.h"):
        files.extend(sorted(fw_dir.glob(pat)))
    for f in files:
        h.update(f.name.encode())
        h.update(f.read_bytes())
    return {"sha256": h.hexdigest()[:16], "files": len(files)}


def _git_hash(fw_dir: Path) -> str | None:
    """その場所を含む git リポジトリの HEAD。"""
    try:
        out = subprocess.run(
            ["git", "-C", str(fw_dir), "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=5,
        )
        return out.stdout.strip() or None
    except Exception:
        return None


class RunLogger:
    """列指向のレコーダ。dict を append していくだけ。"""

    def __init__(self):
        self.cols: dict[str, list] = {}
        self.n = 0

    def append(self, row: dict) -> None:
        for k, v in row.items():
            if k not in self.cols:
                # 途中から現れた列は、それまでを NaN で埋める
                self.cols[k] = [float("nan")] * self.n
            self.cols[k].append(v)
        # この行に無かった列も NaN で伸ばす
        for k, col in self.cols.items():
            if len(col) == self.n:
                col.append(float("nan"))
        self.n += 1

    def as_arrays(self) -> dict[str, np.ndarray]:
        return {k: np.asarray(v, dtype=np.float32) for k, v in self.cols.items()}

    # --- 保存 -------------------------------------------------------------
    def save(self, meta: dict, root: Path | None = None,
             name: str | None = None, csv: bool = False) -> Path:
        root = Path(root) if root else DEFAULT_ROOT
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = root / (name or f"run_{stamp}")
        run_dir.mkdir(parents=True, exist_ok=True)

        arrays = self.as_arrays()
        np.savez_compressed(run_dir / "data.npz", **arrays)

        meta = dict(meta)
        meta.setdefault("created", datetime.now().isoformat(timespec="seconds"))
        meta.setdefault("rows", self.n)
        meta.setdefault("columns", sorted(arrays.keys()))
        (run_dir / "meta.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False, default=str), encoding="utf-8")

        if csv:
            keys = sorted(arrays.keys())
            keys = ["t"] + [k for k in keys if k != "t"] if "t" in keys else keys
            with (run_dir / "data.csv").open("w", encoding="utf-8") as f:
                f.write(",".join(keys) + "\n")
                mat = np.column_stack([arrays[k] for k in keys])
                for r in mat:
                    f.write(",".join(f"{v:.6g}" for v in r) + "\n")
        return run_dir


def build_meta(plant_params, throttle=None, note=None, fw_dir=None, extra=None) -> dict:
    """再現に要る情報を集める。**ここを省くと後でログの意味が分からなくなる。**"""
    fw_dir = Path(fw_dir) if fw_dir else (
        Path(__file__).resolve().parents[2] / "pico/pico-sdk2/pico-sdk/Benzen_new")
    import firmware_consts as fc
    fw_hash = _firmware_source_hash(fw_dir)
    meta = {
        "note": note,
        "throttle": throttle,
        "firmware_dir": str(fw_dir),
        # ファームは未追跡なのでソース内容ハッシュが実質の版識別子
        "firmware_source_sha256": fw_hash["sha256"],
        "firmware_source_files": fw_hash["files"],
        # 参考: pico-sdk サブモジュールの HEAD（ファームの版ではない）
        "pico_sdk_git": _git_hash(fw_dir),
        "sil_git": _git_hash(Path(__file__).resolve().parents[2]),
        "plant_params": asdict(plant_params) if is_dataclass(plant_params) else str(plant_params),
        "firmware_consts": {
            k: getattr(fc, k) for k in
            ("BATTERY_VOLTAGE", "T_REF_COE", "T_REF_THROTTLE_GAIN",
             "DISABLE_DUTY", "FLIGHT_DUTY", "MIN_DUTY", "MAX_DUTY")
        },
    }
    if extra:
        meta.update(extra)
    return meta


def load(run_dir) -> tuple[dict[str, np.ndarray], dict]:
    """保存したランを読み戻す。"""
    run_dir = Path(run_dir)
    with np.load(run_dir / "data.npz") as z:
        data = {k: z[k] for k in z.files}
    meta = json.loads((run_dir / "meta.json").read_text(encoding="utf-8"))
    return data, meta


def latest_run(root: Path | None = None) -> Path:
    root = Path(root) if root else DEFAULT_ROOT
    runs = sorted(p for p in root.glob("run_*") if (p / "data.npz").exists())
    if not runs:
        raise FileNotFoundError(f"ランが見つかりません: {root}")
    return runs[-1]
