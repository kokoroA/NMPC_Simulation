#!/usr/bin/env python3
"""実機 ELF から計算資源（Flash / SRAM）を厳密に算出する。

SIL はホスト上で動くため、メモリ消費はホストのものであって実機のものではない。
資源の「実機との一致」は、ホストで測るのではなく
**実機向けにクロスビルドした ELF を解析して得る**のが唯一正しい方法である。

使い方:
    python3 resource_report.py <path/to/pico_copter.elf> [--top 20]
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

# RP2350 (Pico 2 W)
SRAM_BYTES = 520 * 1024
FLASH_BYTES = 4 * 1024 * 1024
SRAM_BASE = 0x20000000
FLASH_BASE = 0x10000000


@dataclass
class Sym:
    addr: int
    size: int
    kind: str
    name: str

    @property
    def in_sram(self) -> bool:
        return SRAM_BASE <= self.addr < SRAM_BASE + 0x100000

    @property
    def in_flash(self) -> bool:
        return FLASH_BASE <= self.addr < FLASH_BASE + 0x1000000


def _tool(name: str) -> str:
    for cand in (f"arm-none-eabi-{name}", f"llvm-{name}", name):
        if shutil.which(cand):
            return cand
    sys.exit(f"エラー: {name} が見つかりません（arm-none-eabi ツールチェーンが必要）")


def read_size(elf: Path) -> dict[str, int]:
    out = subprocess.run([_tool("size"), str(elf)], capture_output=True, text=True).stdout
    nums = re.findall(r"^\s*(\d+)\s+(\d+)\s+(\d+)", out, re.M)
    if not nums:
        sys.exit(f"size の出力を解釈できません:\n{out}")
    text, data, bss = (int(x) for x in nums[0])
    return {"text": text, "data": data, "bss": bss}


def read_symbols(elf: Path) -> list[Sym]:
    out = subprocess.run(
        [_tool("nm"), "-C", "--size-sort", "-S", "-r", str(elf)],
        capture_output=True, text=True,
    ).stdout
    syms: list[Sym] = []
    for line in out.splitlines():
        m = re.match(r"^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+(\S)\s+(.*)$", line)
        if m:
            syms.append(Sym(int(m.group(1), 16), int(m.group(2), 16), m.group(3), m.group(4)))
    return syms


def bar(used: int, total: int, width: int = 40) -> str:
    filled = min(width, int(width * used / total)) if total else 0
    return "█" * filled + "·" * (width - filled)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("elf", type=Path)
    ap.add_argument("--top", type=int, default=15)
    args = ap.parse_args()
    if not args.elf.exists():
        sys.exit(f"ELF が見つかりません: {args.elf}")

    sz = read_size(args.elf)
    ram = sz["data"] + sz["bss"]
    flash = sz["text"] + sz["data"]

    print(f"=== 実機資源レポート: {args.elf.name} ===\n")
    print(f"SRAM  {ram:>9,} / {SRAM_BYTES:,} B  ({100*ram/SRAM_BYTES:5.1f}%)  [{bar(ram, SRAM_BYTES)}]")
    print(f"Flash {flash:>9,} / {FLASH_BYTES:,} B  ({100*flash/FLASH_BYTES:5.1f}%)  [{bar(flash, FLASH_BYTES)}]")
    print(f"\n  text={sz['text']:,}  data={sz['data']:,}  bss={sz['bss']:,}")

    syms = read_symbols(args.elf)
    ram_syms = [s for s in syms if s.in_sram]

    print(f"\n--- SRAM を消費している上位 {args.top} シンボル ---")
    for s in ram_syms[: args.top]:
        pct = 100 * s.size / SRAM_BYTES
        name = s.name if len(s.name) <= 62 else s.name[:59] + "..."
        print(f"  {s.size:>9,} B  ({pct:4.1f}%)  {name}")

    accounted = sum(s.size for s in ram_syms[: args.top])
    print(f"\n  上位 {args.top} 件で SRAM の {100*accounted/SRAM_BYTES:.1f}% を占める")
    print(f"  残り SRAM: {SRAM_BYTES - ram:,} B")

    if ram > SRAM_BYTES:
        print("\n⚠ SRAM 超過。リンクは通っても実機で動かない。")
    elif ram > SRAM_BYTES * 0.85:
        print("\n⚠ SRAM 使用率が 85% を超えている。スタック余裕を確認すること。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
