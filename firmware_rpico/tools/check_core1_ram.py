#!/usr/bin/env python3
"""
Check that the core 1 code does not use flash (core 1 has no XIP access after its init).

It follows all direct calls from the given root functions in the disassembly, and reports:
- a call or branch to an address in flash,
- a literal pool word with a flash address (for example a const table in .rodata).
Indirect calls are not followed: give such functions as roots.

Usage: check_core1_ram.py <elf> [root ...]
"""
import re
import subprocess
import sys

DEFAULT_ROOTS = [
    "smol_servo_loop_irq_handler",
    "smol_bridge_pwm_wrap_irq_handler",
    "smol_adc_fifo_threshold_irq_handler",
    "smol_servo_loop_idle_core1",
    "_exception_dummy_handler",
]

def is_flash(addr):
    return 0x10000000 <= addr < 0x12000000

def main():
    elf = sys.argv[1]
    roots = sys.argv[2:] or DEFAULT_ROOTS
    dis = subprocess.run(["arm-none-eabi-objdump", "-d", "--no-show-raw-insn", elf], capture_output=True, text=True, check=True).stdout
    funcs = {}
    cur = None
    for line in dis.splitlines():
        m = re.match(r"^([0-9a-f]{8}) <([^>]+)>:", line)
        if m:
            cur = m.group(2)
            funcs[cur] = {"addr": int(m.group(1), 16), "lines": []}
            continue
        if cur and line.strip():
            funcs[cur]["lines"].append(line)

    missing = [r for r in roots if r not in funcs]
    stack = [r for r in roots if r in funcs]
    seen = set()
    problems = []
    while stack:
        f = stack.pop()
        if f in seen:
            continue
        seen.add(f)
        if is_flash(funcs[f]["addr"]):
            problems.append(f"{f} is in flash")
            continue
        for line in funcs[f]["lines"]:
            m = re.search(r"\s(bl|blx|b\.w|b)\s+([0-9a-f]+) <([^>+]+)", line)
            if m:
                target = int(m.group(2), 16)
                name = m.group(3)
                if is_flash(target):
                    problems.append(f"{f}: {m.group(1)} to flash {name} @{target:08x}")
                elif name != f and name in funcs:
                    stack.append(name)
            m = re.search(r"\.word\s+0x([0-9a-f]{8})", line)
            if m and is_flash(int(m.group(1), 16)):
                problems.append(f"{f}: literal flash address 0x{m.group(1)}")

    if missing:
        print("roots not found:", " ".join(missing))
    print(f"checked {len(seen)} functions")
    if problems:
        print("PROBLEMS:")
        for p in problems:
            print("  " + p)
        return 1
    print("no flash references")
    return 0

if __name__ == "__main__":
    sys.exit(main())
