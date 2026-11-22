#!/usr/bin/env python3
"""Combine Ghost Gen2.4.2 JSON and SPIN05 firmware using the verified config layout."""

import argparse
import json
import struct
from pathlib import Path

from cfgfields import BY_NAME, FIELDS, PINS
from mm32cfgstore import CFG_BASE, EXT_MAGIC, MAGIC, parse_new


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--firmware", type=Path, required=True)
    ap.add_argument("--config", type=Path, required=True)
    ap.add_argument("--template", type=Path, required=True,
                    help="previous verified Ghost combined image, for unset config words")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    code = args.firmware.read_bytes()
    template = args.template.read_bytes()
    offset = CFG_BASE - 0x08000000
    if len(code) > offset or len(template) < offset + 256:
        raise SystemExit("firmware does not fit the 31 KB code area, or invalid template")
    _, words = parse_new(template[offset:offset + 1024])
    if words is None or words[32] != MAGIC or words[96] != EXT_MAGIC:
        raise SystemExit("template has no valid main/extension configuration pair")
    cfg = json.loads(args.config.read_text(encoding="utf-8"))["config"]
    for name, value in cfg.items():
        if name not in BY_NAME:
            raise SystemExit(f"unknown config word {name}")
        index = BY_NAME[name]
        if FIELDS[index][1] == "pin":
            value = PINS.index(value.upper())
        if not isinstance(value, int) or not 0 <= value <= 65535:
            raise SystemExit(f"invalid value for {name}: {value}")
        words[index] = value
    page = struct.pack("<64H", *words[64:]) + struct.pack("<64H", *words[:64])
    image = code.ljust(offset, b"\xff") + page
    slot, decoded = parse_new(image[offset:])
    if slot != 1 or decoded != words:
        raise SystemExit("generated config failed round-trip validation")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(image)
    print(f"image={args.output} bytes={len(image)} code={len(code)} config=0x{CFG_BASE:08x}")
    print("limits", {k: cfg[k] for k in ("focimax", "curpull", "curtrip", "softocp", "hardocp")})


if __name__ == "__main__":
    main()
