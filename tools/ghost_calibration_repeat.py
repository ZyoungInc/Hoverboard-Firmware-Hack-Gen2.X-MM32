#!/usr/bin/env python3
"""Repeat the fixed Ghost PinFinder RL and Hall-angle calibrations; save raw runs.

This talks to the existing PinFinder calibration firmware. It neither searches PCB
pins nor saves calibration values to Flash. One failure stops the batch.
"""

import argparse
import json
import statistics
import time
from pathlib import Path

from pinfinder import PinFinder


def number(result, key):
    value = result.get(key)
    return int(value) if isinstance(value, str) else None


def summarize(records):
    for key in ("motorr", "phasel", *(f"hallang{i}" for i in range(1, 7))):
        values = [number(r["results"], key) for r in records]
        values = [v for v in values if v is not None]
        if not values:
            continue
        if key.startswith("hallang"):
            anchor = values[0]
            values = [anchor + ((v - anchor + 32768) % 65536 - 32768) for v in values]
            scale = 360 / 65536
            print(f"SUMMARY {key}: n={len(values)} median={round(statistics.median(values)) % 65536} "
                  f"spread={max(values)-min(values)} counts ({(max(values)-min(values))*scale:.2f} electrical deg)",
                  flush=True)
        else:
            print(f"SUMMARY {key}: n={len(values)} median={statistics.median(values):g} "
                  f"range={min(values)}..{max(values)}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--bus", type=float, default=36.0, help="bench supply voltage, V")
    ap.add_argument("--rl", type=int, default=20)
    ap.add_argument("--hall", type=int, default=10)
    ap.add_argument("--hall-mv", type=int, choices=(1500, 2000, 3000, 4000, 5000), default=2000)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    if not 0 <= args.rl <= 100 or not 0 <= args.hall <= 100 or args.rl + args.hall == 0:
        ap.error("run counts must be 0..100, with at least one run")

    pf = PinFinder(args.port)
    records = []
    try:
        ok, result, lines = pf.cmd("shunt", 10)
        print("PREFLIGHT", lines, flush=True)
        vb = number(result, "bus_mv")
        if not ok or vb is None or not (args.bus - 1.5) * 1000 <= vb <= (args.bus + 1.5) * 1000:
            raise RuntimeError(f"PinFinder bus voltage is not near {args.bus} V")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", encoding="utf-8") as log:
            for command, count, timeout in (("calibrate rl 1500", args.rl, 15),
                                             (f"calibrate hall {args.hall_mv}", args.hall, 35)):
                for run in range(1, count + 1):
                    started = time.monotonic()
                    ok, result, lines = pf.cmd(command, timeout)
                    record = {"command": command, "run": run, "ok": ok,
                              "elapsed_s": round(time.monotonic() - started, 3),
                              "results": result, "lines": lines}
                    records.append(record)
                    log.write(json.dumps(record, ensure_ascii=False) + "\n")
                    log.flush()
                    print("RUN", command, run, "/", count,
                          "OK" if ok else "FAILED", result,
                          "elapsed_s", record["elapsed_s"], flush=True)
                    if not ok and not (command.startswith("calibrate rl")
                                       and lines[-1] == "err no clean current rise: no L"):
                        raise RuntimeError(f"{command} run {run} failed; see {args.output}")
    except KeyboardInterrupt:
        pf.s.write(b"\n")
        pf.s.flush()
        raise
    finally:
        pf.close()
        summarize(records)
        print("RAW_LOG", args.output, flush=True)


if __name__ == "__main__":
    main()
