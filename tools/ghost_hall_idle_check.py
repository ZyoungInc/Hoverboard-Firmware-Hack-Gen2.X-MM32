#!/usr/bin/env python3
"""Read-only Hall odometer check while a Gen2.4.2 wheel is turned by hand.

Run with --sample, then turn the suspended wheel one full revolution in one
direction during the six-second window. This program sends no motor or config
commands and requires the axis to stay IDLE.
"""

import argparse
import time

from cansimple_uart import AXIS_STATE, Bus


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--sample", action="store_true", help="sample for six seconds")
    args = ap.parse_args()
    bus = Bus(args.port, 115200, timeout=0.2)
    try:
        hb = bus.heartbeat(1)
        vi = bus.get_bus_voltage_current(1)
        pv = bus.get_encoder_estimates(1)
        start = bus.read("hoverboard.odometer")
        print("PREFLIGHT", hb, "bus", vi, "encoder", pv, "odometer", start, flush=True)
        if (hb is None or hb["state"] != AXIS_STATE["IDLE"] or hb["errors"]
                or vi is None or pv is None or start is None):
            raise RuntimeError("axis is not ready for a read-only Hall check")
        if not args.sample:
            print("CHECK_ONLY: no configuration or motor command sent", flush=True)
            return

        print("TURN WHEEL BY HAND ONE REVOLUTION, SAME DIRECTION, NOW", flush=True)
        last = start
        first_direction = 0
        along = against = 0
        end = time.monotonic() + 6.0
        last_hb = time.monotonic()
        while time.monotonic() < end:
            odom = bus.read("hoverboard.odometer")
            if odom is None:
                raise RuntimeError("UART odometer read lost")
            d = odom - last
            if d:
                if not first_direction:
                    first_direction = 1 if d > 0 else -1
                if d * first_direction > 0:
                    along += abs(d)
                else:
                    against += abs(d)
                speed = bus.read("hoverboard.fast_speed")
                print("EDGE", "t", round(6.0 - (end - time.monotonic()), 3),
                      "odometer", odom, "delta", d, "fast_rpm", speed, flush=True)
                last = odom
            if time.monotonic() - last_hb >= 0.25:
                hb = bus.heartbeat(1)
                if hb is None or hb["state"] != AXIS_STATE["IDLE"]:
                    raise RuntimeError("axis left IDLE; sampling stopped")
                last_hb = time.monotonic()
            time.sleep(0.02)
        pv = bus.get_encoder_estimates(1)
        print("SUMMARY", "net_steps", last - start, "along_steps", along,
              "reverse_steps", against, "expected_steps_per_turn", 48,
              "encoder", pv, "heartbeat", bus.heartbeat(1), flush=True)
    finally:
        bus.s.close()


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None
