#!/usr/bin/env python3
"""Bounded FOC speed start from rest for the calibrated Ghost Gen2.4.2 wheel.

Uses the existing CANSimple UART Bus. Configuration changes are RAM-only.
The operator must select --run explicitly; --check-only is read-only.
"""

import argparse
import math
import time

from cansimple_uart import AXIS_STATE, Bus
from ghost_foc_current_probe import NODE, preflight
from ghost_foc_speed_sweep import stop, write_checked


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check-only", action="store_true")
    mode.add_argument("--run", action="store_true")
    ap.add_argument("--direction", choices=("forward", "reverse"), default="forward")
    ap.add_argument("--target-tps", type=float, default=0.5)
    ap.add_argument("--current-cap-a", type=float, default=1.0)
    ap.add_argument("--duration-s", type=float, default=1.2)
    args = ap.parse_args()
    if (not 0.2 <= args.target_tps <= 1.0
            or not 0.2 <= args.current_cap_a <= 1.5
            or not 0.2 <= args.duration_s <= 2.0):
        ap.error("target, current cap or duration outside the standstill-test range")

    direction = 1 if args.direction == "forward" else -1
    bus = Bus(args.port, 115200, timeout=0.25)
    saved = None
    try:
        start_pos, _ = preflight(bus)
        saved = {name: bus.read("hoverboard.config." + name)
                 for name in ("drivemode", "posrpm", "focimax")}
        start_odom = bus.read("hoverboard.odometer")
        sinekv = bus.read("hoverboard.config.sinekv")
        if (saved != {"drivemode": 1, "posrpm": 65535, "focimax": 400}
                or start_odom is None or sinekv != 604):
            raise RuntimeError("unexpected baseline config or odometer")
        print("START_PREFLIGHT", "odometer", start_odom, "sinekv", sinekv,
              "saved", saved, "direction", args.direction,
              "target_tps", args.target_tps,
              "current_cap_a", args.current_cap_a, flush=True)
        if args.check_only:
            print("CHECK_ONLY: no configuration or motor command sent", flush=True)
            return

        write_checked(bus, "focimax", round(args.current_cap_a * 10))
        write_checked(bus, "posrpm", 120)
        write_checked(bus, "drivemode", 9)  # existing FOC_SPEED mode
        bus.set_input_vel(NODE, 0.0)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        time.sleep(0.18)  # FOC current-offset calibration with the wheel stopped
        h = bus.heartbeat(NODE)
        vi = bus.get_bus_voltage_current(NODE)
        pv = bus.get_encoder_estimates(NODE)
        iq = bus.get_iq(NODE)
        zero_odom = bus.read("hoverboard.odometer")
        print("FOC_ZERO", "heartbeat", h, "bus", vi,
              "encoder", pv, "iq", iq, "odometer", zero_odom, flush=True)
        if (h is None or h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]
                or vi is None or not 34.0 <= vi[0] <= 43.0
                or pv is None or abs(pv[1]) > 0.1
                or iq is None or not all(map(math.isfinite, iq))
                or abs(iq[1]) > 0.2 or zero_odom != start_odom):
            raise RuntimeError("stationary FOC zero-current check failed")

        target = direction * args.target_tps
        bus.set_input_vel(NODE, target)
        began = time.monotonic()
        print("ARMED", "target_tps", target,
              "current_cap_a", args.current_cap_a,
              "duration_s", args.duration_s, flush=True)
        result = "time_complete"
        samples = 0
        max_forward_steps = 0
        while time.monotonic() - began < args.duration_s:
            bus.set_input_vel(NODE, target)
            time.sleep(0.008)
            h = bus.heartbeat(NODE)
            vi = bus.get_bus_voltage_current(NODE)
            pv = bus.get_encoder_estimates(NODE)
            iq = bus.get_iq(NODE)
            odom = bus.read("hoverboard.odometer")
            elapsed = time.monotonic() - began
            steps = None if odom is None else direction * (odom - start_odom)
            travel = None if pv is None else direction * (pv[0] - start_pos)
            print("SAMPLE", round(elapsed, 3), "heartbeat", h,
                  "bus", vi, "encoder", pv, "iq", iq,
                  "hall_steps", steps, "travel_turns", travel, flush=True)
            samples += 1
            if (h is None or vi is None or pv is None or iq is None or steps is None
                    or not all(map(math.isfinite, (*vi, *pv, *iq)))):
                raise RuntimeError("telemetry lost")
            if h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]:
                raise RuntimeError("firmware fault or axis disarmed")
            if (not 34.0 <= vi[0] <= 43.0 or abs(vi[1]) > 3.0
                    or direction * pv[1] < -0.2 or abs(pv[1]) > 2.0
                    or steps < -1 or abs(iq[1]) > args.current_cap_a * 1.8 + 0.5):
                raise RuntimeError("voltage, current or motion outside standstill-test range")
            max_forward_steps = max(max_forward_steps, steps)
            if steps >= 12 or travel >= 0.25:
                result = "travel_limit"
                break
        print("RESULT", result, "samples", samples,
              "max_forward_hall_steps", max_forward_steps, flush=True)
    finally:
        try:
            if args.run and saved is not None:
                stop(bus, saved)
        finally:
            bus.s.close()


if __name__ == "__main__":
    main()
