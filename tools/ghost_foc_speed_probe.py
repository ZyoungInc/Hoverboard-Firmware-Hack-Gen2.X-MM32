#!/usr/bin/env python3
"""Short CANSimple FOC speed trial for the verified Ghost Gen2.4.2 board.

--run is explicit. The motor-current cap and speed limit are RAM-only; the
saved 40 A / 50 A profile is restored after IDLE. The script never saves Flash.
"""

import argparse
import math
import time

from cansimple_uart import AXIS_STATE, Bus
from ghost_foc_current_probe import NODE, preflight


def stop(bus, estop=False):
    if estop:
        bus.estop(NODE)
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_input_vel(NODE, 0.0)
    bus.set_input_torque(NODE, 0.0)
    for name, value in (("drivemode", 1), ("focimax", 400), ("posrpm", 65535)):
        if not bus.write("hoverboard.config." + name, value):
            print("RESTORE_FAILED", name, flush=True)
    print("STOP", bus.heartbeat(NODE), "error", bus.get_error(NODE),
          "bus", bus.get_bus_voltage_current(NODE),
          "encoder", bus.get_encoder_estimates(NODE),
          "mode", bus.read("hoverboard.config.drivemode"),
          "focimax", bus.read("hoverboard.config.focimax"),
          "posrpm", bus.read("hoverboard.config.posrpm"), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--run", action="store_true")
    group.add_argument("--check-only", action="store_true")
    group.add_argument("--stop", action="store_true", help="ESTOP, IDLE and restore RAM limits")
    ap.add_argument("--speed-turns-s", type=float, default=0.25)
    ap.add_argument("--duration-s", type=float, default=3.0)
    ap.add_argument("--imax-a", type=float, default=2.0,
                    help="temporary FOC phase-current request cap (A)")
    args = ap.parse_args()
    if not 0.1 <= abs(args.speed_turns_s) <= 0.5:
        ap.error("speed magnitude must be 0.1..0.5 turn/s")
    if not 0.2 <= args.duration_s <= 5.0:
        ap.error("duration must be 0.2..5.0 s")
    if not 1.0 <= args.imax_a <= 5.0:
        ap.error("temporary current cap must be 1..5 A")

    bus = Bus(args.port, 115200, timeout=0.25)
    changed = False
    try:
        if args.stop:
            stop(bus, estop=True)
            return
        start_pos, _ = preflight(bus)
        posrpm = bus.read("hoverboard.config.posrpm")
        print("SPEED_PREFLIGHT", "posrpm", posrpm, flush=True)
        if posrpm != 65535:
            raise RuntimeError("unexpected velocity limit; no motor command sent")
        if args.check_only:
            print("CHECK_ONLY: no configuration or motor command sent", flush=True)
            return
        changed = True
        cap_word = round(args.imax_a * 10)
        for name, value in (("focimax", cap_word), ("posrpm", 60), ("drivemode", 9)):
            if not bus.write("hoverboard.config." + name, value):
                raise RuntimeError("could not set RAM " + name)
            if bus.read("hoverboard.config." + name) != value:
                raise RuntimeError("RAM readback failed for " + name)
        bus.set_input_vel(NODE, args.speed_turns_s)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        print("ARMED", args.speed_turns_s, "turn/s, RAM current cap", args.imax_a,
              "A, at most", args.duration_s, "s", flush=True)
        started = time.monotonic()
        result = "time_complete"
        samples = 0
        while time.monotonic() - started < args.duration_s:
            time.sleep(0.015)
            bus.set_input_vel(NODE, args.speed_turns_s)
            h = bus.heartbeat(NODE)
            vi = bus.get_bus_voltage_current(NODE)
            pv = bus.get_encoder_estimates(NODE)
            iq = bus.get_iq(NODE)
            elapsed = time.monotonic() - started
            delta = None if pv is None else pv[0] - start_pos
            print("SAMPLE", round(elapsed, 3), "heartbeat", h, "bus", vi,
                  "encoder", pv, "iq", iq, "travel", delta, flush=True)
            samples += 1
            if (h is None or vi is None or pv is None or iq is None
                    or not all(map(math.isfinite, pv))):
                result = "telemetry_lost"
                break
            if h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]:
                result = "firmware_fault_or_disarm"
                break
            if (not 34.0 <= vi[0] <= 43.0 or abs(vi[1]) > 3.0
                    or abs(delta) > 1.0 or abs(pv[1]) > 1.5
                    or math.isfinite(iq[1]) and abs(iq[1]) > args.imax_a * 1.8 + 0.5):
                result = "measurement_limit"
                break
        print("RESULT", result, "samples", samples, flush=True)
    finally:
        try:
            if changed:
                stop(bus)
        finally:
            bus.s.close()


if __name__ == "__main__":
    main()
