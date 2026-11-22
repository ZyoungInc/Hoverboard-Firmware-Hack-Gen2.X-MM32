#!/usr/bin/env python3
"""Short block-voltage pulse for a calibrated Ghost Gen2.4.2 motor.

The user must explicitly pass --pulse. The test changes control mode in RAM only,
uses a 7% bus-voltage command for at most 0.5 s, and restores IDLE/block speed.
Keep the wheel suspended and the 42 V bench supply limited to 0.5 A.
"""

import argparse
import math
import time

from cansimple_uart import AXIS_STATE, Bus, CONTROL_MODE, INPUT_MODE


NODE = 1
FRACTION = 0.07
DURATION = 0.5
EXPECTED = {
    "drivemode": 1, "halla": 27, "hallb": 26, "hallc": 28,
    "hallinvert": 1, "windings": 16, "vbatgain": 30370,
    "itotaldivider": 250, "softocp": 2, "hardocp": 8,
}


def stop(bus):
    # IDLE first: a zero command while still armed can actively brake the wheel.
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_input_voltage(NODE, 0.0)
    bus.set_controller_mode(NODE, CONTROL_MODE["VELOCITY_CONTROL"], INPUT_MODE["PASSTHROUGH"])
    bus.set_input_vel(NODE, 0.0)
    time.sleep(0.05)
    print("STOP", "heartbeat", bus.heartbeat(NODE), "bus", bus.get_bus_voltage_current(NODE),
          "drivemode", bus.read("hoverboard.config.drivemode"), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--pulse", action="store_true", help="perform the bounded 7% pulse")
    group.add_argument("--check-only", action="store_true", help="read-only preflight; do not change mode")
    group.add_argument("--stop-only", action="store_true", help="send IDLE and restore block speed mode")
    args = ap.parse_args()
    bus = Bus(args.port, 115200, timeout=0.2)
    if args.stop_only:
        try:
            stop(bus)
        finally:
            bus.s.close()
        return
    mode_changed = False
    try:
        h = bus.heartbeat(NODE)
        vi = bus.get_bus_voltage_current(NODE)
        pv = bus.get_encoder_estimates(NODE)
        errors = bus.get_error(NODE)
        cfg = {k: bus.read("hoverboard.config." + k) for k in EXPECTED}
        print("PREFLIGHT", h, "bus", vi, "encoder", pv, "errors", errors,
              "config", cfg, flush=True)
        if (h is None or h["state"] != AXIS_STATE["IDLE"] or h["errors"]
                or errors is None or errors["active_errors"] or errors["disarm_reason"]
                or vi is None or not 41.0 <= vi[0] <= 43.0 or abs(vi[1]) > 0.1
                or pv is None or not all(map(math.isfinite, pv)) or abs(pv[1]) > 0.05
                or cfg != EXPECTED):
            raise RuntimeError("preflight failed; bridge was not armed")
        if args.check_only:
            print("CHECK_ONLY: no mode or motor command sent", flush=True)
            return
        start_pos = pv[0]

        mode_changed = True
        bus.set_controller_mode(NODE, CONTROL_MODE["VOLTAGE_CONTROL"], INPUT_MODE["PASSTHROUGH"])
        if bus.read("hoverboard.config.drivemode") != 0:
            raise RuntimeError("block voltage mode did not take effect")
        bus.set_input_voltage(NODE, 0.0)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        time.sleep(0.03)
        h = bus.heartbeat(NODE)
        if h is None or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"] or h["errors"]:
            raise RuntimeError("axis did not arm at zero voltage")

        started = time.monotonic()
        deadline = started + DURATION
        print("PULSE", FRACTION, "for at most", DURATION, "s", flush=True)
        while time.monotonic() < deadline:
            bus.set_input_voltage(NODE, FRACTION)
            time.sleep(0.04)
            h = bus.heartbeat(NODE)
            vi = bus.get_bus_voltage_current(NODE)
            pv = bus.get_encoder_estimates(NODE)
            delta = None if pv is None else pv[0] - start_pos
            print("SAMPLE", "elapsed_s", round(time.monotonic() - started, 3),
                  "heartbeat", h, "bus", vi, "encoder", pv,
                  "position_delta", None if delta is None else round(delta, 5), flush=True)
            if (h is None or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"] or h["errors"]
                    or vi is None or not 35.0 <= vi[0] <= 43.0 or abs(vi[1]) > 0.8
                    or pv is None or not all(map(math.isfinite, pv)) or delta < -0.01):
                raise RuntimeError("fault, supply droop, or reverse position; stopping")
            if delta > 0.125 or abs(pv[1]) > 0.6:
                print("LIMIT_REACHED: bounded motion complete; sending IDLE", flush=True)
                break
    finally:
        try:
            if mode_changed:
                stop(bus)
        finally:
            bus.s.close()


if __name__ == "__main__":
    main()
