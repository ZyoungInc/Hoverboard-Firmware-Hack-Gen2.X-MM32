#!/usr/bin/env python3
"""One short Gen2.4.2 block-speed trial using the existing CANSimple UART host library.

Run only with the wheel suspended, original phase/Hall harness attached, and a
42 V bench supply limited to 0.5 A. This script does not change saved config.
It commands 0.1 mechanical turn/s for at most one second, then sends IDLE.
"""

import argparse
import time

from cansimple_uart import AXIS_STATE, Bus


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--direction-calibrated", action="store_true",
                    help="run only after the motor phase/Hall mapping has been calibrated")
    ap.add_argument("--stop-only", action="store_true", help="send IDLE twice and zero the idle setpoint")
    args = ap.parse_args()
    if args.stop_only:
        bus = Bus(args.port, 115200, timeout=0.2)
        try:
            bus.set_axis_state(1, AXIS_STATE["IDLE"])
            bus.set_axis_state(1, AXIS_STATE["IDLE"])
            bus.set_input_vel(1, 0.0)
            print("STOP IDLE", bus.heartbeat(1), "bus", bus.get_bus_voltage_current(1), flush=True)
        finally:
            bus.s.close()
        return
    if not args.direction_calibrated:
        raise SystemExit("Disabled: +0.1 turn/s produced reverse acceleration on this motor. "
                         "Calibrate its phase/Hall mapping before another speed-loop test.")
    bus = Bus(args.port, 115200, timeout=0.2)
    node = 1
    try:
        h = bus.heartbeat(node)
        vi = bus.get_bus_voltage_current(node)
        pv = bus.get_encoder_estimates(node)
        cfg = {k: bus.read("hoverboard.config." + k)
               for k in ("drivemode", "itotaldivider", "softocp", "hardocp", "windings",
                         "halla", "hallb", "hallc", "hallinvert")}
        print("PREFLIGHT", h, "bus", vi, "encoder", pv, "config", cfg, flush=True)
        if (h is None or h["state"] != AXIS_STATE["IDLE"] or h["errors"]
                or vi is None or not 40.0 <= vi[0] <= 43.0 or abs(vi[1]) > 0.1
                or pv is None or abs(pv[1]) > 0.05
                or cfg != {"drivemode": 1, "itotaldivider": 250, "softocp": 2,
                           "hardocp": 8, "windings": 16, "halla": 27, "hallb": 26,
                           "hallc": 28, "hallinvert": 1}):
            raise RuntimeError("preflight failed; bridge was not armed")
        pv0 = pv[0]

        # Setpoint first while IDLE so arming never starts with an old command.
        # Repeat it below before the firmware's 500 ms command timeout.
        bus.set_input_vel(node, 0.1)
        bus.set_axis_state(node, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        print("ARMED: command 0.1 turn/s for at most 1.0 s", flush=True)
        started = time.monotonic()
        deadline = started + 1.0
        while time.monotonic() < deadline:
            bus.set_input_vel(node, 0.1)
            time.sleep(0.08)
            h = bus.heartbeat(node)
            vi = bus.get_bus_voltage_current(node)
            pv = bus.get_encoder_estimates(node)
            print("SAMPLE", "elapsed_s", round(time.monotonic() - started, 3),
                  h, "bus", vi, "encoder", pv,
                  "position_delta", None if pv is None else round(pv[0] - pv0, 5), flush=True)
            if (h is None or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]
                    or h["errors"] or vi is None or not 35.0 <= vi[0] <= 43.0
                    or abs(vi[1]) > 0.8 or pv is None
                    or pv[0] < pv0 - 0.01 or pv[0] > pv0 + 0.15
                    or abs(pv[1]) > 0.5):
                raise RuntimeError("fault, supply droop, reverse position, or overspeed; stopping")
    finally:
        # IDLE turns MOE off; do this before setting speed zero because COM_SPEED
        # at zero while still armed actively shorts the low sides to brake.
        try:
            bus.set_axis_state(node, AXIS_STATE["IDLE"])
            bus.set_axis_state(node, AXIS_STATE["IDLE"])
            bus.set_input_vel(node, 0.0)
            print("STOP IDLE", bus.heartbeat(node), "bus",
                  bus.get_bus_voltage_current(node), flush=True)
        finally:
            bus.s.close()


if __name__ == "__main__":
    main()
