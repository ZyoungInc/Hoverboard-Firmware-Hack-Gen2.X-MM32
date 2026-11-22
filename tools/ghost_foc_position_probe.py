#!/usr/bin/env python3
"""Short Hall-step FOC position trial on the verified Ghost Gen2.4.2 wheel.

Uses the existing CANSimple position and trajectory modes. All tuning is RAM-only;
--run requires an explicit operator command. IDLE and saved settings are restored.
"""

import argparse
import math
import time

from cansimple_uart import AXIS_STATE, Bus
from ghost_foc_current_probe import NODE, preflight


TEMPORARY = {
    "focimax": 20,       # 2 A phase current
    "posrpm": 30,        # 0.5 turn/s limit
    "posaccel": 100,     # 1.0 turn/s^2 position approach limit
    "inputmode": 5,      # existing TRAP_TRAJ mode
    "trajvel": 15,       # 0.15 turn/s trajectory limit
    "trajaccel": 50,     # 0.5 turn/s^2
    "trajdecel": 50,
    "holdenable": 1,    # off: report actual Hall position, not the moving hold target
    "drivemode": 11,     # FOC + POSITION
}


def read_cfg(bus, name):
    return bus.read("hoverboard.config." + name)


def restore(bus, saved, estop=False):
    if estop:
        bus.estop(NODE)
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_input_vel(NODE, 0.0)
    bus.set_input_torque(NODE, 0.0)
    pos = bus.get_encoder_estimates(NODE)
    if pos is not None and math.isfinite(pos[0]):
        bus.set_input_pos(NODE, pos[0])
    if saved is not None:
        for name, value in reversed(list(saved.items())):
            if value is None or not bus.write("hoverboard.config." + name, value):
                print("RESTORE_FAILED", name, flush=True)
    print("STOP", bus.heartbeat(NODE), "error", bus.get_error(NODE),
          "bus", bus.get_bus_voltage_current(NODE),
          "encoder", bus.get_encoder_estimates(NODE),
          "mode", read_cfg(bus, "drivemode"),
          "focimax", read_cfg(bus, "focimax"), flush=True)


def sample(bus, start, start_odom, target, stage, elapsed):
    h = bus.heartbeat(NODE)
    vi = bus.get_bus_voltage_current(NODE)
    pv = bus.get_encoder_estimates(NODE)
    iq = bus.get_iq(NODE)
    odom = bus.read("hoverboard.odometer")
    delta = None if pv is None else pv[0] - start
    print("SAMPLE", stage, round(elapsed, 3), "heartbeat", h,
          "bus", vi, "encoder", pv, "iq", iq,
          "travel", delta, "hall_steps", None if odom is None else odom - start_odom,
          "target_travel", target - start, flush=True)
    if h is None or vi is None or pv is None or iq is None or odom is None:
        return "telemetry_lost"
    if h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]:
        return "firmware_fault_or_disarm"
    if not all(map(math.isfinite, (vi[0], vi[1], pv[0], pv[1]))):
        return "invalid_telemetry"
    if (not 34.0 <= vi[0] <= 43.0 or abs(vi[1]) > 3.0
            or abs(delta) > 0.25 or abs(odom - start_odom) > 12
            or abs(pv[1]) > 1.0
            or math.isfinite(iq[1]) and abs(iq[1]) > 4.0):
        return "measurement_limit"
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    modes = ap.add_mutually_exclusive_group(required=True)
    modes.add_argument("--check-only", action="store_true")
    modes.add_argument("--run", action="store_true")
    modes.add_argument("--stop", action="store_true", help="ESTOP, IDLE, restore saved defaults")
    ap.add_argument("--direction", choices=("forward", "reverse"), default="forward")
    ap.add_argument("--steps", type=int, choices=(1, 2, 3, 4), default=1,
                    help="position target in Hall steps (48 steps per mechanical turn)")
    ap.add_argument("--hold", action="store_true",
                    help="use the firmware's FOC position hold near the target")
    ap.add_argument("--duration-s", type=float, default=1.5)
    args = ap.parse_args()
    if not 0.5 <= args.duration_s <= 3.0:
        ap.error("duration must be 0.5..3.0 s")

    bus = Bus(args.port, 115200, timeout=0.25)
    saved = None
    try:
        if args.stop:
            restore(bus, {"drivemode": 1, "focimax": 400, "posrpm": 65535,
                          "posaccel": 65535, "inputmode": 65535,
                          "trajvel": 65535, "trajaccel": 65535,
                          "trajdecel": 65535, "holdenable": 65535}, estop=True)
            return
        start, _ = preflight(bus)
        start_odom = bus.read("hoverboard.odometer")
        if start_odom is None:
            raise RuntimeError("cannot read Hall odometer; no motor command sent")
        saved = {name: read_cfg(bus, name) for name in TEMPORARY}
        print("POSITION_PREFLIGHT", "saved", saved, flush=True)
        if any(value is None for value in saved.values()):
            raise RuntimeError("cannot read all settings; no motor command sent")
        if args.check_only:
            print("CHECK_ONLY: no configuration or motor command sent", flush=True)
            return

        temporary = dict(TEMPORARY)
        temporary["holdenable"] = 2 if args.hold else 1
        for name, value in temporary.items():
            if not bus.write("hoverboard.config." + name, value):
                raise RuntimeError("could not set RAM " + name)
            if read_cfg(bus, name) != value:
                raise RuntimeError("RAM readback failed for " + name)
        bus.set_input_pos(NODE, start)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        time.sleep(0.05)
        status = sample(bus, start, start_odom, start, "zero_target_arm", 0.0)
        if status:
            print("RESULT", status, flush=True)
            return

        direction = 1 if args.direction == "forward" else -1
        target = start + direction * args.steps / 48.0
        bus.set_input_pos(NODE, target)
        print("ARMED", "target", target, "delta_turns", direction * args.steps / 48.0,
              "hall_steps", direction * args.steps, "hold", args.hold,
              "RAM current cap 2 A, duration", args.duration_s, "s", flush=True)
        began = time.monotonic()
        result = "time_complete"
        samples = 0
        while time.monotonic() - began < args.duration_s:
            time.sleep(0.015)
            bus.set_input_pos(NODE, target)  # feed existing command watchdog
            status = sample(bus, start, start_odom, target, "move", time.monotonic() - began)
            samples += 1
            if status:
                result = status
                break
        print("RESULT", result, "samples", samples, flush=True)
    finally:
        try:
            if saved is not None and args.run:
                restore(bus, saved)
        finally:
            bus.s.close()


if __name__ == "__main__":
    main()
