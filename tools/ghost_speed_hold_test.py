#!/usr/bin/env python3
"""Three-second closed-loop block-speed trial for the calibrated Gen2.4.2 board.

The user must explicitly pass --run. Setpoint: 0.25 mechanical turn/s (15 rpm).
The script temporarily caps the requested velocity at 36 rpm and explicitly
disables the firmware overspeed fault. It sends IDLE and restores prior RAM
settings after the trial. --gentle temporarily lowers only the block-speed
startup feed-forward and shortens the trial. Keep the wheel suspended and
the 42 V bench supply limited to 0.5 A.
"""

import argparse
import math
import time

from cansimple_uart import AXIS_STATE, Bus


NODE = 1
SPEED_TURNS_S = 0.25
DURATION_S = 3.0
GENTLE_DURATION_S = 1.2
GENTLE_TRAVEL_TURNS = 0.5
EXPECTED = {
    "drivemode": 1, "halla": 27, "hallb": 26, "hallc": 28,
    "hallinvert": 1, "windings": 16, "vbatgain": 30370,
    "itotaldivider": 250, "softocp": 2, "hardocp": 8,
    "posrpm": 65535, "overspeederr": 65535, "vellimittol": 65535,
    "speedd0": 65535,
}


def stop(bus, restore=False):
    # IDLE before zeroing a speed command: zero speed while armed actively brakes.
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_input_vel(NODE, 0.0)
    if restore:
        if not bus.write("hoverboard.config.posrpm", EXPECTED["posrpm"]):
            print("RESTORE_FAILED posrpm", flush=True)
        if not bus.write("hoverboard.config.overspeederr", EXPECTED["overspeederr"]):
            print("RESTORE_FAILED overspeederr", flush=True)
        if not bus.write("hoverboard.config.speedd0", EXPECTED["speedd0"]):
            print("RESTORE_FAILED speedd0", flush=True)
    time.sleep(0.05)
    print("STOP", "heartbeat", bus.heartbeat(NODE), "bus", bus.get_bus_voltage_current(NODE),
          "encoder", bus.get_encoder_estimates(NODE),
          "posrpm", bus.read("hoverboard.config.posrpm"),
          "overspeederr", bus.read("hoverboard.config.overspeederr"),
          "speedd0", bus.read("hoverboard.config.speedd0"), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--run", action="store_true", help="run the bounded 0.25 turn/s trial")
    group.add_argument("--check-only", action="store_true", help="read-only preflight")
    group.add_argument("--stop-only", action="store_true", help="send IDLE and zero the setpoint")
    ap.add_argument("--gentle", action="store_true", help="RAM-only lower startup feed-forward, 1.2 s/0.5 turn bound")
    args = ap.parse_args()
    bus = Bus(args.port, 115200, timeout=0.2)
    if args.stop_only:
        try:
            stop(bus, restore=True)
        finally:
            bus.s.close()
        return

    limit_changed = False
    result = "time_complete"
    try:
        h = bus.heartbeat(NODE)
        vi = bus.get_bus_voltage_current(NODE)
        pv = bus.get_encoder_estimates(NODE)
        e = bus.get_error(NODE)
        cfg = {name: bus.read("hoverboard.config." + name) for name in EXPECTED}
        print("PREFLIGHT", h, "bus", vi, "encoder", pv, "error", e,
              "config", cfg, flush=True)
        if (h is None or h["state"] != AXIS_STATE["IDLE"] or h["errors"]
                or e is None or e["active_errors"] or e["disarm_reason"]
                or vi is None or not 41.0 <= vi[0] <= 43.0 or abs(vi[1]) > 0.1
                or pv is None or not all(map(math.isfinite, pv)) or abs(pv[1]) > 0.05
                or cfg != EXPECTED):
            raise RuntimeError("preflight failed; no motor command sent")
        if args.check_only:
            print("CHECK_ONLY: no configuration or motor command sent", flush=True)
            return

        limit_changed = True
        if not bus.write("hoverboard.config.posrpm", 36):
            raise RuntimeError("cannot set requested-velocity limit")
        if bus.read("hoverboard.config.posrpm") != 36:
            raise RuntimeError("requested-velocity limit readback failed")
        if not bus.write("hoverboard.config.overspeederr", 1):
            raise RuntimeError("cannot disable firmware overspeed fault")
        if bus.read("hoverboard.config.overspeederr") != 1:
            raise RuntimeError("overspeed-off readback failed")
        print("LIMIT: requested velocity capped at 36 rpm; firmware overspeed fault explicitly off", flush=True)
        if args.gentle:
            if not bus.write("hoverboard.config.speedd0", 30):
                raise RuntimeError("cannot lower startup feed-forward")
            if bus.read("hoverboard.config.speedd0") != 30:
                raise RuntimeError("startup feed-forward readback failed")
            print("TUNE: block-speed dead-band feed-forward 53 -> 30 permille, RAM only", flush=True)

        start_pos = pv[0]
        bus.set_input_vel(NODE, SPEED_TURNS_S)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        started = time.monotonic()
        duration = GENTLE_DURATION_S if args.gentle else DURATION_S
        travel_limit = GENTLE_TRAVEL_TURNS if args.gentle else 2.0
        deadline = started + duration
        print("ARMED", SPEED_TURNS_S, "turn/s for at most", duration, "s", flush=True)
        while time.monotonic() < deadline:
            bus.set_input_vel(NODE, SPEED_TURNS_S)
            time.sleep(0.05)
            h = bus.heartbeat(NODE)
            vi = bus.get_bus_voltage_current(NODE)
            pv = bus.get_encoder_estimates(NODE)
            fast = bus.read("hoverboard.fast_speed")
            delta = None if pv is None else pv[0] - start_pos
            print("SAMPLE", "elapsed_s", round(time.monotonic() - started, 3),
                  "heartbeat", h, "bus", vi, "encoder", pv, "fast_rpm", fast,
                  "position_delta", None if delta is None else round(delta, 5), flush=True)
            if h is None or vi is None or pv is None or fast is None or not all(map(math.isfinite, pv)):
                result = "telemetry_lost"
                break
            if h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]:
                result = "firmware_fault_or_disarm"
                break
            if not 35.0 <= vi[0] <= 43.0 or abs(vi[1]) > 0.8:
                result = "supply_limit"
                break
            if delta < -0.01:
                result = "reverse_position"
                break
            if delta > travel_limit:
                result = "travel_limit"
                break
    finally:
        try:
            if limit_changed:
                stop(bus, restore=True)
        finally:
            bus.s.close()
    print("RESULT", result, flush=True)
    if result not in ("time_complete", "travel_limit"):
        raise SystemExit(2)


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None
