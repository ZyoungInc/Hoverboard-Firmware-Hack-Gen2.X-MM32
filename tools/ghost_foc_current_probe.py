#!/usr/bin/env python3
"""Bounded first nonzero FOC-current probe for one suspended Ghost Gen2.4.2 wheel.

Uses the project's CANSimple UART Bus. Run at about 36-39 V with a current-limited
bench supply. --check-only sends no motor or configuration command. --run is an
explicit signed-Iq trial. --stop sends ESTOP and IDLE.
The --run path restores IDLE and block mode in finally, even on telemetry failure.
"""

import argparse
import math
import time

from cansimple_uart import AXIS_STATE, Bus


NODE = 1
EXPECTED = {
    "drivemode": 1, "halla": 27, "hallb": 26, "hallc": 28,
    "hallinvert": 1, "windings": 16, "iphasea": 2, "iphaseb": 6,
    "iphasecfg": 1, "iphasegain": 354, "motorr": 838,
    "hallang1": 42090, "hallang2": 64708, "hallang3": 52258,
    "hallang4": 19330, "hallang5": 32366, "hallang6": 9904,
    "focimax": 400, "curpull": 450, "curtrip": 500,
    "itotaldivider": 250, "softocp": 40, "hardocp": 160,
    "batfull": 42000,
}


def idle(bus, restore_mode=False, estop=False):
    if estop:
        bus.estop(NODE)
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_input_torque(NODE, 0.0)
    bus.set_input_vel(NODE, 0.0)
    time.sleep(0.04)
    if restore_mode:
        if not bus.write("hoverboard.config.drivemode", 1):
            print("RESTORE_FAILED drivemode", flush=True)
    print("STOP", bus.heartbeat(NODE), bus.get_error(NODE),
          bus.get_bus_voltage_current(NODE),
          "mode", bus.read("hoverboard.config.drivemode"), flush=True)


def preflight(bus):
    h = bus.heartbeat(NODE)
    e = bus.get_error(NODE)
    vi = bus.get_bus_voltage_current(NODE)
    pv = bus.get_encoder_estimates(NODE)
    cfg = {name: bus.read("hoverboard.config." + name) for name in EXPECTED}
    vel = bus.read("axis0.controller.input_vel")
    torque = bus.read("axis0.controller.input_torque")
    voltage = bus.read("hoverboard.input_voltage")
    kt = bus.read("hoverboard.torque_constant_in_use")
    print("PREFLIGHT", h, "error", e, "bus", vi, "encoder", pv,
          "config", cfg, "inputs", (vel, torque, voltage), "kt", kt, flush=True)
    if (h is None or h["state"] != AXIS_STATE["IDLE"] or h["errors"]
            or e is None or e["active_errors"] or e["disarm_reason"]
            or vi is None or not 35.0 <= vi[0] <= 40.5 or abs(vi[1]) > 0.1
            or pv is None or not all(map(math.isfinite, pv)) or abs(pv[1]) > 0.1
            or cfg != EXPECTED or vel != 0.0 or torque != 0.0 or voltage != 0.0
            or kt is None or not 0.1 <= kt <= 1.0):
        raise RuntimeError("preflight failed; no motor command sent")
    return pv[0], kt


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check-only", action="store_true", help="read-only preflight")
    g.add_argument("--run", action="store_true", help="one bounded Iq trial")
    g.add_argument("--stop", action="store_true", help="send ESTOP, IDLE and zero setpoints")
    ap.add_argument("--iq-a", type=float, default=0.5, help="phase-Iq magnitude, 0.1..5 A")
    ap.add_argument("--direction", choices=("forward", "reverse"), default="forward")
    ap.add_argument("--duration-s", type=float, default=0.5, help="motor pulse duration, 0.05..1.0 s")
    args = ap.parse_args()
    bus = Bus(args.port, 115200, timeout=0.25)
    mode_changed = False
    armed = False
    try:
        if args.stop:
            idle(bus, restore_mode=True, estop=True)
            return
        if not 0.1 <= args.iq_a <= 5.0:
            raise ValueError("--iq-a must be 0.1..5 A")
        if not 0.05 <= args.duration_s <= 1.0:
            raise ValueError("--duration-s must be 0.05..1.0")
        direction = 1 if args.direction == "forward" else -1
        start_pos, kt = preflight(bus)
        if args.check_only:
            print("CHECK_ONLY: no configuration or motor command sent", flush=True)
            return
        if not bus.write("hoverboard.config.drivemode", 10):
            raise RuntimeError("cannot select FOC_TORQUE in RAM")
        mode_changed = True
        if bus.read("hoverboard.config.drivemode") != 10:
            raise RuntimeError("FOC_TORQUE mode readback failed")
        bus.set_input_torque(NODE, 0.0)
        command_nm = direction * args.iq_a * kt
        bus.set_input_torque(NODE, command_nm)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        armed = True
        print("ARMED", direction * args.iq_a, "A Iq, CANSimple torque field", command_nm,
              "Nm (firmware Kt estimate; physical Kt uncalibrated), at most",
              args.duration_s, "s", flush=True)
        started = time.monotonic()
        measured = []
        while time.monotonic() - started < args.duration_s:
            time.sleep(0.005)
            h = bus.heartbeat(NODE)
            vi = bus.get_bus_voltage_current(NODE)
            pv = bus.get_encoder_estimates(NODE)
            iq = bus.get_iq(NODE)
            elapsed = time.monotonic() - started
            print("SAMPLE", round(elapsed, 3), h, "bus", vi,
                  "encoder", pv, "iq", iq, flush=True)
            if (h is None or h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]
                    or vi is None or pv is None or iq is None or not all(map(math.isfinite, pv))):
                raise RuntimeError("fault, disarm or missing telemetry")
            if math.isfinite(iq[0]) and direction * iq[0] < -0.1:
                raise RuntimeError("torque velocity limit applied reverse braking; stopping")
            if (not 34.0 <= vi[0] <= 43.0 or abs(vi[1]) > 3.0
                    or abs(pv[0] - start_pos) > 0.5 or abs(pv[1]) > 2.0):
                raise RuntimeError("voltage, bus current or motion guard")
            if math.isfinite(iq[1]):
                measured.append(iq[1])
                if abs(iq[1]) > max(2.5, args.iq_a * 1.5 + 1.0):
                    raise RuntimeError("measured Iq exceeded the probe range")
        if not measured:
            raise RuntimeError("phase current telemetry never became valid")
        print("RESULT", "test_finished", "finite_iq_samples", len(measured),
              "measured_iq_range_a", (min(measured), max(measured)),
              "requested_iq_a", direction * args.iq_a, flush=True)
    finally:
        try:
            if armed or mode_changed:
                idle(bus, restore_mode=mode_changed)
        finally:
            bus.s.close()


if __name__ == "__main__":
    main()
