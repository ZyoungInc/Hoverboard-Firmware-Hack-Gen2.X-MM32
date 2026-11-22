#!/usr/bin/env python3
"""Bounded continuous block-PWM voltage ramp for the Ghost Gen2.4.2 wheel.

Operator must pass --run. Uses CANSimple voltage control, changes RAM mode only,
and sends IDLE before restoring block speed. No Flash writes.
"""

import argparse
from collections import deque
import math
import time

from cansimple_uart import AXIS_STATE, Bus, CONTROL_MODE, INPUT_MODE
from ghost_foc_current_probe import NODE, preflight


def stop(bus):
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_input_voltage(NODE, 0.0)
    bus.set_controller_mode(NODE, CONTROL_MODE["VELOCITY_CONTROL"], INPUT_MODE["PASSTHROUGH"])
    bus.set_input_vel(NODE, 0.0)
    time.sleep(0.05)
    print("STOP", "heartbeat", bus.heartbeat(NODE), "error", bus.get_error(NODE),
          "bus", bus.get_bus_voltage_current(NODE),
          "encoder", bus.get_encoder_estimates(NODE),
          "odometer", bus.read("hoverboard.odometer"),
          "drivemode", bus.read("hoverboard.config.drivemode"), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    modes = ap.add_mutually_exclusive_group(required=True)
    modes.add_argument("--check-only", action="store_true")
    modes.add_argument("--run", action="store_true")
    modes.add_argument("--stop", action="store_true", help="IDLE, zero voltage and restore block speed")
    ap.add_argument("--start-percent", type=int, default=0)
    ap.add_argument("--max-percent", type=int, default=100)
    ap.add_argument("--step-percent", type=int, default=5)
    ap.add_argument("--stage-seconds", type=float, default=0.35)
    ap.add_argument("--speed-limit-tps", type=float, default=18.0,
                    help="host speed cutoff in mechanical turns/s; 0 disables it")
    args = ap.parse_args()
    if not (0 <= args.start_percent <= args.max_percent <= 100
            and 1 <= args.step_percent <= 20
            and 0.2 <= args.stage_seconds <= 5.0
            and (args.speed_limit_tps == 0.0 or 1.0 <= args.speed_limit_tps <= 20.0)):
        ap.error("invalid ramp range, stage duration, or speed limit")
    percents = list(range(args.start_percent, args.max_percent + 1, args.step_percent))
    if percents[-1] != args.max_percent:
        percents.append(args.max_percent)
    stages = tuple(percent / 100.0 for percent in percents)

    bus = Bus(args.port, 115200, timeout=0.25)
    changed = False
    try:
        if args.stop:
            stop(bus)
            return
        start_pos, _ = preflight(bus)
        start_odom = bus.read("hoverboard.odometer")
        if start_odom is None:
            raise RuntimeError("Hall odometer unavailable; no motor command sent")
        print("RAMP_PREFLIGHT", "odometer", start_odom,
              "stages", stages, "stage_seconds", args.stage_seconds,
              "speed_limit_tps", "OFF" if args.speed_limit_tps == 0.0 else args.speed_limit_tps,
              flush=True)
        if args.check_only:
            print("CHECK_ONLY: no configuration or motor command sent", flush=True)
            return

        changed = True
        bus.set_controller_mode(NODE, CONTROL_MODE["VOLTAGE_CONTROL"], INPUT_MODE["PASSTHROUGH"])
        if bus.read("hoverboard.config.drivemode") != 0:
            raise RuntimeError("block voltage mode readback failed")
        bus.set_input_voltage(NODE, 0.0)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        time.sleep(0.03)
        armed = bus.heartbeat(NODE)
        if armed is None or armed["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"] or armed["errors"]:
            raise RuntimeError("zero-voltage arm failed")

        started = time.monotonic()
        history = deque(maxlen=32)
        last_edge = started
        previous_odom = start_odom
        result = "stages_complete"
        print("ARMED", "block voltage ramp", stages,
              "total_seconds", len(stages) * args.stage_seconds, flush=True)
        for stage, fraction in enumerate(stages, 1):
            stage_start = time.monotonic()
            next_print = stage_start
            stage_odom = bus.read("hoverboard.odometer")
            odom = stage_odom
            print("STAGE", stage, "fraction", fraction, flush=True)
            while time.monotonic() - stage_start < args.stage_seconds:
                bus.set_input_voltage(NODE, fraction)
                time.sleep(0.012)
                h = bus.heartbeat(NODE)
                vi = bus.get_bus_voltage_current(NODE)
                pv = bus.get_encoder_estimates(NODE)
                odom = bus.read("hoverboard.odometer")
                now = time.monotonic()
                if h is None or vi is None or pv is None or odom is None:
                    result = "telemetry_lost"
                    break
                if not all(map(math.isfinite, (vi[0], vi[1], pv[0], pv[1]))):
                    result = "invalid_telemetry"
                    break
                history.append((now, odom))
                if odom != previous_odom:
                    last_edge = now
                    previous_odom = odom
                while len(history) > 2 and now - history[0][0] > 0.30:
                    history.popleft()
                hall_speed = ((odom - history[0][1]) / 48.0 / (now - history[0][0])
                              if len(history) > 1 and now > history[0][0] else 0.0)
                if now >= next_print:
                    print("SAMPLE", "stage", stage, "elapsed_s", round(now - started, 3),
                          "bus", tuple(round(x, 3) for x in vi),
                          "encoder", tuple(round(x, 4) for x in pv),
                          "encoder_rpm", round(pv[1] * 60),
                          "hall_steps", odom - start_odom,
                          "hall_speed_turns_s", round(hall_speed, 3),
                          "errors", h["errors"], "state", h["state"], flush=True)
                    next_print = now + (0.25 if args.stage_seconds >= 1.0 else 0.12)
                if h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]:
                    result = "firmware_fault_or_disarm"
                    break
                if (not 34.0 <= vi[0] <= 43.0 or abs(vi[1]) > 3.0
                        or (args.speed_limit_tps > 0.0 and
                            (abs(pv[1]) > args.speed_limit_tps or
                             abs(hall_speed) > args.speed_limit_tps))
                        or odom - start_odom < -2):
                    result = "measurement_limit"
                    break
                if fraction >= 0.10 and now - last_edge > 0.60 and now - stage_start > 0.25:
                    result = "stalled_before_high_duty"
                    break
            print("STAGE_RESULT", stage, "fraction", fraction,
                  "hall_steps_delta", None if stage_odom is None or odom is None else odom - stage_odom,
                  "average_rpm", None if stage_odom is None or odom is None else
                  round((odom - stage_odom) * 60.0 / (48.0 * (time.monotonic() - stage_start))),
                  "result", result, flush=True)
            if result != "stages_complete":
                break
        print("RESULT", result, "elapsed_s", round(time.monotonic() - started, 3), flush=True)
    finally:
        try:
            if changed:
                stop(bus)
        finally:
            bus.s.close()


if __name__ == "__main__":
    main()
