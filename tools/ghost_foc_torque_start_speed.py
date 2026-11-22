#!/usr/bin/env python3
"""Start a Ghost Gen2.4.2 wheel with FOC torque, then hold FOC speed.

This tests FOC startup without block commutation. The torque kick crosses the
measured cogging threshold; FOC speed takes over at the observed rolling speed.
All configuration changes are RAM-only and --run is explicit.
"""

import argparse
import math
import statistics
import time

from cansimple_uart import AXIS_STATE, Bus
from ghost_foc_current_probe import NODE, preflight
from ghost_foc_speed_sweep import stop, write_checked


def sample(bus, stage, since, direction, start_odom, max_bus_v=43.0):
    h = bus.heartbeat(NODE)
    vi = bus.get_bus_voltage_current(NODE)
    pv = bus.get_encoder_estimates(NODE)
    iq = bus.get_iq(NODE)
    odom = bus.read("hoverboard.odometer")
    steps = None if odom is None else direction * (odom - start_odom)
    print("SAMPLE", stage, "elapsed_s", round(time.monotonic() - since, 3),
          "heartbeat", h, "bus", vi, "encoder", pv, "iq", iq,
          "directed_hall_steps", steps, flush=True)
    if (h is None or vi is None or pv is None or iq is None or steps is None
            or not all(map(math.isfinite, (*vi, *pv, *iq)))):
        raise RuntimeError("telemetry lost")
    if h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]:
        raise RuntimeError("firmware fault or axis disarmed")
    if not 34.0 <= vi[0] <= max_bus_v or abs(vi[1]) > 3.0:
        raise RuntimeError("bus voltage or current outside test range")
    if direction * pv[1] < -0.2 or abs(pv[1]) > 10.0 or steps < -1:
        raise RuntimeError("unexpected direction or speed")
    return pv, iq, steps


def speed_stage(bus, target, direction, start_odom, seconds, name, max_bus_v=43.0):
    since = time.monotonic()
    speeds = []
    print("FOC_SPEED_STAGE", name, "target_tps", target,
          "duration_s", seconds, "current_cap_a", 0.2, flush=True)
    while time.monotonic() - since < seconds:
        bus.set_input_vel(NODE, target)
        time.sleep(0.012)
        pv, iq, _ = sample(bus, name, since, direction, start_odom,
                           max_bus_v=max_bus_v)
        if abs(iq[1]) > 0.86:
            raise RuntimeError("measured phase current outside speed-test range")
        speeds.append(pv[1])
    if not speeds:
        raise RuntimeError("no speed samples")
    median = statistics.median(speeds[-min(5, len(speeds)):])
    print("FOC_SPEED_RESULT", name, "target_tps", target,
          "median_last_speed_tps", round(median, 3), flush=True)
    if abs(median - target) > 1.0:
        raise RuntimeError("FOC speed target not tracked")


def reverse_probe(bus, start_odom, cap_a):
    """Brake while rolling; briefly allow breakaway current near zero speed."""
    began = time.monotonic()
    peak_odom = start_odom
    print("REVERSE_BRAKE", "target_tps", 2.0, "current_cap_a", 0.2,
          "max_seconds", 1.0, flush=True)
    while time.monotonic() - began < 1.0:
        bus.set_input_vel(NODE, 2.0)
        time.sleep(0.008)
        pv, iq, _ = sample(bus, "reverse_brake", began, 1, start_odom,
                           max_bus_v=41.0)
        if abs(iq[1]) > 0.86:
            raise RuntimeError("measured phase current outside braking range")
        odom = bus.read("hoverboard.odometer")
        if odom is None:
            raise RuntimeError("Hall odometer lost")
        peak_odom = max(peak_odom, odom)
        if 0.0 <= pv[1] <= 1.5:
            print("REVERSE_BRAKE_READY", "speed_tps", pv[1],
                  "peak_odometer", peak_odom, flush=True)
            break
    else:
        raise RuntimeError("did not slow to the reversal entry range")

    write_checked(bus, "focimax", round(cap_a * 10))
    bus.set_input_vel(NODE, -2.0)
    began = time.monotonic()
    prev_time, prev_steps = began, 0
    print("REVERSE_CROSS", "target_tps", -2.0, "current_cap_a", cap_a,
          "max_seconds", 0.8, flush=True)
    while time.monotonic() - began < 0.8:
        bus.set_input_vel(NODE, -2.0)
        time.sleep(0.008)
        h = bus.heartbeat(NODE)
        vi = bus.get_bus_voltage_current(NODE)
        pv = bus.get_encoder_estimates(NODE)
        iq = bus.get_iq(NODE)
        odom = bus.read("hoverboard.odometer")
        now = time.monotonic()
        if (h is None or vi is None or pv is None or iq is None or odom is None
                or not all(map(math.isfinite, (*vi, *pv, *iq)))):
            raise RuntimeError("reversal telemetry lost")
        peak_odom = max(peak_odom, odom)
        steps = peak_odom - odom
        hall_rate = (steps - prev_steps) / 48.0 / max(now - prev_time, 0.001)
        prev_time, prev_steps = now, steps
        print("SAMPLE", "reverse_cross", "elapsed_s", round(now - began, 3),
              "heartbeat", h, "bus", vi, "encoder", pv, "iq", iq,
              "reverse_hall_steps", steps, flush=True)
        if h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]:
            raise RuntimeError("firmware fault or axis disarmed during reversal")
        if (not 34.0 <= vi[0] <= 41.0 or abs(vi[1]) > 3.0
                or not -8.0 <= pv[1] <= 3.0 or abs(iq[1]) > 2.0
                or steps > 48):
            raise RuntimeError("voltage, current or motion outside reversal range")
        if steps >= 4 and hall_rate >= 2.0:
            write_checked(bus, "focimax", 2)
            bus.set_input_vel(NODE, -6.0)
            print("REVERSE_CUTOFF", "reverse_hall_steps", steps,
                  "recent_hall_rate_tps", round(hall_rate, 3),
                  "encoder_speed_tps", pv[1], flush=True)
            break
    else:
        raise RuntimeError("FOC speed reversal did not cross zero")
    speed_stage(bus, -6.0, -1, peak_odom, 0.8, "reverse_six",
                max_bus_v=41.0)
    print("RESULT", "foc_running_reversal_complete", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check-only", action="store_true")
    mode.add_argument("--run", action="store_true")
    ap.add_argument("--direction", choices=("forward", "reverse"), default="forward")
    ap.add_argument("--kick-a", type=float, default=1.4)
    probe = ap.add_mutually_exclusive_group()
    probe.add_argument("--decel-only", action="store_true",
                       help="after the proven forward start, test FOC speed deceleration to zero")
    probe.add_argument("--reverse-probe", action="store_true",
                       help="brake forward motion, cross zero, and test reverse FOC speed")
    ap.add_argument("--reverse-cap-a", type=float, default=1.3,
                    help="temporary Iq request cap near the zero-speed crossing, 1.2..1.4 A")
    args = ap.parse_args()
    if not 1.2 <= args.kick_a <= 1.5:
        ap.error("--kick-a must be 1.2..1.5 A")
    if not 1.2 <= args.reverse_cap_a <= 1.4:
        ap.error("--reverse-cap-a must be 1.2..1.4 A")
    if (args.decel_only or args.reverse_probe) and args.direction != "forward":
        ap.error("deceleration and running reversal require --direction forward")
    direction = 1 if args.direction == "forward" else -1

    bus = Bus(args.port, 115200, timeout=0.25)
    saved = None
    try:
        _, kt = preflight(bus)
        saved = {name: bus.read("hoverboard.config." + name)
                 for name in ("drivemode", "posrpm", "focimax")}
        odom0 = bus.read("hoverboard.odometer")
        sinekv = bus.read("hoverboard.config.sinekv")
        if (saved != {"drivemode": 1, "posrpm": 65535, "focimax": 400}
                or odom0 is None or sinekv != 604):
            raise RuntimeError("unexpected baseline config or odometer")
        print("START_PREFLIGHT", "direction", args.direction,
              "kick_a", args.kick_a, "odometer", odom0,
              "saved", saved, flush=True)
        if args.check_only:
            print("CHECK_ONLY: no configuration or motor command sent", flush=True)
            return

        write_checked(bus, "focimax", 15)  # 1.5 A cap during torque kick
        write_checked(bus, "posrpm", 600)  # torque-mode velocity limit, 10 turns/s
        write_checked(bus, "drivemode", 10)  # existing FOC_TORQUE mode
        bus.set_input_torque(NODE, 0.0)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        time.sleep(0.18)
        h = bus.heartbeat(NODE)
        vi = bus.get_bus_voltage_current(NODE)
        pv = bus.get_encoder_estimates(NODE)
        iq = bus.get_iq(NODE)
        odom = bus.read("hoverboard.odometer")
        print("FOC_ZERO", h, "bus", vi, "encoder", pv,
              "iq", iq, "odometer", odom, flush=True)
        if (h is None or h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]
                or vi is None or not 34.0 <= vi[0] <= 43.0
                or pv is None or abs(pv[1]) > 0.1
                or iq is None or not all(map(math.isfinite, iq))
                or abs(iq[1]) > 0.2 or odom != odom0):
            raise RuntimeError("stationary FOC zero-current check failed")

        command_nm = direction * args.kick_a * kt
        bus.set_input_torque(NODE, command_nm)
        since = time.monotonic()
        previous_time = since
        previous_steps = 0
        print("FOC_KICK", "iq_target_a", direction * args.kick_a,
              "torque_field_nm", command_nm, "max_seconds", 0.8, flush=True)
        while time.monotonic() - since < 0.8:
            bus.set_input_torque(NODE, command_nm)
            time.sleep(0.008)
            pv, iq, steps = sample(bus, "kick", since, direction, odom0)
            now = time.monotonic()
            hall_rate = (steps - previous_steps) / 48.0 / max(now - previous_time, 0.001)
            previous_time, previous_steps = now, steps
            if abs(iq[1]) > 3.0 or steps > 48:
                raise RuntimeError("kick current or travel outside test range")
            if direction * iq[0] < -0.1:
                raise RuntimeError("torque velocity limit applied reverse braking")
            if steps >= 4 and hall_rate >= 2.5:
                bus.set_input_torque(NODE, 0.0)
                print("FOC_KICK_CUTOFF", "directed_hall_steps", steps,
                      "recent_hall_rate_tps", round(hall_rate, 3),
                      "encoder_speed_tps", pv[1], flush=True)
                break
        else:
            raise RuntimeError("FOC torque kick did not reach rolling handoff")

        bus.set_input_torque(NODE, 0.0)
        bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
        write_checked(bus, "focimax", 2)  # proven 0.2 A rolling-speed cap
        write_checked(bus, "drivemode", 9)  # existing FOC_SPEED mode
        rolling = bus.get_encoder_estimates(NODE)
        if rolling is None or not 3.0 <= direction * rolling[1] <= 9.0:
            raise RuntimeError("wheel left rolling handoff range")
        target = round(rolling[1], 1)
        bus.set_input_vel(NODE, target)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        print("FOC_HANDOFF", "rolling_encoder", rolling,
              "initial_target_tps", target, flush=True)
        speed_stage(bus, target, direction, odom0, 0.6, "hold")
        speed_stage(bus, direction * 6.0, direction, odom0, 0.8, "six_tps")
        if args.reverse_probe:
            reverse_probe(bus, odom0, args.reverse_cap_a)
            return
        if args.decel_only:
            speed_stage(bus, 2.0, direction, odom0, 1.5, "decel_two",
                        max_bus_v=41.0)
            speed_stage(bus, 0.0, direction, odom0, 2.0, "decel_zero",
                        max_bus_v=41.0)
            print("RESULT", "foc_deceleration_probe_complete", flush=True)
            return
        print("RESULT", "foc_start_and_speed_complete", flush=True)
    finally:
        try:
            if args.run and saved is not None:
                stop(bus, saved)
        finally:
            bus.s.close()


if __name__ == "__main__":
    main()
