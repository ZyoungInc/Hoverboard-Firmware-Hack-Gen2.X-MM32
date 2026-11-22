#!/usr/bin/env python3
"""Rolling-start FOC speed sweep using the existing CANSimple UART controls.

The verified block-voltage mode starts the suspended wheel at 7%. The script
then coasts briefly, arms FOC_SPEED near its actual speed, and raises the speed
target in stages. All configuration writes are RAM-only. --run is explicit.
"""

import argparse
from collections import deque
import hashlib
import math
from pathlib import Path
import statistics
import time

from cansimple_uart import AXIS_STATE, Bus
from ghost_foc_current_probe import NODE, preflight


class SwdDiagnostics:
    """Read only counters from the identified, currently flashed SPIN05 image.

    Addresses belong to the ELF used for the exact combined image below. An
    image fingerprint is checked on the running target before any RAM read is
    trusted. The probe attaches without halting or writing target memory.
    """

    IMAGE = (Path(__file__).resolve().parents[1] / "HoverBoardMindMotion" /
             "build/spin05/hoverboard-ghost-gen2_4_2-foc3000-sinekv604-id1.bin")
    IMAGE_SHA256 = "aac10239897463e215d5b753030d91ad87d0cd0f0635784c11da545767acc074"
    FLASH_BASE = 0x08000000
    FINGERPRINT_OFFSETS = (0, 0x1000, 0x4000, 0x7000)
    COUNTERS = {"glitches": 0x200000f6,
                "foldbacks": 0x20000356,
                "hall_invalid": 0x20000350}

    def __init__(self):
        from pyocd.core.helpers import ConnectHelper

        image = self.IMAGE.read_bytes()
        if hashlib.sha256(image).hexdigest() != self.IMAGE_SHA256:
            raise RuntimeError("SWD reference firmware image has changed")
        self.session = ConnectHelper.session_with_chosen_probe(
            target_override="cortex_m", frequency=10000,
            options={"connect_mode": "attach", "resume_on_disconnect": True,
                     "auto_unlock": False})
        self.session.open()
        try:
            target = self.session.target
            if target.get_state().name != "RUNNING":
                raise RuntimeError("SWD target is not running")
            for offset in self.FINGERPRINT_OFFSETS:
                actual = bytes(target.read_memory_block8(self.FLASH_BASE + offset, 16))
                if actual != image[offset:offset + 16]:
                    raise RuntimeError("SWD target firmware fingerprint differs")
            self.last = None
            print("SWD_IDENTIFIED", self.IMAGE.name, "core", "RUNNING", flush=True)
        except BaseException:
            self.session.close()
            raise

    def snapshot(self, label):
        target = self.session.target
        if target.get_state().name != "RUNNING":
            raise RuntimeError("SWD target stopped running")
        current = {name: target.read16(address)
                   for name, address in self.COUNTERS.items()}
        delta = (None if self.last is None else
                 {name: (value - self.last[name]) & 0xffff
                  for name, value in current.items()})
        print("SWD_COUNTERS", "stage", label, "total", current,
              "delta", delta, flush=True)
        self.last = current

    def close(self):
        self.session.close()


def write_checked(bus, name, value):
    path = "hoverboard.config." + name
    if not bus.write(path, value) or bus.read(path) != value:
        raise RuntimeError("RAM configuration failed: " + name)


def stop(bus, saved):
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
    bus.set_input_vel(NODE, 0.0)
    bus.set_input_torque(NODE, 0.0)
    bus.set_input_voltage(NODE, 0.0)
    for name in ("drivemode", "posrpm", "focimax", "focspdkp", "focspdki"):
        if name in saved:
            try:
                write_checked(bus, name, saved[name])
            except Exception as exc:
                print("RESTORE_FAILED", name, exc, flush=True)
    print("STOP", "heartbeat", bus.heartbeat(NODE), "error", bus.get_error(NODE),
          "bus", bus.get_bus_voltage_current(NODE),
          "encoder", bus.get_encoder_estimates(NODE),
          "odometer", bus.read("hoverboard.odometer"),
          "mode", bus.read("hoverboard.config.drivemode"), flush=True)


def check_feedback(h, vi, pv, odom, limit_tps=17.0):
    if h is None or vi is None or pv is None or odom is None:
        raise RuntimeError("telemetry lost")
    if h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]:
        raise RuntimeError("firmware fault or axis disarmed")
    if not all(map(math.isfinite, (*vi, *pv))):
        raise RuntimeError("non-finite telemetry")
    if not 34.0 <= vi[0] <= 43.0 or abs(vi[1]) > 3.0:
        raise RuntimeError("bus voltage or current outside test range")
    if pv[1] < -0.2 or (limit_tps is not None and pv[1] > limit_tps):
        raise RuntimeError("unexpected speed or direction")


def foc_stage(bus, target, seconds, current_cap, speed_limit, poll_sleep=0.012):
    began = time.monotonic()
    next_print = began
    speeds = []
    currents = []
    hall_window = deque()
    print("FOC_STAGE", "target_tps", target, "seconds", seconds,
          "current_cap_a", current_cap, "poll_sleep_s", poll_sleep, flush=True)
    while time.monotonic() - began < seconds:
        bus.set_input_vel(NODE, target)
        time.sleep(poll_sleep)
        h = bus.heartbeat(NODE)
        vi = bus.get_bus_voltage_current(NODE)
        pv = bus.get_encoder_estimates(NODE)
        iq = bus.get_iq(NODE)
        odom = bus.read("hoverboard.odometer")
        now = time.monotonic()
        hall_speed = None
        if odom is not None:
            hall_window.append((now, odom))
            while hall_window and now - hall_window[0][0] > 0.25:
                hall_window.popleft()
            if hall_window and now - hall_window[0][0] >= 0.12:
                first_time, first_odom = hall_window[0]
                hall_speed = (odom - first_odom) / 48.0 / (now - first_time)
        if (h is not None and vi is not None and pv is not None and odom is not None
                and (not 34.0 <= vi[0] <= 43.0 or abs(vi[1]) > 3.0
                     or pv[1] < -0.2 or (pv[1] > speed_limit
                     and (hall_speed is None or hall_speed > speed_limit
                          or pv[1] > speed_limit + 5.0)))):
            print("FOC_LIMIT_SAMPLE", "target_tps", target,
                  "elapsed_s", round(now - began, 3),
                  "speed_tps", pv[1], "iq_set_meas_a", iq,
                  "bus", vi, "odometer", odom,
                  "hall_window_tps", hall_speed,
                  "limit_tps", speed_limit, flush=True)
        if pv is not None and pv[1] > speed_limit and hall_speed is not None and hall_speed <= speed_limit and pv[1] <= speed_limit + 5.0:
            print("FOC_SPEED_OUTLIER", "target_tps", target,
                  "elapsed_s", round(now - began, 3),
                  "encoder_tps", pv[1], "hall_window_tps", hall_speed,
                  "odometer", odom, flush=True)
            check_feedback(h, vi, pv, odom, limit_tps=None)
        else:
            check_feedback(h, vi, pv, odom, limit_tps=speed_limit)
        if iq is None or not all(map(math.isfinite, iq)):
            raise RuntimeError("phase-current telemetry unavailable")
        if abs(iq[1]) > min(0.86, current_cap * 1.8 + 0.5):
            print("FOC_LIMIT_SAMPLE", "target_tps", target,
                  "elapsed_s", round(now - began, 3),
                  "speed_tps", pv[1], "iq_set_meas_a", iq,
                  "bus", vi, "odometer", odom, flush=True)
            raise RuntimeError("measured phase current outside test range")
        speeds.append(pv[1])
        currents.append(iq[1])
        if now >= next_print:
            print("FOC_SAMPLE", "target_tps", target,
                  "elapsed_s", round(now - began, 3),
                  "speed_tps", round(pv[1], 3),
                  "iq_set_meas_a", tuple(round(x, 3) for x in iq),
                  "bus", tuple(round(x, 3) for x in vi),
                  "odometer", odom, "errors", h["errors"], flush=True)
            next_print = now + 0.20
    if not speeds:
        raise RuntimeError("no FOC speed sample")
    tail = speeds[-min(5, len(speeds)):]
    median_speed = statistics.median(tail)
    print("FOC_STAGE_RESULT", "target_tps", target,
          "median_last_speed_tps", round(median_speed, 3),
          "min_max_iq_a", (round(min(currents), 3), round(max(currents), 3)),
          "samples", len(speeds), flush=True)
    if abs(median_speed - target) > 1.0:
        raise RuntimeError("FOC speed target not tracked")


def rolling_torque(bus, torque_a, kt):
    began = time.monotonic()
    previous_odom = bus.read("hoverboard.odometer")
    print("FOC_TORQUE_ROLLING", "iq_target_a", torque_a,
          "duration_s", 0.8, flush=True)
    while time.monotonic() - began < 0.8:
        bus.set_input_torque(NODE, torque_a * kt)
        time.sleep(0.012)
        h = bus.heartbeat(NODE)
        vi = bus.get_bus_voltage_current(NODE)
        pv = bus.get_encoder_estimates(NODE)
        iq = bus.get_iq(NODE)
        fast = bus.read("hoverboard.fast_speed")
        odom = bus.read("hoverboard.odometer")
        check_feedback(h, vi, pv, odom, limit_tps=10.0)
        if iq is None or fast is None or not all(map(math.isfinite, iq)):
            raise RuntimeError("FOC current or fast-speed telemetry unavailable")
        print("FOC_TORQUE_SAMPLE", "elapsed_s", round(time.monotonic() - began, 3),
              "speed_tps", round(pv[1], 3), "fast_rpm", fast,
              "iq_set_meas_a", tuple(round(x, 3) for x in iq),
              "bus", tuple(round(x, 3) for x in vi),
              "odometer", odom, flush=True)
        if (abs(iq[0]-torque_a) > 0.25
                or abs(iq[1]) > max(1.5, torque_a*2.0)
                or odom < previous_odom - 1):
            raise RuntimeError("rolling current or Hall direction inconsistent")
        previous_odom = odom
    print("RESULT", "rolling_torque_complete", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    modes = ap.add_mutually_exclusive_group(required=True)
    modes.add_argument("--check-only", action="store_true")
    modes.add_argument("--run", action="store_true")
    modes.add_argument("--stop", action="store_true", help="IDLE and restore baseline RAM config")
    ap.add_argument("--targets", default="8,10,12,15",
                    help="ascending FOC targets in mechanical turns/s; --high permits up to 40")
    ap.add_argument("--stage-seconds", type=float, default=1.0)
    ap.add_argument("--current-cap-a", type=float, default=0.2)
    ap.add_argument("--high", action="store_true",
                    help="allow targets above the previous 15 turns/s test, with 3000 rpm firmware")
    ap.add_argument("--high-speed-kp", type=int,
                    help="RAM FOC speed Kp (mA/rpm) from the 20 turns/s stage")
    ap.add_argument("--high-speed-ki", type=int,
                    help="RAM FOC speed Ki (mA/rpm/s) from the 20 turns/s stage")
    ap.add_argument("--high-current-cap-a", type=float,
                    help="RAM Iq cap from the 25 turns/s stage; current sample guard stays 0.86 A")
    ap.add_argument("--swd-diagnostics", action="store_true",
                    help="read-only, non-halting SWD glitch/foldback/Hall counter deltas per stage")
    ap.add_argument("--low-traffic-compare", action="store_true",
                    help="after a final 20 turns/s stage, repeat it with 10 Hz rather than 30 Hz telemetry")
    ap.add_argument("--rolling-torque-zero", action="store_true",
                    help="isolate the FOC current loop at rolling speed, with zero torque")
    ap.add_argument("--rolling-torque-a", type=float,
                    help="isolate FOC current loop with constant 0..1.5 A Iq while rolling")
    args = ap.parse_args()
    try:
        targets = [float(x) for x in args.targets.split(",")]
    except ValueError:
        ap.error("--targets must be comma-separated numbers")
    if (not targets or any(not 6.0 <= x <= (40.0 if args.high else 15.0) for x in targets)
            or any(a >= b for a, b in zip(targets, targets[1:]))
            or not 0.5 <= args.stage_seconds <= 3.0
            or not 0.2 <= args.current_cap_a <= 5.0
            or (args.high_speed_kp is None) != (args.high_speed_ki is None)
            or (args.high_speed_kp is not None
                and not (1 <= args.high_speed_kp <= 30
                         and 1 <= args.high_speed_ki <= 100))
            or (args.high_current_cap_a is not None
                and not 0.2 <= args.high_current_cap_a <= 0.5)
            or (args.rolling_torque_a is not None
                and not 0.0 <= args.rolling_torque_a <= 1.5)
            or (args.low_traffic_compare
                and (not args.swd_diagnostics or targets[-1] != 20.0))):
        ap.error("invalid FOC speed plan or current cap")
    rolling_torque_a = (args.rolling_torque_a if args.rolling_torque_a is not None
                        else 0.0 if args.rolling_torque_zero else None)

    bus = Bus(args.port, 115200, timeout=0.25)
    saved = None
    swd = None
    active_stage = "pre_arm"
    try:
        if args.stop:
            stop(bus, {"drivemode": 1, "posrpm": 65535, "focimax": 400})
            return
        _, kt = preflight(bus)
        saved = {name: bus.read("hoverboard.config." + name)
                 for name in ("drivemode", "posrpm", "focimax")}
        sinekv = bus.read("hoverboard.config.sinekv")
        odom = bus.read("hoverboard.odometer")
        timeout_ms = (bus.read("hoverboard.config.timeout")
                      if args.low_traffic_compare else None)
        if None in saved.values() or odom is None or sinekv != 604 or saved != {
                "drivemode": 1, "posrpm": 65535, "focimax": 400}:
            raise RuntimeError("unexpected baseline config, sinekv or odometer")
        if args.low_traffic_compare and (timeout_ms is None or timeout_ms < 400):
            raise RuntimeError("serial command timeout too short for low-traffic comparison")
        if args.high_speed_kp is not None:
            for name in ("focspdkp", "focspdki"):
                saved[name] = bus.read("hoverboard.config." + name)
            if saved["focspdkp"] != 65535 or saved["focspdki"] != 65535:
                raise RuntimeError("unexpected baseline FOC speed gains")
        print("SWEEP_PREFLIGHT", "config", saved, "odometer", odom,
              "sinekv", sinekv,
              "targets_tps", targets, "current_cap_a", args.current_cap_a,
              "high_current_cap_a", args.high_current_cap_a,
              flush=True)
        if args.swd_diagnostics:
            swd = SwdDiagnostics()
            swd.snapshot("preflight")
        if args.check_only:
            print("CHECK_ONLY: no configuration or motor command sent", flush=True)
            return

        write_checked(bus, "focimax", round(args.current_cap_a * 10))
        write_checked(bus, "posrpm", max(900, math.ceil(max(targets) * 60)))
        write_checked(bus, "drivemode", 9)  # FOC_SPEED
        bus.set_input_vel(NODE, 0.0)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        time.sleep(0.18)  # phase-current zero offsets at standstill
        h = bus.heartbeat(NODE)
        iq = bus.get_iq(NODE)
        pv = bus.get_encoder_estimates(NODE)
        print("FOC_ZERO", h, "iq", iq, "encoder", pv, flush=True)
        if (h is None or h["errors"] or h["state"] != AXIS_STATE["CLOSED_LOOP_CONTROL"]
                or iq is None or not all(map(math.isfinite, iq))
                or pv is None or abs(pv[1]) > 0.2):
            raise RuntimeError("FOC zero-current preparation failed")
        bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
        bus.set_input_vel(NODE, 0.0)

        write_checked(bus, "drivemode", 0)  # verified block voltage
        bus.set_input_voltage(NODE, 0.0)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        began = time.monotonic()
        next_print = began
        spin_odom = bus.read("hoverboard.odometer")
        print("BLOCK_SPINUP", "fraction", 0.07, "max_seconds", 2.5, flush=True)
        while True:
            bus.set_input_voltage(NODE, 0.07)
            time.sleep(0.012)
            h = bus.heartbeat(NODE)
            vi = bus.get_bus_voltage_current(NODE)
            pv = bus.get_encoder_estimates(NODE)
            odom = bus.read("hoverboard.odometer")
            now = time.monotonic()
            check_feedback(h, vi, pv, odom, limit_tps=10.0)
            if now >= next_print:
                print("BLOCK_SAMPLE", "elapsed_s", round(now - began, 3),
                      "speed_tps", round(pv[1], 3), "bus", vi,
                      "hall_steps", odom - spin_odom, flush=True)
                next_print = now + 0.20
            if pv[1] >= 5.5 and odom - spin_odom >= 24:
                break
            if now - began > 2.5:
                raise RuntimeError("block-voltage spinup did not reach rolling speed")

        bus.set_axis_state(NODE, AXIS_STATE["IDLE"])
        bus.set_input_voltage(NODE, 0.0)
        write_checked(bus, "drivemode", 10 if rolling_torque_a is not None else 9)
        rolling = bus.get_encoder_estimates(NODE)
        if rolling is None or not 3.0 <= rolling[1] <= 9.0:
            raise RuntimeError("wheel left rolling handoff range")
        initial = round(rolling[1], 1)
        if rolling_torque_a is not None:
            bus.set_input_torque(NODE, rolling_torque_a * kt)
            bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
            print("FOC_HANDOFF", "mode", "TORQUE", "iq_target_a", rolling_torque_a,
                  "rolling_encoder", rolling, flush=True)
            rolling_torque(bus, rolling_torque_a, kt)
            return
        bus.set_input_vel(NODE, initial)
        bus.set_axis_state(NODE, AXIS_STATE["CLOSED_LOOP_CONTROL"])
        print("FOC_HANDOFF", "initial_target_tps", initial,
              "rolling_encoder", rolling, flush=True)
        speed_limit = max(targets) + 2.0
        if swd is not None:
            swd.snapshot("before_foc")
        active_stage = "handoff"
        foc_stage(bus, initial, 0.8, args.current_cap_a, speed_limit)
        if swd is not None:
            swd.snapshot("handoff")
        active_stage = None
        tuned = False
        raised_cap = False
        active_current_cap = args.current_cap_a
        for target in targets:
            if args.high_speed_kp is not None and target >= 20.0 and not tuned:
                write_checked(bus, "focspdkp", args.high_speed_kp)
                write_checked(bus, "focspdki", args.high_speed_ki)
                print("GAIN_TUNE", "from_target_tps", target,
                      "kp_mA_per_rpm", args.high_speed_kp,
                      "ki_mA_per_rpm_s", args.high_speed_ki, flush=True)
                tuned = True
            if args.high_current_cap_a is not None and target >= 25.0 and not raised_cap:
                write_checked(bus, "focimax", round(args.high_current_cap_a * 10))
                active_current_cap = args.high_current_cap_a
                print("CURRENT_CAP_TUNE", "from_target_tps", target,
                      "cap_a", active_current_cap, flush=True)
                raised_cap = True
            active_stage = str(target)
            foc_stage(bus, target, args.stage_seconds, active_current_cap, speed_limit)
            if swd is not None:
                swd.snapshot(active_stage)
            active_stage = None
        if args.low_traffic_compare:
            active_stage = "20.0_low_traffic"
            print("UART_TRAFFIC_COMPARE", "target_tps", 20.0,
                  "poll_sleep_s", 0.09, "watchdog_timeout_ms", timeout_ms, flush=True)
            foc_stage(bus, 20.0, args.stage_seconds, active_current_cap,
                      speed_limit, poll_sleep=0.09)
            swd.snapshot(active_stage)
            active_stage = None
        print("RESULT", "FOC_speed_sweep_complete", flush=True)
    finally:
        try:
            if saved is not None and args.run:
                stop(bus, saved)
        finally:
            try:
                if swd is not None and args.run:
                    try:
                        swd.snapshot("after_stop_" + str(active_stage))
                    except Exception as exc:
                        print("SWD_ERROR", repr(exc), flush=True)
            finally:
                if swd is not None:
                    swd.close()
                bus.s.close()


if __name__ == "__main__":
    main()
