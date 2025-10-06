import argparse
import csv
import math
import os
import sys
import time
from statistics import mean
from typing import Iterable, Optional


# ======================================================================
# Section: Imports and Path Setup
# ======================================================================
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.robot_systems.robot import HamBot


# ======================================================================
# Section: PID Parameters and Control Defaults
# ======================================================================
TARGET_DIST_M = 1.0             # desired clearance to the forward wall [m]
DEFAULT_LOOP_HZ = 15.0          # control loop frequency [Hz]
FRONT_INDEX = 180               # lidar index facing forward
FRONT_WINDOW = 7                # half-window (±) for averaging forward samples
DEFAULT_MAX_RPM = 35.0          # saturation limit for motor commands [RPM]
DEFAULT_MIN_EFFORT = 6.0        # minimum effort to overcome drivetrain friction [RPM]
DEFAULT_SETTLE_BAND = 0.01      # acceptable distance band around the setpoint [m]
DEFAULT_SETTLE_TIME = 1.0       # duration to remain in band before declaring success [s]
DEFAULT_TIMEOUT = 120.0         # safety timeout for the control loop [s]
DEFAULT_I_CLAMP = 0.7           # integral term clamp [m·s]
NEAR_TARGET_MIN_EFFORT = 0.8    # absolute floor for min effort near the target [RPM]
NEAR_TARGET_ERROR_BAND = 0.08   # error magnitude where we begin tapering the min effort [m]
DEFAULT_SLEW_RATE = 160.0       # maximum change in applied command per second [RPM/s]


# ======================================================================
# Section: Lidar Helpers
# ======================================================================
def _front_distance_m(range_image: Iterable[float],
                      center_idx: int = FRONT_INDEX,
                      window: int = FRONT_WINDOW) -> Optional[float]:
    """Return the averaged forward distance (meters) from the lidar scan."""
    if not range_image:
        return None

    samples = []
    scan = list(range_image)
    total = len(scan)
    if total == 0:
        return None

    for offset in range(-window, window + 1):
        idx = (center_idx + offset) % total
        dist_mm = scan[idx]
        if dist_mm and dist_mm > 0:
            samples.append(dist_mm / 1000.0)  # convert mm → m

    if not samples:
        return None

    return mean(samples)


# ======================================================================
# Section: PID Controller Loop
# ======================================================================
def forward_wall_stop(bot: HamBot,
                      kp: float,
                      ki: float,
                      kd: float,
                      target_m: float = TARGET_DIST_M,
                      loop_hz: float = DEFAULT_LOOP_HZ,
                      max_rpm: float = DEFAULT_MAX_RPM,
                      min_effort_rpm: float = DEFAULT_MIN_EFFORT,
                      settle_band_m: float = DEFAULT_SETTLE_BAND,
                      settle_time_s: float = DEFAULT_SETTLE_TIME,
                      timeout_s: float = DEFAULT_TIMEOUT,
                      integral_clamp: float = DEFAULT_I_CLAMP,
                      log_path: Optional[str] = None,
                      min_start_distance: Optional[float] = None,
                      max_start_distance: Optional[float] = None,
                      slew_rate_rpm_per_s: float = DEFAULT_SLEW_RATE) -> None:
    """
    Run a PID loop that drives toward (or backs up from) an end wall until
    the robot stabilizes at the requested clearance.
    """
    dt_target = 1.0 / max(loop_hz, 1e-3)
    last_loop = None
    integral = 0.0
    prev_error = None
    settle_start = None
    start_time = time.time()

    print("Starting PID wall-stop controller")
    print(f"  target distance   : {target_m:.3f} m")
    print(f"  gains (kp, ki, kd): ({kp:.4f}, {ki:.4f}, {kd:.4f})")
    print(f"  loop frequency    : {loop_hz:.1f} Hz")

    peak_abs_cmd = 0.0
    peak_abs_raw = 0.0
    first_valid_dist = None
    final_front = None
    saw_forward_cmd = False
    saw_reverse_cmd = False
    log_writer = None
    log_file_handle = None

    if log_path:
        log_file_handle = open(log_path, "w", newline="")
        log_writer = csv.writer(log_file_handle)
        log_writer.writerow([
            "timestamp_s",
            "front_m",
            "error_m",
            "p_term",
            "i_term",
            "d_term",
            "raw_cmd_rpm",
            "applied_cmd_rpm",
            "saturated",
            "min_effort_applied",
            "integral_clamped",
        ])

    prev_cmd = 0.0

    try:
        while True:
            loop_begin = time.time()
            scan = bot.get_range_image()
            front_m = _front_distance_m(scan)

            now = time.time()
            dt = dt_target if last_loop is None else max(1e-3, now - last_loop)
            last_loop = now

            if front_m is None:
                bot.stop_motors()
                print("No valid forward lidar samples; holding still")
                time.sleep(0.2)
                continue

            if first_valid_dist is None:
                first_valid_dist = front_m
                if (min_start_distance is not None and
                        front_m < min_start_distance):
                    print(
                        f"WARNING: start distance {front_m:.3f} m is below the minimum"
                        f" threshold ({min_start_distance:.3f} m)"
                    )
                if (max_start_distance is not None and
                        front_m > max_start_distance):
                    print(
                        f"WARNING: start distance {front_m:.3f} m exceeds the maximum"
                        f" threshold ({max_start_distance:.3f} m)"
                    )

            error = front_m - target_m
            integral = max(-integral_clamp, min(integral_clamp, integral + error * dt))
            integral_clamped = integral_clamp > 0 and abs(integral) >= (integral_clamp - 1e-6)
            derivative = 0.0 if prev_error is None else (error - prev_error) / dt
            prev_error = error

            p_term = kp * error
            i_term = ki * integral
            d_term = kd * derivative
            raw_cmd = p_term + i_term + d_term
            peak_abs_raw = max(peak_abs_raw, abs(raw_cmd))

            cmd = raw_cmd
            saturated = False
            if cmd > max_rpm:
                cmd = max_rpm
                saturated = True
            elif cmd < -max_rpm:
                cmd = -max_rpm
                saturated = True

            effective_min_effort = min_effort_rpm
            if abs(error) <= settle_band_m:
                effective_min_effort = 0.0
            elif abs(error) <= NEAR_TARGET_ERROR_BAND:
                scale = abs(error) / NEAR_TARGET_ERROR_BAND
                effective_min_effort = max(NEAR_TARGET_MIN_EFFORT,
                                           scale * min_effort_rpm)

            min_effort_applied = False
            if abs(cmd) < effective_min_effort:
                if effective_min_effort <= 0.0:
                    cmd = 0.0
                else:
                    direction = cmd if cmd != 0.0 else error
                    cmd = math.copysign(effective_min_effort, direction)
                    min_effort_applied = True

            if slew_rate_rpm_per_s > 0:
                max_delta = slew_rate_rpm_per_s * dt
                cmd = max(min(cmd, prev_cmd + max_delta), prev_cmd - max_delta)
            prev_cmd = cmd

            bot.set_left_motor_speed(cmd)
            bot.set_right_motor_speed(cmd)
            peak_abs_cmd = max(peak_abs_cmd, abs(cmd))
            if cmd > 1e-6:
                saw_forward_cmd = True
            elif cmd < -1e-6:
                saw_reverse_cmd = True

            elapsed = now - start_time
            if log_writer:
                log_writer.writerow([
                    elapsed,
                    front_m,
                    error,
                    p_term,
                    i_term,
                    d_term,
                    raw_cmd,
                    cmd,
                    int(saturated),
                    int(min_effort_applied),
                    int(integral_clamped),
                ])
                log_file_handle.flush()
            print(
                "t={:6.2f}s front={:5.3f} m err={:+.3f} m "
                "P={:+6.2f} I={:+6.2f} D={:+6.2f} raw={:+6.2f} rpm cmd={:+6.2f} rpm {}{}{}".format(
                    elapsed,
                    front_m,
                    error,
                    p_term,
                    i_term,
                    d_term,
                    raw_cmd,
                    cmd,
                    "[SAT]" if saturated else "",
                    "[BIAS]" if min_effort_applied else "",
                    "[I-CLAMP]" if integral_clamped else "",
                )
            )

            final_front = front_m

            if abs(error) <= settle_band_m and cmd == 0.0:
                if settle_start is None:
                    settle_start = now
                elif now - settle_start >= settle_time_s:
                    print("Setpoint maintained; stopping motors")
                    break
            else:
                settle_start = None

            if timeout_s and elapsed >= timeout_s:
                print(f"Timeout {timeout_s:.1f}s reached; stopping controller")
                break

            loop_elapsed = time.time() - loop_begin
            sleep_time = max(0.0, dt_target - loop_elapsed)
            time.sleep(sleep_time)
    finally:
        bot.stop_motors()
        elapsed_total = time.time() - start_time
        if final_front is not None:
            final_error = final_front - target_m
            print(
                f"Final distance: {final_front:.3f} m (error {final_error:+.3f} m) | "
                f"runtime {elapsed_total:.2f} s"
            )
        if first_valid_dist is not None:
            print(f"Start distance: {first_valid_dist:.3f} m")
        print(
            f"Peak raw cmd: {peak_abs_raw:.2f} rpm | Peak applied cmd: {peak_abs_cmd:.2f} rpm"
        )
        direction_report = []
        if saw_forward_cmd:
            direction_report.append("forward")
        if saw_reverse_cmd:
            direction_report.append("reverse")
        if direction_report:
            print("Observed motion commands: " + ", ".join(direction_report))
        if log_file_handle:
            log_file_handle.close()


# ======================================================================
# Section: CLI Entry Point
# ======================================================================
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Lab 2 Task 1: PID forward wall stop using HamBot lidar"
    )
    parser.add_argument("--kp", type=float, default=7.0, help="Proportional gain")
    parser.add_argument("--ki", type=float, default=0.02, help="Integral gain")
    parser.add_argument("--kd", type=float, default=0.8, help="Derivative gain")
    parser.add_argument("--target", type=float, default=TARGET_DIST_M,
                        help="Desired distance from the wall [m]")
    parser.add_argument("--loop-hz", type=float, default=DEFAULT_LOOP_HZ,
                        help="Control loop frequency [Hz]")
    parser.add_argument("--max-rpm", type=float, default=DEFAULT_MAX_RPM,
                        help="Motor command saturation [RPM]")
    parser.add_argument("--min-effort", type=float, default=DEFAULT_MIN_EFFORT,
                        help="Minimum effort to overcome stiction [RPM]")
    parser.add_argument("--settle-band", type=float, default=DEFAULT_SETTLE_BAND,
                        help="Settle band around the setpoint [m]")
    parser.add_argument("--settle-time", type=float, default=DEFAULT_SETTLE_TIME,
                        help="Required time in settle band before stopping [s]")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT,
                        help="Safety timeout for the controller [s]")
    parser.add_argument("--i-clamp", type=float, default=DEFAULT_I_CLAMP,
                        help="Integral windup clamp (|integral| limit) [m·s]")
    parser.add_argument("--log-file", type=str, default=None,
                        help="Optional CSV file to record each control loop sample")
    parser.add_argument("--min-start-dist", type=float, default=None,
                        help="Warn if initial distance is below this threshold [m]")
    parser.add_argument("--max-start-dist", type=float, default=None,
                        help="Warn if initial distance is above this threshold [m]")
    parser.add_argument("--slew-rate", type=float, default=DEFAULT_SLEW_RATE,
                        help="Max change in applied RPM per second (set <=0 to disable)")

    args = parser.parse_args()

    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    try:
        forward_wall_stop(bot,
                          kp=args.kp,
                          ki=args.ki,
                          kd=args.kd,
                          target_m=args.target,
                          loop_hz=args.loop_hz,
                          max_rpm=args.max_rpm,
                          min_effort_rpm=args.min_effort,
                          settle_band_m=args.settle_band,
                          settle_time_s=args.settle_time,
                          timeout_s=args.timeout,
                          integral_clamp=args.i_clamp,
                          log_path=args.log_file,
                          min_start_distance=args.min_start_dist,
                          max_start_distance=args.max_start_dist,
                          slew_rate_rpm_per_s=args.slew_rate)
    except KeyboardInterrupt:
        print("\nInterrupted; stopping motors")
    finally:
        bot.stop_motors()
        try:
            bot.disconnect_robot()
        except Exception:
            pass


if __name__ == "__main__":
    main()
