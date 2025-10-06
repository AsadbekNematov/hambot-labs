import argparse
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
TARGET_DIST_M = 1.0           # desired clearance to the forward wall [m]
DEFAULT_LOOP_HZ = 15.0        # control loop frequency [Hz]
FRONT_INDEX = 180             # lidar index facing forward
FRONT_WINDOW = 7              # half-window (±) for averaging forward samples
DEFAULT_MAX_RPM = 35.0        # saturation limit for motor commands [RPM]
DEFAULT_MIN_EFFORT = 6.0      # minimum effort to overcome drivetrain friction [RPM]
DEFAULT_SETTLE_BAND = 0.03    # acceptable distance band around the setpoint [m]
DEFAULT_SETTLE_TIME = 1.0     # duration to remain in band before declaring success [s]
DEFAULT_TIMEOUT = 120.0       # safety timeout for the control loop [s]
DEFAULT_I_CLAMP = 0.7         # integral term clamp [m·s]
MOTOR_MAX_RPM = 75.0          # HamBot motor driver clamp (see HamBot.check_speed)
DEFAULT_APPROACH_ZONE_M = 0.08   # slowdown starts only in the final few centimeters [m]
DEFAULT_APPROACH_FLOOR = 0.10    # fraction of max speed allowed when sitting at the target


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
                      max_speed_pct: Optional[float] = None,
                      min_effort_rpm: float = DEFAULT_MIN_EFFORT,
                      settle_band_m: float = DEFAULT_SETTLE_BAND,
                      settle_time_s: float = DEFAULT_SETTLE_TIME,
                      timeout_s: float = DEFAULT_TIMEOUT,
                      integral_clamp: float = DEFAULT_I_CLAMP,
                      meas_alpha: float = 1.0,
                      deriv_alpha: float = 1.0,
                      stop_mode: str = "settle",
                      reach_tol: float = 0.005,
                      reach_confirm: int = 3,
                      approach_zone_m: float = DEFAULT_APPROACH_ZONE_M,
                      approach_floor: float = DEFAULT_APPROACH_FLOOR) -> None:
    """
    Run a PID loop that drives toward (or backs up from) an end wall until
    the robot stabilizes at the requested clearance.

    If ``approach_zone_m`` is positive the controller linearly tapers the
    allowable motor output from 100% when outside the zone down to at least
    ``approach_floor`` (fraction of the allowed top speed) right at the
    target. ``max_speed_pct`` provides an optional percentage-based way to
    configure the top speed relative to the HamBot's 75 rpm ceiling.
    """
    if max_speed_pct is not None:
        max_speed_pct = max(0.0, min(100.0, max_speed_pct))
        max_rpm = (max_speed_pct / 100.0) * MOTOR_MAX_RPM
    else:
        max_speed_pct = (max_rpm / MOTOR_MAX_RPM) * 100.0 if MOTOR_MAX_RPM > 1e-6 else 0.0

    max_rpm = max(0.0, min(MOTOR_MAX_RPM, max_rpm))
    approach_zone_m = max(0.0, approach_zone_m)
    approach_floor = max(0.0, min(1.0, approach_floor))
    floor_from_min_effort = min(1.0, min_effort_rpm / max_rpm) if max_rpm > 1e-6 else 0.0
    effective_floor = max(approach_floor, floor_from_min_effort)

    dt_target = 1.0 / max(loop_hz, 1e-3)
    last_loop = None
    integral = 0.0
    prev_error = None
    # smoothed forward distance (exponential moving average)
    smoothed_front = None
    # filtered derivative (low-pass)
    prev_filtered_derivative = None
    settle_start = None
    # counter used by 'reach' stop mode to confirm consecutive in-tolerance readings
    reach_count = 0
    start_time = time.time()

    settle_stop_tol = settle_band_m
    if stop_mode == "settle" and reach_tol is not None and reach_tol > 0.0:
        settle_stop_tol = min(settle_band_m, reach_tol)

    print("Starting PID wall-stop controller")
    print(f"  target distance   : {target_m:.3f} m")
    print(f"  gains (kp, ki, kd): ({kp:.4f}, {ki:.4f}, {kd:.4f})")
    print(f"  loop frequency    : {loop_hz:.1f} Hz")
    print(f"  max command       : ±{max_rpm:.1f} rpm ({max_speed_pct:.1f}% of {MOTOR_MAX_RPM:.0f} rpm full-scale)")
    if approach_zone_m > 0.0:
        requested_floor_pct = approach_floor * 100.0
        effective_floor_pct = effective_floor * 100.0
        print(f"  approach slowdown : start {approach_zone_m:.2f} m out, floor {requested_floor_pct:.1f}%")
        if effective_floor > approach_floor:
            print(f"    adjusted floor  : {effective_floor_pct:.1f}% (limited by min-effort {min_effort_rpm:.1f} rpm)")
    print(f"  stop mode         : {stop_mode}")
    if stop_mode == "settle":
        print(f"    settle band     : ±{settle_band_m:.3f} m (stop tol ±{settle_stop_tol:.3f} m)")
    else:
        print(f"    reach tolerance : ±{reach_tol:.3f} m with {reach_confirm} confirmations")

    try:
        while True:
            loop_begin = time.time()
            scan = bot.get_range_image()
            front_m = _front_distance_m(scan)

            # clamp alphas
            meas_alpha = max(0.0, min(1.0, meas_alpha))
            deriv_alpha = max(0.0, min(1.0, deriv_alpha))

            now = time.time()
            dt = dt_target if last_loop is None else max(1e-3, now - last_loop)
            last_loop = now

            if front_m is None:
                bot.stop_motors()
                print("No valid forward lidar samples; holding still")
                time.sleep(0.2)
                continue

            # exponential smoothing on the forward measurement
            if smoothed_front is None:
                smoothed_front = front_m
            else:
                smoothed_front = meas_alpha * front_m + (1.0 - meas_alpha) * smoothed_front

            used_front = smoothed_front

            error = used_front - target_m
            distance_error = abs(error)

            if approach_zone_m > 1e-6:
                blend = min(1.0, distance_error / approach_zone_m)
                speed_scale = effective_floor + (1.0 - effective_floor) * blend
            else:
                speed_scale = 1.0
            if approach_zone_m > 0.0:
                speed_scale = max(effective_floor, min(1.0, speed_scale))
            else:
                speed_scale = max(0.0, min(1.0, speed_scale))

            current_max_rpm = max_rpm * speed_scale

            integral = max(-integral_clamp, min(integral_clamp, integral + error * dt))
            integral_clamped = integral_clamp > 0 and abs(integral) >= (integral_clamp - 1e-6)

            # derivative based on filtered error
            derivative = 0.0 if prev_error is None else (error - prev_error) / dt
            prev_error = error

            # low-pass filter the derivative to reduce noise
            if prev_filtered_derivative is None:
                filtered_deriv = derivative
            else:
                filtered_deriv = deriv_alpha * derivative + (1.0 - deriv_alpha) * prev_filtered_derivative
            prev_filtered_derivative = filtered_deriv

            p_term = kp * error
            i_term = ki * integral
            d_term = kd * filtered_deriv
            raw_cmd = p_term + i_term + d_term

            cmd = raw_cmd
            saturated = False
            if cmd > current_max_rpm:
                cmd = current_max_rpm
                saturated = True
            elif cmd < -current_max_rpm:
                cmd = -current_max_rpm
                saturated = True

            effective_min_effort_rpm = min(min_effort_rpm, current_max_rpm)
            min_effort_applied = False
            if effective_min_effort_rpm > 0.0 and abs(cmd) < effective_min_effort_rpm:
                # In 'settle' mode we may zero small commands when already within the settle band.
                # In 'reach' mode we do NOT zero small commands here because we want the robot to
                # continue moving until it reaches the precise target (reach_tol). Instead always
                # apply the minimum effort to overcome stiction so the robot can close the final gap.
                if stop_mode == "settle":
                    if abs(error) <= settle_stop_tol:
                        cmd = 0.0
                    else:
                        direction = cmd if cmd != 0.0 else error
                        cmd = math.copysign(effective_min_effort_rpm, direction)
                        min_effort_applied = True
                else:
                    # default/other modes (including 'reach'): always apply minimum effort
                    direction = cmd if cmd != 0.0 else error
                    cmd = math.copysign(effective_min_effort_rpm, direction)
                    min_effort_applied = True

            bot.set_left_motor_speed(cmd)
            bot.set_right_motor_speed(cmd)

            elapsed = now - start_time
            raw_pct = (raw_cmd / MOTOR_MAX_RPM) * 100.0 if MOTOR_MAX_RPM > 1e-6 else 0.0
            cmd_pct = (cmd / MOTOR_MAX_RPM) * 100.0 if MOTOR_MAX_RPM > 1e-6 else 0.0
            max_pct = (current_max_rpm / MOTOR_MAX_RPM) * 100.0 if MOTOR_MAX_RPM > 1e-6 else 0.0
            slowdown_flag = "[SLOW]" if speed_scale < 0.999 else ""
            print(
                "t={:6.2f}s front={:5.3f} m err={:+.3f} m "
                "P={:+6.2f} I={:+6.2f} D={:+6.2f} raw={:+6.2f} rpm ({:+6.1f}%) "
                "cmd={:+6.2f} rpm ({:+6.1f}%) max={:5.1f}% {}{}{}{}".format(
                    elapsed,
                    used_front,
                    error,
                    p_term,
                    i_term,
                    d_term,
                    raw_cmd,
                    raw_pct,
                    cmd,
                    cmd_pct,
                    max_pct,
                    "[SAT]" if saturated else "",
                    "[BIAS]" if min_effort_applied else "",
                    "[I-CLAMP]" if integral_clamped else "",
                    slowdown_flag,
                )
            )

            if stop_mode == "settle":
                if abs(error) <= settle_stop_tol and cmd == 0.0:
                    if settle_start is None:
                        settle_start = now
                    elif now - settle_start >= settle_time_s:
                        print("Setpoint maintained; stopping motors")
                        break
                else:
                    settle_start = None
            elif stop_mode == "reach":
                # require several consecutive measurements within reach_tol to avoid false triggers
                if abs(error) <= reach_tol:
                    reach_count += 1
                else:
                    reach_count = 0

                if reach_count >= max(1, int(reach_confirm)):
                    print(f"Reached target (|err|<={reach_tol}) for {reach_count} loops; stopping motors")
                    break
            else:
                # unknown stop mode: default to settle behavior
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


# ======================================================================
# Section: CLI Entry Point
# ======================================================================
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Lab 2 Task 1: PID forward wall stop using HamBot lidar"
    )
    parser.add_argument("--kp", type=float, default=5.0, help="Proportional gain")
    parser.add_argument("--ki", type=float, default=0.0, help="Integral gain")
    parser.add_argument("--kd", type=float, default=0.8, help="Derivative gain")
    parser.add_argument("--target", type=float, default=TARGET_DIST_M,
                        help="Desired distance from the wall [m]")
    parser.add_argument("--loop-hz", type=float, default=DEFAULT_LOOP_HZ,
                        help="Control loop frequency [Hz]")
    parser.add_argument("--max-rpm", type=float, default=DEFAULT_MAX_RPM,
                        help="Motor command saturation [RPM] (overridden by --max-speed-pct if provided)")
    parser.add_argument("--max-speed-pct", type=float, default=None,
                        help="Motor command saturation as a percentage of full-scale ({:.0f} rpm)".format(MOTOR_MAX_RPM))
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
    parser.add_argument("--meas-alpha", type=float, default=1.0,
                        help="Exponential smoothing alpha for range measurement (0..1). 1.0=no smoothing")
    parser.add_argument("--deriv-alpha", type=float, default=1.0,
                        help="Low-pass alpha for derivative term (0..1). 1.0=no filtering")
    parser.add_argument("--stop-mode", type=str, default="settle",
                        choices=["settle", "reach"],
                        help="How to decide when to stop: 'settle' waits in-band; 'reach' stops when within reach-tol")
    parser.add_argument("--reach-tol", type=float, default=0.005,
                        help="Tolerance [m] for 'reach' stop mode (default 0.005 m)")
    parser.add_argument("--reach-confirm", type=int, default=3,
                        help="Number of consecutive loops within reach-tol required to confirm reach (default 3)")
    parser.add_argument("--approach-zone", type=float, default=DEFAULT_APPROACH_ZONE_M,
                        help="Distance from target where speed begins tapering [m]")
    parser.add_argument("--approach-floor-pct", type=float, default=DEFAULT_APPROACH_FLOOR * 100.0,
                        help="Minimum percent of top speed to retain when on target (0-100)")

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
                          max_speed_pct=args.max_speed_pct,
                          min_effort_rpm=args.min_effort,
                          settle_band_m=args.settle_band,
                          settle_time_s=args.settle_time,
                          timeout_s=args.timeout,
                          integral_clamp=args.i_clamp,
                          meas_alpha=args.meas_alpha,
                          deriv_alpha=args.deriv_alpha,
                          stop_mode=args.stop_mode,
                          reach_tol=args.reach_tol,
                          reach_confirm=args.reach_confirm,
                          approach_zone_m=args.approach_zone,
                          approach_floor=args.approach_floor_pct / 100.0)
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
