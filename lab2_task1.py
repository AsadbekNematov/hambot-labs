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
                      integral_clamp: float = DEFAULT_I_CLAMP) -> None:
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

            error = front_m - target_m
            integral = max(-integral_clamp, min(integral_clamp, integral + error * dt))
            integral_clamped = integral_clamp > 0 and abs(integral) >= (integral_clamp - 1e-6)
            derivative = 0.0 if prev_error is None else (error - prev_error) / dt
            prev_error = error

            p_term = kp * error
            i_term = ki * integral
            d_term = kd * derivative
            raw_cmd = p_term + i_term + d_term

            cmd = raw_cmd
            saturated = False
            if cmd > max_rpm:
                cmd = max_rpm
                saturated = True
            elif cmd < -max_rpm:
                cmd = -max_rpm
                saturated = True

            min_effort_applied = False
            if abs(cmd) < min_effort_rpm:
                if abs(error) <= settle_band_m:
                    cmd = 0.0
                else:
                    direction = cmd if cmd != 0.0 else error
                    cmd = math.copysign(min_effort_rpm, direction)
                    min_effort_applied = True

            bot.set_left_motor_speed(cmd)
            bot.set_right_motor_speed(cmd)

            elapsed = now - start_time
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
                          integral_clamp=args.i_clamp)
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
