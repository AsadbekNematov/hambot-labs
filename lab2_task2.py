"""
HamBot Lab 2 Task 2 – minimal left-wall follower.

The script keeps only the logic required to trail the left wall. When the robot
drifts toward the wall it steers right a little, and when it drifts away it
steers left to tighten the gap again.
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import Iterable, List, Optional, Tuple

# Allow running the controller directly from the repository root without
# modifying PYTHONPATH externally.
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.robot_systems.robot import HamBot  # noqa: E402 (import after sys.path)


# Lidar sampling windows around the robot.
FRONT_WIN: Tuple[int, int] = (175, 185)
LEFT_WIN: Tuple[int, int] = (90, 115)
RIGHT_WIN: Tuple[int, int] = (245, 270)
MAX_RANGE: float = 4.0

# Simplified motion parameters.
SIDE_TARGET_M: float = 0.22
SIDE_DEADBAND_M: float = 0.005
BASE_FWD_RPM: float = 12.0
STEER_KP: float = 20.0
MAX_STEER_RPM: float = 12.0
MAX_RPM: float = 35.0
MIN_EFFORT_RPM: float = 6.0

# Obstacle handling thresholds.
FRONT_BLOCK_M: float = 0.25
RIGHT_CLEAR_M: float = 0.40

# Turn behaviour.
TURN_KP: float = 0.6
TURN_FAST_RPM: float = 24.0
TURN_SLOW_RPM: float = 12.0
TURN_SLOW_BAND_DEG: float = 22.0
TURN_EPS_DEG: float = 2.0
TURN_SETTLE_TIME: float = 0.08
TURN_TIMEOUT_SEC: float = 4.0

TURN_RIGHT_ANGLE_90: float = -85.0
TURN_RIGHT_ANGLE_180: float = -170.0

MAX_SENSOR_ATTEMPTS: int = 6

LOG_PREFIX = "[Task2]"

DT_SEC: float = 0.032


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp ``value`` to the closed interval [lo, hi]."""
    return max(lo, min(hi, value))


def sat_rpm(command_rpm: float) -> float:
    """Apply saturation and minimum-effort behaviour to a wheel command."""
    clipped = clamp(command_rpm, -MAX_RPM, MAX_RPM)
    if abs(clipped) < 1e-6:
        return 0.0
    if abs(clipped) < MIN_EFFORT_RPM:
        return math.copysign(MIN_EFFORT_RPM, clipped)
    return clipped


def log_status(stage: str, message: str) -> None:
    """Uniform logging helper."""
    print(f"{LOG_PREFIX}[{stage}] {message}")


def normalize_deg(angle_deg: float) -> float:
    """Wrap an absolute heading to [0, 360)."""
    return angle_deg % 360.0


def smallest_angle_deg(angle_deg: float) -> float:
    """Return the signed shortest rotation in degrees."""
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped <= -180.0:
        wrapped += 360.0
    return wrapped


def window_min(range_image: Iterable[float], start_idx: int, end_idx: int) -> float:
    """
    Return the minimum distance inside a lidar window, rejecting invalid samples.
    """
    try:
        scan = list(range_image)
    except TypeError:
        return MAX_RANGE

    total = len(scan)
    if total == 0:
        return MAX_RANGE

    samples: List[float] = []
    idx = start_idx
    while True:
        reading = scan[idx % total]

        if reading is None or reading <= 0.0:
            if idx == end_idx:
                break
            idx += 1
            continue

        reading_m = float(reading)
        if reading_m > MAX_RANGE * 2.0:
            reading_m /= 1000.0

        samples.append(min(reading_m, MAX_RANGE))
        if idx == end_idx:
            break
        idx += 1

    if not samples:
        return MAX_RANGE
    return min(samples)


def get_probe_distances(range_image: Iterable[float]) -> Tuple[float, float, float]:
    """Extract front, left, and right wall distances from the scan."""
    front = window_min(range_image, *FRONT_WIN)
    left = window_min(range_image, *LEFT_WIN)
    right = window_min(range_image, *RIGHT_WIN)
    return front, left, right


def poll_distances(bot: HamBot, attempts: int = MAX_SENSOR_ATTEMPTS) -> Optional[Tuple[float, float, float]]:
    """
    Attempt to fetch a fresh set of wall distances, retrying if all probes read MAX_RANGE.

    Returns None when no valid measurements arrive within ``attempts`` retries.
    """
    for attempt in range(1, attempts + 1):
        ranges = get_range_image(bot)
        front, left, right = get_probe_distances(ranges)
        if any(dist < MAX_RANGE for dist in (front, left, right)):
            return front, left, right

        log_status("LIDAR", f"No valid scan (attempt {attempt}/{attempts}); waiting...")
        if supervisor_step(bot, DT_SEC) == -1:
            break
    return None


def supervisor_step(bot: HamBot, dt: float) -> int:
    """Advance the simulator or sleep for hardware for ``dt`` seconds."""
    if hasattr(bot, "experiment_supervisor"):
        return bot.experiment_supervisor.step(dt)  # type: ignore[attr-defined]

    time.sleep(dt)
    return 0


def get_range_image(bot: HamBot) -> Iterable[float]:
    """Fetch the latest lidar scan."""
    if hasattr(bot, "get_lidar_range_image"):
        return bot.get_lidar_range_image()  # type: ignore[attr-defined]
    return bot.get_range_image()


def get_heading_deg(bot: HamBot) -> float:
    """Return the robot heading in degrees."""
    if hasattr(bot, "get_compass_reading"):
        return bot.get_compass_reading()  # type: ignore[attr-defined]
    return bot.get_heading()


def set_wheel_rpms(bot: HamBot, left_rpm: float, right_rpm: float) -> None:
    """Send wheel RPMs using whichever motor interface is available."""
    if hasattr(bot, "set_left_motor_velocity"):
        bot.set_left_motor_velocity(left_rpm)  # type: ignore[attr-defined]
        bot.set_right_motor_velocity(right_rpm)  # type: ignore[attr-defined]
        return

    if hasattr(bot, "set_left_motor_speed"):
        bot.set_left_motor_speed(left_rpm)  # type: ignore[attr-defined]
        bot.set_right_motor_speed(right_rpm)  # type: ignore[attr-defined]
        return

    raise AttributeError("HamBot instance lacks motor velocity or speed controls.")


def stop_robot(bot: HamBot) -> None:
    """Stop the robot regardless of interface differences."""
    if hasattr(bot, "stop"):
        try:
            bot.stop()  # type: ignore[attr-defined]
            return
        except Exception:
            pass

    if hasattr(bot, "stop_motors"):
        bot.stop_motors()  # type: ignore[attr-defined]
        return

    try:
        set_wheel_rpms(bot, 0.0, 0.0)
    except AttributeError:
        pass


def compute_steering(error_m: float) -> float:
    """
    Convert left distance error into a steering term.

    Positive error means we are too close to the wall, so steer to the right.
    """
    if abs(error_m) < SIDE_DEADBAND_M:
        return 0.0
    steer = STEER_KP * error_m
    return clamp(steer, -MAX_STEER_RPM, MAX_STEER_RPM)


def mix_wheel_commands(forward_rpm: float, steer_rpm: float) -> Tuple[float, float]:
    """Blend forward motion and steering into individual wheel RPMs."""
    left_cmd = sat_rpm(forward_rpm + steer_rpm)
    right_cmd = sat_rpm(forward_rpm - steer_rpm)
    return left_cmd, right_cmd


def perform_turn(bot: HamBot, angle_deg: float, context: str = "TURN") -> None:
    """Rotate the robot in place by ``angle_deg`` degrees (positive turns left)."""
    if abs(angle_deg) < 1e-3:
        return

    direction = "left" if angle_deg > 0 else "right"
    log_status(context, f"Begin in-place turn {direction} {abs(angle_deg):.0f}°")

    current = normalize_deg(get_heading_deg(bot))
    target = normalize_deg(current + angle_deg)
    desired_sign = 1.0 if angle_deg >= 0.0 else -1.0
    start_time = time.time()
    settle_start: Optional[float] = None
    final_error = float("nan")

    while True:
        heading = normalize_deg(get_heading_deg(bot))
        raw_error = smallest_angle_deg(target - heading)
        if abs(raw_error) > 179.5:
            error = desired_sign * abs(raw_error)
        else:
            error = raw_error

        abs_error = abs(error)
        final_error = error

        if abs_error <= TURN_EPS_DEG:
            if settle_start is None:
                settle_start = time.time()
            elif time.time() - settle_start >= TURN_SETTLE_TIME:
                break
        else:
            settle_start = None

        max_rpm = TURN_SLOW_RPM if abs_error < TURN_SLOW_BAND_DEG else TURN_FAST_RPM
        omega = clamp(TURN_KP * error, -max_rpm, max_rpm)
        if abs(omega) < MIN_EFFORT_RPM and abs_error > TURN_EPS_DEG:
            omega = math.copysign(MIN_EFFORT_RPM, error)

        left_cmd = sat_rpm(-omega)
        right_cmd = sat_rpm(omega)
        set_wheel_rpms(bot, left_cmd, right_cmd)

        if supervisor_step(bot, DT_SEC) == -1:
            log_status(context, "Supervisor requested shutdown during turn")
            break

        if time.time() - start_time >= TURN_TIMEOUT_SEC:
            log_status(context, f"Timeout while turning (remaining error {error:+.1f}°)")
            break

    set_wheel_rpms(bot, 0.0, 0.0)
    supervisor_step(bot, DT_SEC)
    log_status(context, f"Turn complete; residual error {final_error:+.1f}°")


def main() -> None:
    """Simple left-wall following loop."""
    bot = HamBot()
    last_debug = 0.0

    log_status("INIT", "Following left wall")

    # Pre-fill lidar values to let the sensor settle before control starts.
    warm_start_time = time.time()
    log_status("INIT", "Warming up lidar stream")
    while time.time() - warm_start_time < 1.0:
        _ = get_range_image(bot)
        supervisor_step(bot, DT_SEC)
    log_status("INIT", "Lidar ready")

    last_valid: Optional[Tuple[float, float, float]] = poll_distances(bot)
    if last_valid is None:
        log_status("LIDAR", "No valid scan after warm-up; waiting for data before moving")
    else:
        log_status(
            "INIT",
            f"Initial ranges front={last_valid[0]:0.2f}m left={last_valid[1]:0.2f}m right={last_valid[2]:0.2f}m",
        )

    try:
        while supervisor_step(bot, DT_SEC) != -1:
            distances = poll_distances(bot)
            if distances is None:
                if last_valid is None:
                    log_status("LIDAR", "Sensor still unavailable; holding position")
                    set_wheel_rpms(bot, 0.0, 0.0)
                    continue
                log_status("LIDAR", "Using last known distances while waiting for updates")
                front_dist, left_dist, right_dist = last_valid
            else:
                front_dist, left_dist, right_dist = distances
                last_valid = distances

            if front_dist < FRONT_BLOCK_M:
                turn_angle = (
                    TURN_RIGHT_ANGLE_90 if right_dist > RIGHT_CLEAR_M else TURN_RIGHT_ANGLE_180
                )
                turn_label = (
                    f"TURN_RIGHT_{abs(TURN_RIGHT_ANGLE_90):.0f}"
                    if turn_angle == TURN_RIGHT_ANGLE_90
                    else f"TURN_RIGHT_{abs(TURN_RIGHT_ANGLE_180):.0f}"
                )
                log_status("BLOCKED", f"Front={front_dist:0.2f}m Right={right_dist:0.2f}m -> {turn_label}")
                perform_turn(bot, turn_angle, context=turn_label)

                # Step once more to let lidar publish a fresh scan before resuming.
                refreshed = poll_distances(bot)
                if refreshed is None:
                    log_status("LIDAR", "Turn complete but still waiting for fresh scan")
                    set_wheel_rpms(bot, 0.0, 0.0)
                    last_valid = None
                    last_debug = time.time()
                    continue

                front_dist, left_dist, right_dist = refreshed
                last_valid = refreshed
                log_status(
                    "FOLLOW",
                    (
                        "Resuming after turn with "
                        f"front={front_dist:0.2f}m left={left_dist:0.2f}m right={right_dist:0.2f}m"
                    ),
                )
                last_debug = time.time()
                continue

            error = SIDE_TARGET_M - left_dist
            steer = compute_steering(error)

            cmd_left, cmd_right = mix_wheel_commands(BASE_FWD_RPM, steer)
            set_wheel_rpms(bot, cmd_left, cmd_right)

            now = time.time()
            if now - last_debug >= 0.2:
                log_status(
                    "FOLLOW",
                    (
                        f"front={front_dist:0.2f}m left={left_dist:0.2f}m "
                        f"right={right_dist:0.2f}m target={SIDE_TARGET_M:0.2f}m "
                        f"error={error:+0.2f}m steer={steer:+0.2f} "
                        f"rpmL={cmd_left:0.1f} rpmR={cmd_right:0.1f}"
                    ),
                )
                last_debug = now

    except KeyboardInterrupt:
        print("\n[Task2] Interrupted by user.")
    finally:
        try:
            stop_robot(bot)
        except Exception:
            pass


if __name__ == "__main__":
    main()
