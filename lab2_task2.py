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
from typing import Iterable, List, Tuple

# Allow running the controller directly from the repository root without
# modifying PYTHONPATH externally.
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.robot_systems.robot import HamBot  # noqa: E402 (import after sys.path)


# Lidar sampling window for the left wall.
LEFT_WIN: Tuple[int, int] = (90, 115)
MAX_RANGE: float = 4.0

# Simplified motion parameters.
SIDE_TARGET_M: float = 0.28
SIDE_DEADBAND_M: float = 0.01
BASE_FWD_RPM: float = 16.0
STEER_KP: float = 20.0
MAX_STEER_RPM: float = 12.0
MAX_RPM: float = 35.0
MIN_EFFORT_RPM: float = 6.0

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


def get_left_distance(range_image: Iterable[float]) -> float:
    """Extract the current left wall distance from the scan."""
    return window_min(range_image, *LEFT_WIN)


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


def main() -> None:
    """Simple left-wall following loop."""
    bot = HamBot()
    last_debug = 0.0

    print("[Task2] Following left wall")

    try:
        while supervisor_step(bot, DT_SEC) != -1:
            ranges = get_range_image(bot)
            left_dist = get_left_distance(ranges)

            error = SIDE_TARGET_M - left_dist
            steer = compute_steering(error)

            cmd_left, cmd_right = mix_wheel_commands(BASE_FWD_RPM, steer)
            set_wheel_rpms(bot, cmd_left, cmd_right)

            now = time.time()
            if now - last_debug >= 0.2:
                print(
                    f"[Task2] left={left_dist:0.2f}m "
                    f"target={SIDE_TARGET_M:0.2f}m error={error:+0.2f}m "
                    f"steer={steer:+0.2f} rpmL={cmd_left:0.1f} rpmR={cmd_right:0.1f}"
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
