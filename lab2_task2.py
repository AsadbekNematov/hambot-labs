"""
HamBot Lab 2 Task 2 controller adapted from the FAIRIS-Lite simulator script.

The implementation mirrors the structure of ``Lab2_Task2fairis lite.py`` while
swapping the Webots robot wrapper for the real HamBot platform. Distances are
derived from the HamBot lidar (mm → m) and motor commands are converted from
rad/s to RPM before being sent to the Build HAT motors.
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import Iterable, Optional, Tuple, List

# Allow running the controller directly from the repository root without
# modifying PYTHONPATH externally.
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.robot_systems.robot import HamBot  # noqa: E402

# ---------------------------------------------------------------------------
# Constants that reflect the simulator configuration
# ---------------------------------------------------------------------------
MAX_MOTOR_VELOCITY_RAD = 7.85  # ≈75 RPM, matches the HamBot README spec
RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * math.pi)

FRONT_INDEX = 180
FRONT_SPREAD = 5
LEFT_WINDOW: Tuple[int, int] = (80, 100)
RIGHT_WINDOW: Tuple[int, int] = (260, 280)

# Global HamBot instance (populated in main)
robot: Optional[HamBot] = None


def rad_to_rpm(rad_per_sec: float) -> float:
    """Convert a rad/s command to RPM for the Build HAT motors."""
    return rad_per_sec * RAD_PER_SEC_TO_RPM


def get_scan() -> Optional[List[float]]:
    """Fetch the latest lidar scan as a list of distances in millimetres."""
    if robot is None:
        return None

    scan = robot.get_range_image()
    if scan == -1 or scan is None:
        return None
    return list(scan)


def window_min_distance_m(scan: Iterable[float], start_idx: int, end_idx: int) -> Optional[float]:
    """
    Return the minimum distance (metres) inside the specified inclusive index range.

    Any invalid readings (≤0) are ignored. The window wraps around the scan if the
    indices exceed 0..len(scan)-1.
    """
    if not scan:
        return None

    measurements: List[float] = []
    scan_list = list(scan)
    total = len(scan_list)
    if total == 0:
        return None

    if end_idx < start_idx:
        end_idx += total

    for idx in range(start_idx, end_idx + 1):
        actual_idx = idx % total
        reading = scan_list[actual_idx]
        if reading is None or reading <= 0:
            continue
        # Convert mm → m when magnitudes indicate millimetres.
        distance_m = reading / 1000.0 if reading > 10.0 else float(reading)
        measurements.append(distance_m)

    if not measurements:
        return None

    return min(measurements)


def saturation_func(signal_rad: float) -> float:
    """Match the simulator saturation behaviour (clamp ±max/2 when exceeding bounds)."""
    if signal_rad >= MAX_MOTOR_VELOCITY_RAD:
        signal_rad = MAX_MOTOR_VELOCITY_RAD / 2.0
    elif signal_rad <= -MAX_MOTOR_VELOCITY_RAD:
        signal_rad = -MAX_MOTOR_VELOCITY_RAD / 2.0
    return signal_rad


def proportional_gain(scan: Iterable[float],
                      d_maintain: float = 0.30,
                      kp: float = 5.0) -> Tuple[float, Optional[float]]:
    """
    Compute the forward velocity command based on front distance error.

    Returns:
        Tuple of (command_rad_per_sec, front_distance_m or None)
    """
    d_front = window_min_distance_m(scan, FRONT_INDEX - FRONT_SPREAD, FRONT_INDEX + FRONT_SPREAD)

    print("---------------------------------")
    print(f"d_front: {d_front}")

    if d_front is None:
        print("No valid front samples; holding position.")
        return 0.0, None

    dist_error_front = d_front - d_maintain
    print(f"error_front: {dist_error_front}")
    v_front = kp * dist_error_front

    return v_front, d_front


def wall_following_pid(wall_to_follow: str,
                       scan: Iterable[float],
                       d_mid: float = 0.40,
                       k_p: float = 5.0) -> Tuple[float, float, Optional[float], Optional[float], Optional[float]]:
    """
    Perform wall following and compute individual wheel velocities (rad/s).

    Returns:
        (v_left_rad, v_right_rad, front_m, left_m, right_m)
    """
    forward_velocity, d_front = proportional_gain(scan)

    # get sensor readings to detect min distance to side walls
    d_left = window_min_distance_m(scan, LEFT_WINDOW[0], LEFT_WINDOW[1])
    print(f"d_left {d_left}")
    d_right = window_min_distance_m(scan, RIGHT_WINDOW[0], RIGHT_WINDOW[1])
    print(f"d_right: {d_right}")

    if wall_to_follow.upper() == "R":
        error = (d_mid - d_right) if d_right is not None else 0.0
        if d_right is not None and d_right < d_mid:
            v_left = saturation_func(forward_velocity - abs(k_p * error))
            v_right = saturation_func(forward_velocity)
        elif d_right is not None and d_right > d_mid:
            v_left = saturation_func(forward_velocity)
            v_right = saturation_func(forward_velocity - abs(k_p * error))
        else:
            v_left = saturation_func(forward_velocity)
            v_right = saturation_func(forward_velocity)
    else:
        error = (d_mid - d_left) if d_left is not None else 0.0
        if d_left is not None and d_left < d_mid:
            v_right = saturation_func(forward_velocity - abs(k_p * error))
            v_left = saturation_func(forward_velocity)
        elif d_left is not None and d_left > d_mid:
            v_right = saturation_func(forward_velocity)
            v_left = saturation_func(forward_velocity - abs(k_p * error))
        else:
            v_left = saturation_func(forward_velocity)
            v_right = saturation_func(forward_velocity)

    return v_left, v_right, d_front, d_left, d_right


def shortest_angle_deg(target: float, current: float) -> float:
    """Return the signed difference (target - current) in (-180, 180]."""
    diff = (target - current + 180.0) % 360.0 - 180.0
    if diff <= -180.0:
        diff += 360.0
    return diff


def rotate(bot: HamBot, angular_speed_rad: float, angle_deg: float, timeout: float = 6.0) -> None:
    """
    Rotate the HamBot in place, emulating ``robot.rotate`` from the simulator.

    Uses the IMU heading when available, otherwise falls back to a timed turn.
    """
    target_rpm = min(rad_to_rpm(abs(angular_speed_rad)), 40.0)
    target_rpm = max(target_rpm, 10.0)  # ensure enough torque to move

    start_heading = bot.get_heading(blocking=True)
    if start_heading is None:
        # Fallback: open-loop timing assuming ~0.9 s per 90° at target_rpm
        duration = abs(angle_deg) / 90.0 * 0.9
        command = target_rpm if angle_deg > 0 else -target_rpm
        bot.set_left_motor_speed(command)
        bot.set_right_motor_speed(command)
        time.sleep(duration)
        bot.stop_motors()
        time.sleep(0.2)
        return

    target_heading = (start_heading + angle_deg) % 360.0
    deadline = time.time() + timeout

    while time.time() < deadline:
        heading = bot.get_heading()
        if heading is None:
            break

        error = shortest_angle_deg(target_heading, heading)
        if abs(error) <= 3.0:
            break

        command = target_rpm if error > 0 else -target_rpm
        bot.set_left_motor_speed(command)
        bot.set_right_motor_speed(command)
        time.sleep(0.05)

    bot.stop_motors()
    time.sleep(0.2)


def main() -> None:
    global robot

    robot = HamBot(lidar_enabled=True, camera_enabled=False)
    if not hasattr(robot, "camera"):
        robot.camera = None  # keep disconnect_robot happy

    wall = "R"  # Follow the right wall by default, matching the simulator script

    try:
        while True:
            scan = get_scan()
            if scan is None:
                robot.stop_motors()
                print("Waiting for lidar data...")
                time.sleep(0.2)
                continue

            left_speed_rad, right_speed_rad, dist_front, dist_left, dist_right = wall_following_pid(
                wall_to_follow=wall,
                scan=scan,
            )

            left_speed_rpm = rad_to_rpm(left_speed_rad)
            right_speed_rpm = rad_to_rpm(right_speed_rad)

            print(f"left speed: {left_speed_rpm}")
            robot.set_left_motor_speed(left_speed_rpm)
            print(f" right speed: {right_speed_rpm}")
            robot.set_right_motor_speed(right_speed_rpm)

            # Decide on rotations using the same thresholds as the simulator script
            if dist_front is not None and dist_front < 0.7 and wall == "R":
                rotate(robot, MAX_MOTOR_VELOCITY_RAD, -90)

            if (
                wall == "L"
                and dist_front is not None
                and dist_left is not None
                and dist_front < 0.6
                and dist_left < 0.6
            ):
                rotate(robot, MAX_MOTOR_VELOCITY_RAD, 90)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[Task2] Interrupted by user.")
    finally:
        if robot is not None:
            robot.stop_motors()
            try:
                robot.disconnect_robot()
            except AttributeError:
                if hasattr(robot, "lidar") and robot.lidar is not None:
                    robot.lidar.stop_lidar()
                if hasattr(robot, "imu"):
                    robot.imu.stop()


if __name__ == "__main__":
    main()

