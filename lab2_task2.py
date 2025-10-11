"""
HamBot Lab 2 Task 2 — left-wall alignment controller.

This script implements the straight-line portion of the wall-following
behaviour using the geometry outlined in the project docs:

* maintain a fixed perpendicular distance to the left wall,
* keep the chassis parallel to the wall using lidar beams at 90° and 60°,
* gently trim the heading with a PD controller while driving forward.

Distances are sourced from the HamBot lidar (indices follow docs/lidar.md),
and motor commands are converted into RPM for the Build HAT motors described
in README.md and docs/motors.md.
"""

from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple, List

# Allow running the controller directly from the repository root without
# modifying PYTHONPATH externally.
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.robot_systems.robot import HamBot  # noqa: E402


# ---------------------------------------------------------------------------
# Robot geometry and controller gains
# ---------------------------------------------------------------------------
WHEEL_RADIUS_M: float = 0.045          # README.md
AXLE_LENGTH_M: float = 0.184           # README.md
MAX_RPM: float = 75.0                  # Build HAT limit

BASE_SPEED_M_S: float = 0.20           # nominal forward speed
SLOW_SPEED_M_S: float = 0.08           # crawl when obstacle ahead
DISTANCE_SETPOINT_M: float = 0.40      # desired perpendicular clearance to wall

K_DISTANCE: float = 2.2                # gain on perpendicular distance error
K_ANGLE: float = 3.0                   # gain on wall-relative heading error

FRONT_STOP_M: float = 0.25             # emergency brake distance
FRONT_SLOW_M: float = 0.45             # start tapering speed ahead of obstacles

LOOP_HZ: float = 15.0
LOG_PERIOD_S: float = 0.5

SIDE_ANGLE_DEG: int = 90
FRONT_LEFT_ANGLE_DEG: int = 60
FRONT_SECTOR: Tuple[int, int] = (160, 200)
ANGLE_WINDOW: int = 2                  # smoothing window for single-beam reads


# ---------------------------------------------------------------------------
# Helper data structures
# ---------------------------------------------------------------------------
@dataclass
class ControllerOutput:
    left_rpm: float
    right_rpm: float
    linear_m_s: float
    angular_rad_s: float
    distance_perp: Optional[float]
    angle_rad: Optional[float]
    d_left: Optional[float]
    d_front_left: Optional[float]
    d_front: Optional[float]


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------
def rad_s_to_rpm(rad_s: float) -> float:
    """Convert wheel angular velocity from rad/s to RPM."""
    return rad_s * 60.0 / (2.0 * math.pi)


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to the closed interval [lo, hi]."""
    return max(lo, min(hi, value))


def saturate_wheels(left_rpm: float, right_rpm: float, limit: float = MAX_RPM) -> Tuple[float, float]:
    """Scale wheel RPMs so neither exceeds the hardware limit."""
    max_mag = max(abs(left_rpm), abs(right_rpm), 1e-6)
    if max_mag <= limit:
        return left_rpm, right_rpm
    scale = limit / max_mag
    return left_rpm * scale, right_rpm * scale


def mm_to_m(distance: float) -> float:
    """Convert millimetres to metres when readings are reported in mm ranges."""
    return distance / 1000.0 if distance > 10.0 else distance


def distance_at_angle(scan: List[float], angle_deg: int, window: int = 0) -> Optional[float]:
    """
    Return the average distance (metres) around a given lidar angle.

    Angles follow docs/lidar.md conventions (0° rear, 90° left, 180° front).
    Invalid readings (<= 0) are ignored; None is returned if no valid samples exist.
    """
    if not scan:
        return None

    total = len(scan)
    if total == 0:
        return None

    samples: List[float] = []
    for offset in range(-window, window + 1):
        idx = (angle_deg + offset) % total
        reading = scan[idx]
        if reading is None or reading <= 0:
            continue
        samples.append(mm_to_m(float(reading)))

    if not samples:
        return None

    return sum(samples) / len(samples)


def window_min_distance(scan: List[float], start_idx: int, end_idx: int) -> Optional[float]:
    """Return the minimum distance (metres) within an inclusive index window."""
    if not scan:
        return None

    total = len(scan)
    if total == 0:
        return None

    if end_idx < start_idx:
        end_idx += total

    values: List[float] = []
    for idx in range(start_idx, end_idx + 1):
        real_idx = idx % total
        reading = scan[real_idx]
        if reading is None or reading <= 0:
            continue
        values.append(mm_to_m(float(reading)))

    if not values:
        return None

    return min(values)


def compute_wall_geometry(d_left: float, d_front_left: float) -> Tuple[float, float]:
    """
    Compute the wall-relative orientation and perpendicular distance.

    Returns:
        angle_rad: signed angle between robot heading and wall normal (rad)
        distance_perp: perpendicular distance from robot to wall (m)
    """
    if d_front_left <= 1e-6:
        return 0.0, d_left

    cos60 = math.cos(math.radians(60.0))
    sin60 = math.sin(math.radians(60.0))

    numerator = d_front_left * cos60 - d_left
    denominator = d_front_left * sin60
    if abs(denominator) <= 1e-6:
        angle_rad = 0.0
    else:
        angle_rad = math.atan2(numerator, denominator)

    distance_perp = d_left * math.cos(angle_rad)
    return angle_rad, distance_perp


def diff_drive_to_rpm(linear_m_s: float, angular_rad_s: float) -> Tuple[float, float]:
    """Convert chassis linear/angular velocities to wheel RPM commands."""
    half_axle = AXLE_LENGTH_M / 2.0
    left_linear = linear_m_s - angular_rad_s * half_axle
    right_linear = linear_m_s + angular_rad_s * half_axle

    left_rad_s = left_linear / WHEEL_RADIUS_M
    right_rad_s = right_linear / WHEEL_RADIUS_M

    return rad_s_to_rpm(left_rad_s), rad_s_to_rpm(right_rad_s)


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------
class LeftWallAlignController:
    """Closed-loop controller that keeps HamBot parallel to a left wall."""

    def __init__(self,
                 distance_setpoint: float = DISTANCE_SETPOINT_M,
                 base_speed: float = BASE_SPEED_M_S,
                 k_distance: float = K_DISTANCE,
                 k_angle: float = K_ANGLE,
                 front_stop: float = FRONT_STOP_M,
                 front_slow: float = FRONT_SLOW_M,
                 slow_speed: float = SLOW_SPEED_M_S) -> None:
        self.distance_setpoint = distance_setpoint
        self.base_speed = base_speed
        self.k_distance = k_distance
        self.k_angle = k_angle
        self.front_stop = front_stop
        self.front_slow = max(front_stop, front_slow)
        self.slow_speed = slow_speed

    def step(self, scan: List[float]) -> ControllerOutput:
        """Compute motor commands from the latest lidar scan."""
        d_left = distance_at_angle(scan, SIDE_ANGLE_DEG, ANGLE_WINDOW)
        d_front_left = distance_at_angle(scan, FRONT_LEFT_ANGLE_DEG, ANGLE_WINDOW)
        d_front = window_min_distance(scan, FRONT_SECTOR[0], FRONT_SECTOR[1])

        if d_left is None or d_front_left is None:
            return ControllerOutput(
                left_rpm=0.0,
                right_rpm=0.0,
                linear_m_s=0.0,
                angular_rad_s=0.0,
                distance_perp=None,
                angle_rad=None,
                d_left=d_left,
                d_front_left=d_front_left,
                d_front=d_front,
            )

        angle_rad, distance_perp = compute_wall_geometry(d_left, d_front_left)
        e_distance = self.distance_setpoint - distance_perp
        e_angle = -angle_rad
        omega = self.k_distance * e_distance + self.k_angle * e_angle

        linear_speed = self.base_speed
        if d_front is not None:
            if d_front <= self.front_stop:
                linear_speed = 0.0
            elif d_front < self.front_slow:
                ratio = (d_front - self.front_stop) / (self.front_slow - self.front_stop)
                linear_speed = self.slow_speed + (self.base_speed - self.slow_speed) * clamp(ratio, 0.0, 1.0)

        left_rpm, right_rpm = diff_drive_to_rpm(linear_speed, omega)
        left_rpm, right_rpm = saturate_wheels(left_rpm, right_rpm)

        return ControllerOutput(
            left_rpm=left_rpm,
            right_rpm=right_rpm,
            linear_m_s=linear_speed,
            angular_rad_s=omega,
            distance_perp=distance_perp,
            angle_rad=angle_rad,
            d_left=d_left,
            d_front_left=d_front_left,
            d_front=d_front,
        )


# ---------------------------------------------------------------------------
# Runtime loop
# ---------------------------------------------------------------------------
def main() -> None:
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    if not hasattr(bot, "camera"):
        bot.camera = None  # keep disconnect_robot safe

    controller = LeftWallAlignController()
    loop_dt = 1.0 / max(LOOP_HZ, 1e-6)
    last_log = 0.0

    try:
        while True:
            start = time.time()
            scan = bot.get_range_image()
            if scan == -1 or scan is None:
                bot.stop_motors()
                print("Waiting for lidar data...")
                time.sleep(0.2)
                continue

            output = controller.step(list(scan))
            bot.set_left_motor_speed(output.left_rpm)
            bot.set_right_motor_speed(output.right_rpm)

            now = time.time()
            if now - last_log >= LOG_PERIOD_S:
                def fmt(value: Optional[float]) -> str:
                    return f"{value:.3f}" if value is not None else "nan"

                print(
                    f"[{time.strftime('%H:%M:%S')}] "
                    f"d_left={fmt(output.d_left)} m, "
                    f"d_front_left={fmt(output.d_front_left)} m, "
                    f"dist_perp={fmt(output.distance_perp)} m, "
                    f"angle={fmt(output.angle_rad)} rad, "
                    f"front={fmt(output.d_front)} m | "
                    f"v={output.linear_m_s:.2f} m/s, "
                    f"omega={output.angular_rad_s:.2f} rad/s, "
                    f"rpmL={output.left_rpm:+.1f}, "
                    f"rpmR={output.right_rpm:+.1f}"
                )
                last_log = now

            elapsed = time.time() - start
            sleep_time = loop_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Stopping HamBot.")
    finally:
        bot.stop_motors()
        try:
            bot.disconnect_robot()
        except AttributeError:
            if hasattr(bot, "lidar") and bot.lidar is not None:
                bot.lidar.stop_lidar()
            if hasattr(bot, "imu"):
                bot.imu.stop()


if __name__ == "__main__":
    main()

