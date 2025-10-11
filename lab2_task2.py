"""
HamBot Lab 2 Task 2 — real robot wall following.

The controller adapts the FAIRIS-Lite simulator logic from
``Lab2_Task2fairis lite.py`` to the physical HamBot platform.
It reads the RPLidar, keeps a safe forward buffer, and steers to
maintain a target distance from either wall while slowing down or
turning when the path ahead is blocked.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

# Allow direct execution from the repository root without modifying PYTHONPATH.
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.robot_systems.robot import HamBot  # noqa: E402


# ---------------------------------------------------------------------------
# Lidar geometry and controller defaults
# ---------------------------------------------------------------------------
FRONT_ARC: Tuple[int, int] = (165, 195)
LEFT_ARC: Tuple[int, int] = (60, 120)
RIGHT_ARC: Tuple[int, int] = (240, 300)

MAX_LIDAR_METERS: float = 4.0

FORWARD_SETPOINT_M: float = 0.30
SIDE_SETPOINT_M: float = 0.40

FORWARD_KP: float = 60.0          # rpm per meter of forward error
SIDE_KP: float = 30.0             # rpm per meter of lateral error
MAX_CMD_RPM: float = 32.0         # safe ceiling for HamBot drivetrain
MIN_CMD_RPM: float = 6.0          # overcome drivetrain static friction
LOOP_HZ_DEFAULT: float = 12.0
LOG_PERIOD_SEC: float = 0.5

TURN_MIN_RPM: float = 10.0
TURN_SPEED_RPM: float = 24.0
TURN_KP: float = 0.6
TURN_TOLERANCE_DEG: float = 4.0
TURN_TIMEOUT_SEC: float = 4.5

TURN_FRONT_TRIGGER_M: float = 0.60
TURN_SIDE_CONFIRM_M: float = 0.60
TURN_COOLDOWN_SEC: float = 2.0


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------
def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp ``value`` to the closed interval [lo, hi]."""
    return max(lo, min(hi, value))


def normalize_heading(deg: float) -> float:
    """Wrap an absolute heading to [0, 360)."""
    return deg % 360.0


def shortest_angle_deg(target: float, current: float) -> float:
    """Return the signed difference (target - current) in (-180, 180]."""
    diff = (target - current + 180.0) % 360.0 - 180.0
    if diff <= -180.0:
        diff += 360.0
    return diff


def degrees_inclusive_range(start: int, end: int, total: int) -> Iterable[int]:
    """
    Yield indices from ``start`` to ``end`` (inclusive) while wrapping.

    The helper covers lidar windows that might wrap across 0°/360°.
    """
    idx = start % total
    end = end % total
    yield idx
    while idx != end:
        idx = (idx + 1) % total
        yield idx


def window_min_distance_m(range_image: Iterable[float],
                          window: Tuple[int, int]) -> Tuple[Optional[float], int]:
    """
    Return the minimum distance (meters) inside a lidar arc along with the
    number of valid samples used. Invalid readings (≤0) are skipped.
    """
    try:
        scan = list(range_image)
    except TypeError:
        return None, 0

    if not scan:
        return None, 0

    total = len(scan)
    start, end = window

    readings: list[float] = []

    # Clamp sample size so a misconfigured window cannot overrun.
    limit = total * 2
    processed = 0

    for idx in degrees_inclusive_range(start, end, total):
        processed += 1
        if processed > limit:
            break

        reading = scan[idx]
        if reading is None or reading <= 0:
            continue

        # Convert millimetres to metres if magnitudes indicate so.
        reading_m = reading / 1000.0 if reading > 10.0 else float(reading)
        reading_m = min(reading_m, MAX_LIDAR_METERS)
        readings.append(reading_m)

    if not readings:
        return None, 0

    return min(readings), len(readings)


def saturation_rpm(command_rpm: float) -> float:
    """
    Apply motor command saturation with a minimum effort band.

    Commands inside ±MIN_CMD_RPM are set to the minimum necessary to keep the
    drivetrain moving unless the requested command is effectively zero.
    """
    limited = clamp(command_rpm, -MAX_CMD_RPM, MAX_CMD_RPM)
    if abs(limited) < 1e-3:
        return 0.0
    if abs(limited) < MIN_CMD_RPM:
        limited = math.copysign(MIN_CMD_RPM, limited)
    return limited


# ---------------------------------------------------------------------------
# Wall-following controller
# ---------------------------------------------------------------------------
@dataclass
class ControllerOutput:
    left_rpm: float
    right_rpm: float
    forward_component: float
    steer_component: float
    front_m: Optional[float]
    left_m: Optional[float]
    right_m: Optional[float]
    side_m: Optional[float]


def compute_forward_velocity(front_m: Optional[float],
                             maintain_m: float) -> float:
    """
    Convert the forward distance error into a motor command.

    When the lidar is unavailable the robot keeps a gentle crawl so the
    operator can recover or abort manually.
    """
    if front_m is None:
        return MIN_CMD_RPM

    error = front_m - maintain_m
    command = FORWARD_KP * error
    return saturation_rpm(command)


def wall_follow_step(scan: Iterable[float],
                     follow_side: str,
                     side_setpoint_m: float,
                     forward_setpoint_m: float) -> ControllerOutput:
    """Compute the left/right RPM commands needed for the next control step."""
    front_m, front_samples = window_min_distance_m(scan, FRONT_ARC)
    left_m, left_samples = window_min_distance_m(scan, LEFT_ARC)
    right_m, right_samples = window_min_distance_m(scan, RIGHT_ARC)

    forward_rpm = compute_forward_velocity(front_m, forward_setpoint_m)

    if follow_side.upper() == "R":
        side_m = right_m if right_samples else None
        error = (side_setpoint_m - side_m) if side_m is not None else 0.0
        steer = SIDE_KP * error
        left_cmd = saturation_rpm(forward_rpm + steer)
        right_cmd = saturation_rpm(forward_rpm - steer)
    else:
        side_m = left_m if left_samples else None
        error = (side_setpoint_m - side_m) if side_m is not None else 0.0
        steer = SIDE_KP * error
        left_cmd = saturation_rpm(forward_rpm - steer)
        right_cmd = saturation_rpm(forward_rpm + steer)

    return ControllerOutput(
        left_rpm=left_cmd,
        right_rpm=right_cmd,
        forward_component=forward_rpm,
        steer_component=steer,
        front_m=front_m,
        left_m=left_m,
        right_m=right_m,
        side_m=side_m,
    )


# ---------------------------------------------------------------------------
# Heading-based turns (quarter turns for obstacle avoidance)
# ---------------------------------------------------------------------------
def turn_in_place(bot: HamBot, angle_deg: float) -> None:
    """
    Execute an in-place rotation using the IMU for feedback.

    Falls back to a timed open-loop turn if the IMU heading is unavailable.
    """
    start_heading = bot.get_heading(blocking=True)

    if start_heading is None:
        # Fallback: estimate turn duration assuming 90° ≈ 1.0 s at TURN_SPEED_RPM.
        duration = TURN_TIMEOUT_SEC * (abs(angle_deg) / 360.0)
        rpm = min(TURN_SPEED_RPM, MAX_CMD_RPM)
        rpm = max(rpm, TURN_MIN_RPM)
        left_cmd = -rpm if angle_deg > 0 else rpm
        right_cmd = rpm if angle_deg > 0 else -rpm
        bot.set_left_motor_speed(left_cmd)
        bot.set_right_motor_speed(right_cmd)
        time.sleep(duration)
        bot.stop_motors()
        time.sleep(0.2)
        return

    target = normalize_heading(start_heading + angle_deg)
    deadline = time.time() + TURN_TIMEOUT_SEC

    while time.time() < deadline:
        heading = bot.get_heading()
        if heading is None:
            break

        error = shortest_angle_deg(target, heading)
        if abs(error) <= TURN_TOLERANCE_DEG:
            break

        turn_rpm = clamp(TURN_KP * error, -TURN_SPEED_RPM, TURN_SPEED_RPM)
        if abs(turn_rpm) < TURN_MIN_RPM:
            turn_rpm = math.copysign(TURN_MIN_RPM, turn_rpm if turn_rpm else error)

        left_cmd = -abs(turn_rpm) if turn_rpm > 0 else abs(turn_rpm)
        right_cmd = abs(turn_rpm) if turn_rpm > 0 else -abs(turn_rpm)

        bot.set_left_motor_speed(left_cmd)
        bot.set_right_motor_speed(right_cmd)
        time.sleep(0.05)

    bot.stop_motors()
    time.sleep(0.2)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HamBot wall following controller.")
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default="right",
        help="Which wall to follow (default: right).",
    )
    parser.add_argument(
        "--loop-hz",
        type=float,
        default=LOOP_HZ_DEFAULT,
        help=f"Main control loop frequency (default: {LOOP_HZ_DEFAULT} Hz).",
    )
    parser.add_argument(
        "--forward-setpoint",
        type=float,
        default=FORWARD_SETPOINT_M,
        help="Desired clearance to obstacles ahead in meters.",
    )
    parser.add_argument(
        "--side-setpoint",
        type=float,
        default=SIDE_SETPOINT_M,
        help="Desired distance to the tracked wall in meters.",
    )
    parser.add_argument(
        "--front-turn",
        type=float,
        default=TURN_FRONT_TRIGGER_M,
        help="Trigger a 90° turn when the forward distance falls below this value.",
    )
    parser.add_argument(
        "--side-confirm",
        type=float,
        default=TURN_SIDE_CONFIRM_M,
        help="For left-wall mode, require the left distance to also be below this value before turning.",
    )
    parser.add_argument(
        "--turn-speed",
        type=float,
        default=TURN_SPEED_RPM,
        help="In-place turn speed in RPM.",
    )
    parser.add_argument(
        "--log-period",
        type=float,
        default=LOG_PERIOD_SEC,
        help="Seconds between console telemetry updates.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    follow_side = "R" if args.side.lower().startswith("r") else "L"
    loop_dt = 1.0 / max(1e-3, args.loop_hz)

    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    # Ensure the camera attribute exists even when disabled to keep disconnect safe.
    if not hasattr(bot, "camera"):
        bot.camera = None

    last_log = time.time()
    last_turn = 0.0

    print(f"Starting wall follower on the {args.side.upper()} wall")
    print(f"  forward setpoint : {args.forward_setpoint:.2f} m")
    print(f"  side setpoint    : {args.side_setpoint:.2f} m")
    print(f"  turn trigger     : front < {args.front_turn:.2f} m")

    try:
        while True:
            loop_start = time.time()
            scan = bot.get_range_image()
            if not scan or scan == -1:
                bot.stop_motors()
                print("No lidar data; waiting for sensor...")
                time.sleep(0.25)
                continue

            output = wall_follow_step(
                scan=scan,
                follow_side=follow_side,
                side_setpoint_m=args.side_setpoint,
                forward_setpoint_m=args.forward_setpoint,
            )

            now = time.time()

            # Corner handling: rotate when the forward path is blocked.
            should_turn = False
            if output.front_m is not None and output.front_m < args.front_turn:
                if follow_side == "R":
                    should_turn = True
                else:
                    if output.left_m is not None and output.left_m < args.side_confirm:
                        should_turn = True

            if should_turn and (now - last_turn) > TURN_COOLDOWN_SEC:
                bot.stop_motors()
                time.sleep(0.1)
                angle = -90.0 if follow_side == "R" else 90.0
                print(f"Front blocked ({output.front_m:.2f} m). Executing {angle:+.0f}° turn.")
                turn_in_place(bot, angle)
                last_turn = time.time()
                continue

            bot.set_left_motor_speed(output.left_rpm)
            bot.set_right_motor_speed(output.right_rpm)

            if now - last_log >= args.log_period:
                def fmt(value: Optional[float]) -> str:
                    return f"{value:.2f}" if value is not None else "nan"

                print(
                    f"[{time.strftime('%H:%M:%S')}] "
                    f"front={fmt(output.front_m)} m, "
                    f"left={fmt(output.left_m)} m, "
                    f"right={fmt(output.right_m)} m | "
                    f"cmdL={output.left_rpm:+.1f} rpm, "
                    f"cmdR={output.right_rpm:+.1f} rpm"
                )
                last_log = now

            loop_elapsed = time.time() - loop_start
            sleep_time = loop_dt - loop_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Stopping HamBot.")
    finally:
        bot.stop_motors()
        try:
            bot.disconnect_robot()
        except AttributeError:
            # Graceful shutdown if disconnect_robot touches missing attributes.
            if hasattr(bot, "lidar") and bot.lidar is not None:
                bot.lidar.stop_lidar()
            if hasattr(bot, "imu"):
                bot.imu.stop()


if __name__ == "__main__":
    main()

