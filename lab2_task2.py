"""
HamBot Lab 2 Task 2 controller implementing left-wall following behaviour.

The script keeps everything in a single file, exposes the full wall-following
logic, and remains easy to extend with future runtime mode switching. All
control remains fully compatible with the physical HamBot API.
"""

from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Iterable, List, Optional, Tuple

# Allow running the controller directly from the repository root without
# modifying PYTHONPATH externally.
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.robot_systems.robot import HamBot  # noqa: E402  (import after sys.path)


# ---------------------------------------------------------------------------
# Lidar windows and geometry constants
# ---------------------------------------------------------------------------
FRONT_WIN: Tuple[int, int] = (175, 185)
LEFT_WIN: Tuple[int, int] = (90, 115)
LEFT_FWD_WIN: Tuple[int, int] = (135, 150)
RIGHT_WIN: Tuple[int, int] = (245, 270)
RIGHT_FWD_WIN: Tuple[int, int] = (210, 225)
MAX_RANGE: float = 4.0


# ---------------------------------------------------------------------------
# Control targets and motion limits
# ---------------------------------------------------------------------------
SIDE_TARGET_M: float = 0.28
BASE_FWD_RPM: float = 16.0
MAX_RPM: float = 35.0
MIN_EFFORT_RPM: float = 6.0


# ---------------------------------------------------------------------------
# Detection thresholds
# ---------------------------------------------------------------------------
FRONT_TH: float = 0.40
SIDE_TH: float = 0.26
LOOKAHEAD_TH: float = 0.70
GAP: float = 0.12


# ---------------------------------------------------------------------------
# Gains, smoothing, and turn controller parameters
# ---------------------------------------------------------------------------
KP: float = 3.0
KI: float = 0.10
KD: float = 0.50
I_CLAMP: float = 0.5
OMEGA_ALPHA: float = 0.2

ROT_KP: float = 0.5
ROT_EPS_DEG: float = 4.0

DT_SEC: float = 0.032


class RobotState(Enum):
    """Discrete controller modes used by the high-level state machine."""

    FOLLOW = auto()
    TURN_90 = auto()
    TURN_180 = auto()


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp ``value`` to the closed interval [lo, hi]."""
    return max(lo, min(hi, value))


def sat_rpm(command_rpm: float) -> float:
    """
    Apply motor saturation and minimum-effort enforcement to an RPM command.

    The drivetrain needs a small bias (``MIN_EFFORT_RPM``) to overcome static
    friction. Commands close to zero are zeroed so the robot can stop cleanly.
    """
    clipped = clamp(command_rpm, -MAX_RPM, MAX_RPM)
    if abs(clipped) < 1e-6:
        return 0.0
    if abs(clipped) < MIN_EFFORT_RPM:
        return math.copysign(MIN_EFFORT_RPM, clipped)
    return clipped


def normalize_deg(angle_deg: float) -> float:
    """
    Wrap an absolute heading to the range [0, 360).

    Using modulo keeps the heading compatible with HamBot's compass readings.
    """
    return angle_deg % 360.0


def smallest_angle_deg(angle_deg: float) -> float:
    """
    Reduce an angle to the shortest representation in (-180, 180].

    Returning signed degrees makes it easy to apply proportional control for
    rotations while understanding the direction to move.
    """
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped <= -180.0:
        wrapped += 360.0
    return wrapped


def window_min(range_image: Iterable[float], start_idx: int, end_idx: int) -> float:
    """
    Compute the minimum distance within a lidar window while clamping noise.

    Distances greater than ``MAX_RANGE`` are capped so that missing walls do
    not bias the minimum toward infinity. Invalid or non-positive samples are
    ignored, which mirrors the behaviour in Lab 1.
    """
    samples: List[float] = []
    try:
        scan = list(range_image)
    except TypeError:
        return MAX_RANGE
    total = len(scan)
    if total == 0:
        return MAX_RANGE

    idx = start_idx
    while True:
        wrapped_idx = idx % total
        reading = scan[wrapped_idx]
        if reading is None or reading <= 0.0:
            if idx == end_idx:
                break
            idx += 1
            continue

        reading_m = float(reading)
        if reading_m > MAX_RANGE * 2.0:
            # Hardware returns millimetres; convert to metres when magnitudes imply so.
            reading_m = reading_m / 1000.0

        samples.append(min(reading_m, MAX_RANGE))
        if idx == end_idx:
            break
        idx += 1

    if not samples:
        return MAX_RANGE
    return min(samples)


def read_probes(range_image: Iterable[float], follow_side: str) -> Tuple[float, float, float, float]:
    """
    Collect the key lidar beams used for the controller.

    Returns (front, left, right, side_lookahead) where ``side_lookahead``
    matches the currently followed wall. Both left and right distances are
    provided because the dead-end heuristic needs access to the opposite wall.
    """
    front = window_min(range_image, *FRONT_WIN)
    left = window_min(range_image, *LEFT_WIN)
    right = window_min(range_image, *RIGHT_WIN)
    left_fwd = window_min(range_image, *LEFT_FWD_WIN)
    right_fwd = window_min(range_image, *RIGHT_FWD_WIN)

    if follow_side == "Left":
        side_fwd = left_fwd
    else:
        side_fwd = right_fwd

    return front, left, right, side_fwd


def turn_sign(follow_side: str) -> int:
    """Return +1 when trailing the left wall and -1 for the right wall."""
    return 1 if follow_side == "Left" else -1


def mix_speeds(forward_rpm: float, omega_rpm: float) -> Tuple[float, float]:
    """
    Combine linear and angular components into individual wheel commands.

    Saturation and minimum-effort handling are applied per wheel so the robot
    keeps torque on both sides even when steering corrections are small.
    """
    left_cmd = sat_rpm(forward_rpm - omega_rpm)
    right_cmd = sat_rpm(forward_rpm + omega_rpm)
    return left_cmd, right_cmd


def supervisor_step(bot: HamBot, dt: float) -> int:
    """
    Advance the robot simulation or hardware loop by ``dt`` seconds.

    The HamBot simulator exposes ``experiment_supervisor.step`` while the physical
    platform simply requires a timed delay. Returning 0 keeps the loop running in
    hardware mode; simulation mirrors Webots by propagating the supervisor result.
    """
    if hasattr(bot, "experiment_supervisor"):
        return bot.experiment_supervisor.step(dt)  # type: ignore[attr-defined]

    time.sleep(dt)
    return 0


def get_range_image(bot: HamBot) -> Iterable[float]:
    """Fetch the latest lidar scan, supporting both simulator and hardware APIs."""
    if hasattr(bot, "get_lidar_range_image"):
        return bot.get_lidar_range_image()  # type: ignore[attr-defined]
    return bot.get_range_image()


def get_heading_deg(bot: HamBot) -> float:
    """Return the robot's compass bearing in degrees."""
    if hasattr(bot, "get_compass_reading"):
        return bot.get_compass_reading()  # type: ignore[attr-defined]
    return bot.get_heading()


def set_wheel_rpms(bot: HamBot, left_rpm: float, right_rpm: float) -> None:
    """Send wheel RPMs using whichever HamBot motor interface is available."""
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
    """Attempt to halt the robot cleanly regardless of interface differences."""
    if hasattr(bot, "stop"):
        try:
            bot.stop()  # type: ignore[attr-defined]
            return
        except Exception:
            pass

    if hasattr(bot, "stop_motors"):
        bot.stop_motors()  # type: ignore[attr-defined]
        return

    # Fallback to explicit zero commands.
    try:
        set_wheel_rpms(bot, 0.0, 0.0)
    except AttributeError:
        pass


@dataclass
class IncrementalPID:
    """
    Lightweight PID controller that tracks the integral and previous error.

    The class mirrors the controller structure used in Lab 1 but exposes an
    ``update`` method that returns the new control effort based on incremental
    state.
    """

    kp: float
    ki: float
    kd: float
    i_limit: float
    i_accum: float = 0.0
    prev_error: Optional[float] = None

    def update(self, error: float, dt: float) -> float:
        """Advance the PID calculation by one timestep."""
        if dt <= 0.0:
            return 0.0

        self.i_accum += error * dt
        self.i_accum = clamp(self.i_accum, -self.i_limit, self.i_limit)

        derivative = 0.0
        if self.prev_error is not None:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error
        return self.kp * error + self.ki * self.i_accum + self.kd * derivative

    def reset(self) -> None:
        """Clear the internal integrator and derivative memory."""
        self.i_accum = 0.0
        self.prev_error = None


def rotation_PID(curr_bearing: float, target_bearing: float) -> Tuple[float, bool]:
    """
    Compute a proportional spin command that steers toward ``target_bearing``.

    Returns a tuple (omega_command, done) where ``done`` indicates whether the
    robot is within ``ROT_EPS_DEG`` of the target heading.
    """
    angle_error = smallest_angle_deg(target_bearing - curr_bearing)
    omega_cmd = ROT_KP * angle_error
    converged = abs(angle_error) < ROT_EPS_DEG
    return omega_cmd, converged


def dead_end_condition(front: float, side_fwd: float, opposite_dist: float) -> bool:
    """
    Identify dead-end scenarios requiring a 180° escape turn.

    Direct translation of the specification: the robot triggers a dead-end turn
    when the front is blocked while either the look-ahead beam or the opposite
    wall is also too close.
    """
    return front < FRONT_TH and (side_fwd < SIDE_TH or opposite_dist < SIDE_TH)


def print_follow_banner(follow_side: str) -> None:
    """Emit a consistent status banner when the follow side changes."""
    print(f"[Task2] Following {follow_side}")


def main() -> None:
    """Entry point that wires together parsing, state machine, and control."""
    follow_side = "Left"

    bot = HamBot()
    side_pid = IncrementalPID(KP, KI, KD, I_CLAMP)
    state = RobotState.FOLLOW
    target_bearing: Optional[float] = None
    omega_prev = 0.0
    last_debug = 0.0

    print_follow_banner(follow_side)

    try:
        while supervisor_step(bot, DT_SEC) != -1:
            ranges = get_range_image(bot)
            front, left, right, side_fwd = read_probes(ranges, follow_side)
            bearing = normalize_deg(get_heading_deg(bot))

            side_distance = left if follow_side == "Left" else right
            opposite_side = right if follow_side == "Left" else left

            cmd_left = 0.0
            cmd_right = 0.0
            reported_omega = 0.0
            mode_label = state.name
            side_error: Optional[float] = None

            if state == RobotState.FOLLOW:
                if front < FRONT_TH:
                    if dead_end_condition(front, side_fwd, opposite_side):
                        target_bearing = normalize_deg(
                            bearing + 180.0 * turn_sign(follow_side)
                        )
                        state = RobotState.TURN_180
                    else:
                        target_bearing = normalize_deg(
                            bearing + 90.0 * turn_sign(follow_side)
                        )
                        state = RobotState.TURN_90
                    side_pid.reset()
                    omega_prev = 0.0
                    mode_label = state.name
                else:
                    error = SIDE_TARGET_M - side_distance
                    side_error = error
                    pid_output = side_pid.update(error, DT_SEC)

                    if side_distance > SIDE_TARGET_M + GAP and side_fwd > LOOKAHEAD_TH:
                        corner_bias = 0.25 * turn_sign(follow_side)
                        pid_output += corner_bias

                    omega = (1.0 - OMEGA_ALPHA) * omega_prev + OMEGA_ALPHA * pid_output
                    cmd_left, cmd_right = mix_speeds(BASE_FWD_RPM, omega)

                    set_wheel_rpms(bot, cmd_left, cmd_right)

                    omega_prev = omega
                    reported_omega = omega
                    mode_label = state.name

            if state in (RobotState.TURN_90, RobotState.TURN_180):
                target = target_bearing if target_bearing is not None else bearing
                omega_cmd, done = rotation_PID(bearing, target)

                cmd_left = sat_rpm(-omega_cmd)
                cmd_right = sat_rpm(omega_cmd)
                set_wheel_rpms(bot, cmd_left, cmd_right)

                reported_omega = omega_cmd
                mode_label = state.name

                if done and front >= FRONT_TH:
                    state = RobotState.FOLLOW
                    target_bearing = None
                    side_pid.reset()
                    omega_prev = 0.0
                    mode_label = state.name

            now = time.time()
            if now - last_debug >= 0.2:
                side_error_str = f"{side_error:+0.2f}m" if side_error is not None else "n/a"
                print(
                    f"[Task2] mode={mode_label:<7} front={front:0.2f}m "
                    f"left={left:0.2f}m right={right:0.2f}m look={side_fwd:0.2f}m "
                    f"err={side_error_str} ω={reported_omega:+0.2f} "
                    f"rpmL={cmd_left:0.1f} rpmR={cmd_right:0.1f} bearing={bearing:0.1f}°"
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
