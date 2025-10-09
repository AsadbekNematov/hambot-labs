"""
HamBot Lab 2 Task 2 controller tuned for tight, slow left-wall following.

The script keeps everything in a single file, exposes the full wall-following
logic, and remains easy to extend with runtime side toggles. For now the turn
states are disabled so the robot continuously steers while wrapping around
compact obstacles (e.g., circling a portable fridge). The operating speed is
intentionally very low so small distance errors become obvious during tuning.
All control remains fully compatible with the physical HamBot API.
"""

from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Iterable, Optional, Tuple

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
SIDE_TARGET_M: float = 0.20
BASE_FWD_RPM: float = 6.0
MAX_RPM: float = 35.0
MIN_EFFORT_RPM: float = 4.0

# ---------------------------------------------------------------------------
# Additional tuning for curvature and frontal avoidance
# ---------------------------------------------------------------------------
CORNER_WRAP_GAIN: float = 1.2
FRONT_REPULSE_GAIN: float = 10.0
ENABLE_TURN_STATES: bool = True

# ---------------------------------------------------------------------------
# Strict tracking modifiers
# ---------------------------------------------------------------------------
STRICT_ERROR_BAND: float = 0.18
STRICT_WRAP_ERR: float = 0.05
MIN_FWD_FACTOR: float = 0.05
FRONT_STOP_DIST: float = 0.18
FRONT_SLOW_DIST: float = 0.45


# ---------------------------------------------------------------------------
# Detection thresholds
# ---------------------------------------------------------------------------
FRONT_TH: float = 0.35
SIDE_TH: float = 0.26
LOOKAHEAD_TH: float = 0.70
GAP: float = 0.10


# ---------------------------------------------------------------------------
# Gains, smoothing, and turn controller parameters
# ---------------------------------------------------------------------------
KP: float = 4.2
KI: float = 0.16
KD: float = 0.35
I_CLAMP: float = 0.5
OMEGA_ALPHA: float = 0.6

ROT_KP: float = 0.5
ROT_EPS_DEG: float = 4.0

DT_SEC: float = 0.032
LOG_PERIOD: float = 0.10

# ---------------------------------------------------------------------------
# Robot geometry and quarter-turn tuning
# ---------------------------------------------------------------------------
AXLE_LENGTH_M: float = 0.184
WHEEL_RADIUS_M: float = 0.045
TURN_ARC_INNER_RPM: float = 6.0
TURN_ARC_OUTER_RPM: float = 22.0
TURN_ARC_OVERSHOOT: float = 1.08
TURN_ARC_MIN_TIME: float = 0.55
TURN_EXIT_FRONT_CLEAR: float = 0.55
TURN_EXIT_SIDE_ERR: float = 0.15


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


def window_min(range_image: Iterable[float], start_idx: int, end_idx: int) -> Tuple[float, int]:
    """
    Compute the minimum distance within a lidar window while clamping noise.

    Distances greater than ``MAX_RANGE`` are capped so that missing walls do
    not bias the minimum toward infinity. Invalid or non-positive samples are
    ignored. The function also reports how many valid samples contributed so
    the caller can detect whether the reading is trustworthy.
    """
    try:
        scan = list(range_image)
    except TypeError:
        return MAX_RANGE, 0
    total = len(scan)
    if total == 0:
        return MAX_RANGE, 0

    idx = start_idx
    valid_count = 0
    best = MAX_RANGE
    while True:
        wrapped_idx = idx % total
        reading = scan[wrapped_idx]
        if reading is not None and reading > 0.0:
            reading_m = float(reading)
            if reading_m > MAX_RANGE * 2.0:
                # Hardware returns millimetres; convert to metres when magnitudes imply so.
                reading_m = reading_m / 1000.0

            reading_m = min(reading_m, MAX_RANGE)
            best = min(best, reading_m)
            valid_count += 1

        if idx == end_idx:
            break
        idx += 1

    if valid_count == 0:
        return MAX_RANGE, 0
    return best, valid_count


def read_probes(range_image: Iterable[float], follow_side: str) -> Tuple[
    float, float, float, float, int, int, int, int
]:
    """
    Collect the key lidar beams used for the controller.

    Returns (front, left, right, side_lookahead) where ``side_lookahead``
    matches the currently followed wall. Both left and right distances are
    provided because the dead-end heuristic needs access to the opposite wall.
    """
    front, front_valid = window_min(range_image, *FRONT_WIN)
    left, left_valid = window_min(range_image, *LEFT_WIN)
    right, right_valid = window_min(range_image, *RIGHT_WIN)
    left_fwd, left_fwd_valid = window_min(range_image, *LEFT_FWD_WIN)
    right_fwd, right_fwd_valid = window_min(range_image, *RIGHT_FWD_WIN)

    if follow_side == "Left":
        side_fwd = left_fwd
        side_valid = left_fwd_valid
    else:
        side_fwd = right_fwd
        side_valid = right_fwd_valid

    return (
        front,
        left,
        right,
        side_fwd,
        front_valid,
        left_valid,
        right_valid,
        side_valid,
    )


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


@dataclass
class ArcTurnContext:
    """Store parameters governing a quarter-turn arc manoeuvre."""

    follow_side: str
    inner_rpm: float
    outer_rpm: float
    start_time: float
    duration: float


def rpm_to_linear(rpm: float) -> float:
    """Convert wheel RPM to linear velocity at the wheel perimeter (m/s)."""
    return abs(rpm) * (2.0 * math.pi * WHEEL_RADIUS_M) / 60.0


def create_arc_turn_context(follow_side: str) -> ArcTurnContext:
    """
    Configure a quarter-turn arc using fixed inner/outer wheel RPMs.

    Uses differential-drive kinematics to estimate the duration required to
    sweep 90 degrees without stopping in place. The duration is slightly
    overestimated via TURN_ARC_OVERSHOOT so the wall can be reacquired.
    """
    inner_rpm = TURN_ARC_INNER_RPM
    outer_rpm = TURN_ARC_OUTER_RPM
    v_inner = rpm_to_linear(inner_rpm)
    v_outer = rpm_to_linear(outer_rpm)
    omega = abs(v_outer - v_inner) / max(AXLE_LENGTH_M, 1e-6)
    duration = (math.pi / 2.0) / max(omega, 1e-6)
    duration *= TURN_ARC_OVERSHOOT
    duration = max(duration, TURN_ARC_MIN_TIME)

    return ArcTurnContext(
        follow_side=follow_side,
        inner_rpm=inner_rpm,
        outer_rpm=outer_rpm,
        start_time=time.time(),
        duration=duration,
    )


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
    lidar_ready = False
    turn_states_enabled = ENABLE_TURN_STATES
    arc_ctx: Optional[ArcTurnContext] = None

    print_follow_banner(follow_side)

    try:
        while supervisor_step(bot, DT_SEC) != -1:
            ranges = get_range_image(bot)
            (
                front,
                left,
                right,
                side_fwd,
                front_valid,
                left_valid,
                right_valid,
                look_valid,
            ) = read_probes(ranges, follow_side)
            bearing = normalize_deg(get_heading_deg(bot))

            side_distance = left if follow_side == "Left" else right
            opposite_side = right if follow_side == "Left" else left
            side_samples = left_valid if follow_side == "Left" else right_valid

            cmd_left = 0.0
            cmd_right = 0.0
            reported_omega = 0.0
            omega_unsmoothed = 0.0
            mode_label = state.name
            side_error: Optional[float] = None
            forward_scale = 0.0
            forward_cmd = 0.0

            if state == RobotState.FOLLOW:
                front_blocked = front < FRONT_TH

                if turn_states_enabled and front_blocked:
                    if dead_end_condition(front, side_fwd, opposite_side):
                        target_bearing = normalize_deg(
                            bearing + 180.0 * turn_sign(follow_side)
                        )
                        arc_ctx = None
                        state = RobotState.TURN_180
                    else:
                        arc_ctx = create_arc_turn_context(follow_side)
                        target_bearing = None
                        state = RobotState.TURN_90
                        print(
                            f"[Task2] Quarter arc turn start ({follow_side}) "
                            f"≈ {arc_ctx.duration:.2f}s"
                        )
                    side_pid.reset()
                    omega_prev = 0.0
                    mode_label = state.name
                else:
                    if not lidar_ready and side_samples == 0:
                        set_wheel_rpms(bot, 0.0, 0.0)
                        if time.time() - last_debug >= 0.5:
                            print("[Task2] Waiting for lidar side data...")
                            last_debug = time.time()
                        continue
                    elif side_samples > 0:
                        lidar_ready = True

                    error = side_distance - SIDE_TARGET_M
                    side_error = error
                    pid_output = side_pid.update(error, DT_SEC)
                    pid_output *= turn_sign(follow_side)

                    abs_error = abs(error)
                    if STRICT_ERROR_BAND > 1e-6:
                        forward_scale = clamp(
                            1.0 - abs_error / STRICT_ERROR_BAND, MIN_FWD_FACTOR, 1.0
                        )
                    else:
                        forward_scale = 1.0

                    if front < FRONT_SLOW_DIST:
                        denom = max(FRONT_SLOW_DIST - FRONT_STOP_DIST, 1e-3)
                        front_scale = clamp(
                            (front - FRONT_STOP_DIST) / denom,
                            0.0,
                            1.0,
                        )
                        forward_scale = min(forward_scale, front_scale)

                    if side_distance > SIDE_TARGET_M + GAP and side_fwd > LOOKAHEAD_TH:
                        pid_output += CORNER_WRAP_GAIN * turn_sign(follow_side)

                    if error > STRICT_WRAP_ERR:
                        pid_output += turn_sign(follow_side) * (
                            1.8 * (error - STRICT_WRAP_ERR) / max(STRICT_ERROR_BAND, 1e-3)
                        )
                    elif error < -STRICT_WRAP_ERR:
                        pid_output -= turn_sign(follow_side) * (
                            1.2 * (-error - STRICT_WRAP_ERR) / max(STRICT_ERROR_BAND, 1e-3)
                        )

                    if not turn_states_enabled and front_blocked:
                        front_bias = FRONT_REPULSE_GAIN * (FRONT_TH - front)
                        pid_output += front_bias * turn_sign(follow_side)

                    omega_unsmoothed = pid_output
                    omega = (1.0 - OMEGA_ALPHA) * omega_prev + OMEGA_ALPHA * omega_unsmoothed

                    forward_cmd = BASE_FWD_RPM * forward_scale
                    cmd_left, cmd_right = mix_speeds(forward_cmd, omega)

                    set_wheel_rpms(bot, cmd_left, cmd_right)

                    omega_prev = omega
                    reported_omega = omega
                    mode_label = state.name

            if turn_states_enabled and state == RobotState.TURN_90:
                if arc_ctx is not None:
                    if arc_ctx.follow_side == "Left":
                        cmd_left = sat_rpm(arc_ctx.inner_rpm)
                        cmd_right = sat_rpm(arc_ctx.outer_rpm)
                    else:
                        cmd_left = sat_rpm(arc_ctx.outer_rpm)
                        cmd_right = sat_rpm(arc_ctx.inner_rpm)

                    set_wheel_rpms(bot, cmd_left, cmd_right)

                    reported_omega = cmd_right - cmd_left
                    omega_unsmoothed = reported_omega
                    forward_cmd = max(0.0, min(cmd_left, cmd_right))
                    if BASE_FWD_RPM > 1e-6:
                        forward_scale = clamp(forward_cmd / BASE_FWD_RPM, 0.0, 1.5)

                    elapsed = time.time() - arc_ctx.start_time
                    front_clear = front >= TURN_EXIT_FRONT_CLEAR
                    side_error_now = side_distance - SIDE_TARGET_M
                    side_error = side_error_now
                    good_side = abs(side_error_now) <= TURN_EXIT_SIDE_ERR

                    if (
                        elapsed >= arc_ctx.duration
                        or (
                            elapsed >= TURN_ARC_MIN_TIME
                            and front_clear
                            and good_side
                        )
                    ):
                        state = RobotState.FOLLOW
                        arc_ctx = None
                        target_bearing = None
                        side_pid.reset()
                        omega_prev = 0.0
                        mode_label = state.name
                else:
                    target = target_bearing if target_bearing is not None else bearing
                    omega_cmd, done = rotation_PID(bearing, target)

                    cmd_left = sat_rpm(-omega_cmd)
                    cmd_right = sat_rpm(omega_cmd)
                    set_wheel_rpms(bot, cmd_left, cmd_right)

                    side_error = None
                    reported_omega = omega_cmd
                    omega_unsmoothed = omega_cmd
                    forward_cmd = 0.0
                    forward_scale = 0.0
                    mode_label = state.name

                    if done and front >= FRONT_TH:
                        state = RobotState.FOLLOW
                        target_bearing = None
                        side_pid.reset()
                        omega_prev = 0.0
                        mode_label = state.name

            if turn_states_enabled and state == RobotState.TURN_180:
                target = target_bearing if target_bearing is not None else bearing
                omega_cmd, done = rotation_PID(bearing, target)

                cmd_left = sat_rpm(-omega_cmd)
                cmd_right = sat_rpm(omega_cmd)
                set_wheel_rpms(bot, cmd_left, cmd_right)

                side_error = None
                reported_omega = omega_cmd
                omega_unsmoothed = omega_cmd
                forward_cmd = 0.0
                forward_scale = 0.0
                mode_label = state.name

                if done and front >= FRONT_TH:
                    state = RobotState.FOLLOW
                    target_bearing = None
                    side_pid.reset()
                    omega_prev = 0.0
                    mode_label = state.name

            now = time.time()
            if now - last_debug >= LOG_PERIOD:
                side_error_str = f"{side_error:+0.2f}m" if side_error is not None else "n/a"
                side_sample_str = (
                    f"{side_samples} side samples, {look_valid} look samples"
                )
                print(
                    f"[Task2] mode={mode_label:<7} front={front:0.2f}m "
                    f"left={left:0.2f}m right={right:0.2f}m look={side_fwd:0.2f}m "
                    f"err={side_error_str} ({side_sample_str}) ω={reported_omega:+0.2f} "
                    f"ω_raw={omega_unsmoothed:+0.2f} scale={forward_scale:0.2f} "
                    f"spd={forward_cmd:0.1f} rpmL={cmd_left:0.1f} rpmR={cmd_right:0.1f} "
                    f"bearing={bearing:0.1f}°"
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
