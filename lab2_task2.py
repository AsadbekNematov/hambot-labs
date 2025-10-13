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
from typing import Iterable, Optional, Sequence, Tuple

# Allow running the controller directly from the repository root without
# modifying PYTHONPATH externally.
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.robot_systems.robot import HamBot  # noqa: E402 (import after sys.path)


# Lidar sampling windows around the robot.
FRONT_WIN: Tuple[int, int] = (175, 185)
LEFT_WIN: Tuple[int, int] = (92, 104)
RIGHT_WIN: Tuple[int, int] = (245, 270)
MAX_RANGE: float = 4.0

# Simplified motion parameters.
SIDE_TARGET_M: float = 0.25
SIDE_DEADBAND_M: float = 0.005
BASE_FWD_RPM: float = 12.0
BASE_FWD_MIN_RPM: float = 7.0
STEER_SLOWDOWN_FRACTION: float = 0.6
STEER_KP_NEAR: float = 22.0
STEER_KP_FAR: float = 55.0
STEER_ERROR_FULL_M: float = 0.10
MAX_STEER_RPM: float = 16.0
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

TURN_RIGHT_ANGLE_90: float = -80.0
TURN_RIGHT_ANGLE_180: float = -170.0

MAX_SENSOR_ATTEMPTS: int = 6

LOG_PREFIX = "[Task2]"

DT_SEC: float = 0.032
LIDAR_SAMPLE_TTL_SEC: float = 0.35
POST_TURN_LIDAR_REFRESH_SEC: float = 1.0
INITIAL_LIDAR_WARMUP_SEC: float = 2.0

# Geometry for arc turns.
AXLE_LEN_M: float = 0.184
WHEEL_RADIUS_M: float = 0.045

# Detection thresholds for spotting a left wall drop (corner entry).
LEFT_DROP_MIN_DELTA_M: float = 0.08
LEFT_DROP_RATIO: float = 0.6
LEFT_DROP_PREV_MIN_M: float = 0.16
LEFT_DROP_COOLDOWN_SEC: float = 2.5
LEFT_DROP_FRONT_MARGIN_M: float = 0.05

# Cornering behaviour configuration (keeps ~0.25 m clearance through a 90° bend).
CORNER_TARGET_RADIUS_M: float = SIDE_TARGET_M  # centreline radius
CORNER_ARC_DEG: float = 80.0
CORNER_BASE_CENTER_RPM: float = 15.0
CORNER_MIN_CENTER_RPM: float = BASE_FWD_MIN_RPM
CORNER_SLOW_BAND_DEG: float = 35.0
CORNER_EPS_DEG: float = 3.0
CORNER_SETTLE_SEC: float = 0.08
CORNER_TIMEOUT_SEC: float = 3.5
CORNER_MIN_SCALE: float = 0.52
CORNER_SENSOR_PERIOD_SEC: float = 0.06
CORNER_RADIUS_KP: float = 0.75
CORNER_RADIUS_MAX_DELTA_M: float = 0.10
CORNER_RADIUS_MIN_M: float = AXLE_LEN_M / 2.0 + 0.01
CORNER_RADIUS_MAX_M: float = 0.45
CORNER_RADIUS_BIAS_M: float = 0.02
CORNER_LEFT_EXTRA_RPM: float = 2.0

# Running estimate of the nominal left-wall clearance.
LEFT_REF_ALPHA: float = 0.25
LEFT_REF_MIN_M: float = 0.16
LEFT_REF_MAX_M: float = 0.45


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


def fetch_lidar_scan(bot: HamBot) -> Tuple[Sequence[float], Optional[Sequence[float]]]:
    """
    Fetch the current lidar scan and optional per-angle timestamps, if available.
    """
    if hasattr(bot, "get_range_image_with_timestamps"):
        scan, timestamps = bot.get_range_image_with_timestamps()  # type: ignore[attr-defined]
    else:
        scan = get_range_image(bot)
        timestamps = None

    if scan == -1:
        return [], None

    try:
        scan_list = list(scan)
    except TypeError:
        scan_list = [float(scan)]

    if timestamps is None:
        stamp_list: Optional[list[float]] = None
    else:
        try:
            stamp_list = list(timestamps)
        except TypeError:
            stamp_list = None

    return scan_list, stamp_list


def window_min(
    range_image: Iterable[float],
    start_idx: int,
    end_idx: int,
    sample_timestamps: Optional[Iterable[float]] = None,
) -> Tuple[float, bool]:
    """
    Return the minimum distance inside a lidar window and flag whether anything was usable.
    """
    if isinstance(range_image, list):
        scan = range_image
    else:
        try:
            scan = list(range_image)
        except TypeError:
            return MAX_RANGE, False

    total = len(scan)
    if total == 0:
        return MAX_RANGE, False

    timestamps: Optional[list[float]]
    if sample_timestamps is None:
        timestamps = None
    else:
        if isinstance(sample_timestamps, list):
            timestamps = sample_timestamps
        else:
            try:
                timestamps = list(sample_timestamps)
                if len(timestamps) != total:
                    timestamps = None
            except TypeError:
                timestamps = None

    now = time.monotonic()
    best = MAX_RANGE
    had_sample = False
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
        reading_m = min(reading_m, MAX_RANGE)

        if timestamps is not None:
            sample_time = timestamps[idx % total]
            if sample_time <= 0.0 or now - sample_time > LIDAR_SAMPLE_TTL_SEC:
                if idx == end_idx:
                    break
                idx += 1
                continue

        if reading_m < best:
            best = reading_m
        had_sample = True
        if idx == end_idx:
            break
        idx += 1

    if not had_sample:
        return MAX_RANGE, False
    return best, True


def get_probe_distances(
    range_image: Iterable[float],
    sample_timestamps: Optional[Iterable[float]] = None,
) -> Tuple[Tuple[float, float, float], Tuple[bool, bool, bool]]:
    """Extract front, left, and right wall distances and validity flags from the scan."""
    front, front_ok = window_min(range_image, *FRONT_WIN, sample_timestamps=sample_timestamps)
    left, left_ok = window_min(range_image, *LEFT_WIN, sample_timestamps=sample_timestamps)
    right, right_ok = window_min(range_image, *RIGHT_WIN, sample_timestamps=sample_timestamps)
    return (front, left, right), (front_ok, left_ok, right_ok)


def poll_distances(bot: HamBot, attempts: int = MAX_SENSOR_ATTEMPTS) -> Optional[Tuple[float, float, float]]:
    """
    Attempt to fetch a fresh set of wall distances, retrying if all probes read MAX_RANGE.

        Returns None when no valid measurements arrive within ``attempts`` retries.
    """
    for attempt in range(1, attempts + 1):
        ranges, timestamps = fetch_lidar_scan(bot)
        (front, left, right), (front_ok, left_ok, right_ok) = get_probe_distances(
            ranges, sample_timestamps=timestamps
        )
        if front_ok or left_ok or right_ok:
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


def refresh_lidar_stream(bot: HamBot, duration_sec: float = POST_TURN_LIDAR_REFRESH_SEC) -> None:
    """
    Actively step the simulation/robot for a short period so the lidar publishes a fresh scan.
    """
    deadline = time.time() + max(0.0, duration_sec)
    while time.time() < deadline:
        _ = fetch_lidar_scan(bot)
        if supervisor_step(bot, DT_SEC) == -1:
            break


def compute_steering(error_m: float) -> float:
    """
    Convert left distance error into a steering term.

    Positive error means we are too close to the wall, so steer to the right.
    """
    if abs(error_m) < SIDE_DEADBAND_M:
        return 0.0
    abs_error = abs(error_m)
    blend = clamp((abs_error - SIDE_DEADBAND_M) / max(STEER_ERROR_FULL_M - SIDE_DEADBAND_M, 1e-6), 0.0, 1.0)
    gain = STEER_KP_NEAR + (STEER_KP_FAR - STEER_KP_NEAR) * blend
    steer = gain * error_m
    return clamp(steer, -MAX_STEER_RPM, MAX_STEER_RPM)


def compute_forward_speed(error_m: float) -> float:
    """Slow down the forward velocity when the distance error is large."""
    abs_error = abs(error_m)
    if abs_error <= SIDE_DEADBAND_M:
        return BASE_FWD_RPM

    blend = clamp(abs_error / max(STEER_ERROR_FULL_M, 1e-6), 0.0, 1.0)
    slowdown = STEER_SLOWDOWN_FRACTION * blend
    forward = BASE_FWD_RPM * (1.0 - slowdown)
    return clamp(forward, BASE_FWD_MIN_RPM, BASE_FWD_RPM)


def rpm_to_mps(rpm: float) -> float:
    """Convert wheel RPM to linear speed at the robot centre line."""
    return (rpm * 2.0 * math.pi * WHEEL_RADIUS_M) / 60.0


def mps_to_rpm(speed_mps: float) -> float:
    """Convert linear speed (m/s) back to wheel RPM."""
    return (speed_mps * 60.0) / (2.0 * math.pi * WHEEL_RADIUS_M)


def compute_arc_wheel_rpms(center_rpm: float, radius_m: float, turn_left: bool = True) -> Tuple[float, float]:
    """
    Compute the ideal per-wheel RPMs for a constant-radius arc (no saturation).

    ``center_rpm`` is the straight-line wheel RPM that would yield the requested
    centreline speed; ``radius_m`` is the radius of the robot's centre path.
    """
    half_axle = AXLE_LEN_M / 2.0
    radius = max(radius_m, half_axle + 1e-4)
    inner_radius = max(radius - half_axle, 1e-4)
    outer_radius = radius + half_axle

    inner_rpm = center_rpm * (inner_radius / radius)
    outer_rpm = center_rpm * (outer_radius / radius)

    if turn_left:
        return inner_rpm, outer_rpm
    return outer_rpm, inner_rpm


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


def perform_left_corner_arc(
    bot: HamBot,
    radius_m: float = CORNER_TARGET_RADIUS_M,
    angle_deg: float = CORNER_ARC_DEG,
    desired_left_m: Optional[float] = None,
    context: str = "ARC_LEFT",
) -> None:
    """Follow a quarter-circle arc to the left while advancing."""
    if angle_deg <= 1e-3:
        return

    desired_left = clamp(desired_left_m if desired_left_m is not None else SIDE_TARGET_M, LEFT_REF_MIN_M, LEFT_REF_MAX_M)

    biased_radius = radius_m + CORNER_RADIUS_BIAS_M
    base_radius = clamp(biased_radius, CORNER_RADIUS_MIN_M, CORNER_RADIUS_MAX_M)
    radius_cmd = base_radius
    half_axle = AXLE_LEN_M / 2.0
    inner_radius = max(base_radius - half_axle, 1e-4)

    base_left, base_right = compute_arc_wheel_rpms(CORNER_BASE_CENTER_RPM, base_radius, turn_left=True)
    min_center_rpm = max(CORNER_MIN_CENTER_RPM, MIN_EFFORT_RPM * base_radius / inner_radius)
    slow_left, _ = compute_arc_wheel_rpms(min_center_rpm, base_radius, turn_left=True)

    if abs(base_left) > 1e-6:
        slow_scale = abs(slow_left) / abs(base_left)
    else:
        slow_scale = 1.0
    scale_floor = max(CORNER_MIN_SCALE, slow_scale)

    expected_time = (math.radians(angle_deg) * base_radius) / max(rpm_to_mps(CORNER_BASE_CENTER_RPM), 1e-6)
    timeout_limit = max(CORNER_TIMEOUT_SEC, expected_time * 1.5)
    log_status(
        context,
        (
            f"Begin left arc radius={base_radius:.2f}m "
            f"(bias={CORNER_RADIUS_BIAS_M:+0.3f}m) angle={angle_deg:.0f}° "
            f"(allow {timeout_limit:.1f}s)"
        ),
    )

    start_heading = normalize_deg(get_heading_deg(bot))
    target = normalize_deg(start_heading + angle_deg)
    start_time = time.time()
    settle_start: Optional[float] = None
    final_error = float("nan")
    last_sensor = 0.0
    last_feedback: Optional[float] = None
    last_feedback_log = 0.0
    radius_adjust = 0.0
    last_cmd_log = 0.0

    while True:
        heading = normalize_deg(get_heading_deg(bot))
        error = smallest_angle_deg(target - heading)
        final_error = error
        abs_error = abs(error)

        if abs_error <= CORNER_EPS_DEG:
            if settle_start is None:
                settle_start = time.time()
            elif time.time() - settle_start >= CORNER_SETTLE_SEC:
                break
            set_wheel_rpms(bot, 0.0, 0.0)
        else:
            settle_start = None
            speed_ratio = min(abs_error / max(CORNER_SLOW_BAND_DEG, 1e-6), 1.0)
            scale = slow_scale + (1.0 - slow_scale) * speed_ratio
            scale = max(scale_floor, scale)

            now = time.time()
            if now - last_sensor >= CORNER_SENSOR_PERIOD_SEC:
                ranges, timestamps = fetch_lidar_scan(bot)
                (front_s, left_s, _), flags = get_probe_distances(ranges, sample_timestamps=timestamps)
                if flags[1]:
                    last_feedback = left_s
                    err_left = desired_left - left_s
                    radius_adjust = clamp(
                        CORNER_RADIUS_KP * err_left,
                        -CORNER_RADIUS_MAX_DELTA_M,
                        CORNER_RADIUS_MAX_DELTA_M,
                    )
                    radius_cmd = clamp(
                        base_radius + radius_adjust,
                        CORNER_RADIUS_MIN_M,
                        CORNER_RADIUS_MAX_M,
                    )
                last_sensor = now

            effective_radius = clamp(radius_cmd, CORNER_RADIUS_MIN_M, CORNER_RADIUS_MAX_M)
            inner_r = max(effective_radius - half_axle, 1e-4)
            center_rpm = CORNER_BASE_CENTER_RPM * scale
            required_center = MIN_EFFORT_RPM * effective_radius / inner_r
            center_rpm = clamp(center_rpm, min_center_rpm, CORNER_BASE_CENTER_RPM)
            center_rpm = max(center_rpm, required_center)
            cmd_left, cmd_right = compute_arc_wheel_rpms(center_rpm, effective_radius, turn_left=True)
            cmd_left += CORNER_LEFT_EXTRA_RPM
            if abs(cmd_left) < MIN_EFFORT_RPM:
                cmd_left = math.copysign(MIN_EFFORT_RPM, cmd_left if abs(cmd_left) > 1e-6 else 1.0)
            cmd_left = sat_rpm(cmd_left)
            cmd_right = sat_rpm(cmd_right)
            set_wheel_rpms(bot, cmd_left, cmd_right)

            if last_feedback is not None and now - last_feedback_log >= 0.25:
                log_status(
                    context,
                    (
                            f"heading_err={error:+.1f}° scale={scale:.2f} "
                        f"left={last_feedback:0.2f}m target={desired_left:0.2f}m "
                        f"radius={effective_radius:0.2f}m rpmL={cmd_left:0.1f} rpmR={cmd_right:0.1f}"
                    ),
                )
                last_feedback_log = now
                last_cmd_log = now
            elif now - last_cmd_log >= 0.35:
                log_status(
                    context,
                    (
                        f"heading_err={error:+.1f}° scale={scale:.2f} "
                        f"radius={effective_radius:0.2f}m rpmL={cmd_left:0.1f} rpmR={cmd_right:0.1f}"
                    ),
                )
                last_cmd_log = now

        if supervisor_step(bot, DT_SEC) == -1:
            log_status(context, "Supervisor requested shutdown during arc")
            break

        if time.time() - start_time >= timeout_limit:
            log_status(context, f"Timeout while executing arc (remaining error {error:+.1f}°)")
            break

    set_wheel_rpms(bot, 0.0, 0.0)
    supervisor_step(bot, DT_SEC)
    log_status(context, f"Arc complete; residual error {final_error:+.1f}°")


def main() -> None:
    """Simple left-wall following loop."""
    bot = HamBot()
    last_debug = 0.0

    log_status("INIT", "Following left wall")

    # Pre-fill lidar values to let the sensor settle before control starts.
    log_status("INIT", "Warming up lidar stream")
    refresh_lidar_stream(bot, duration_sec=INITIAL_LIDAR_WARMUP_SEC)
    log_status("INIT", "Lidar ready")

    last_valid: Optional[Tuple[float, float, float]] = poll_distances(bot)
    if last_valid is None:
        log_status("LIDAR", "No valid scan after warm-up; waiting for data before moving")
    else:
        log_status(
            "INIT",
            f"Initial ranges front={last_valid[0]:0.2f}m left={last_valid[1]:0.2f}m right={last_valid[2]:0.2f}m",
        )

    last_left_sample: Optional[float] = last_valid[1] if last_valid is not None else None
    last_corner_time = 0.0
    left_reference = SIDE_TARGET_M

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
                new_sample = False
            else:
                front_dist, left_dist, right_dist = distances
                last_valid = distances
                new_sample = True
                if LEFT_REF_MIN_M <= left_dist <= LEFT_REF_MAX_M:
                    filtered = clamp(left_dist, LEFT_REF_MIN_M, LEFT_REF_MAX_M)
                    left_reference = (1.0 - LEFT_REF_ALPHA) * left_reference + LEFT_REF_ALPHA * filtered

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

                # Give the lidar time to publish a scan aligned with the new heading.
                refresh_lidar_stream(bot)
                refreshed = poll_distances(bot)
                if refreshed is None:
                    log_status("LIDAR", "Turn complete but still waiting for fresh scan")
                    set_wheel_rpms(bot, 0.0, 0.0)
                    last_valid = None
                    last_left_sample = None
                    last_debug = time.time()
                    continue

                front_dist, left_dist, right_dist = refreshed
                last_valid = refreshed
                last_left_sample = left_dist
                if LEFT_REF_MIN_M <= left_dist <= LEFT_REF_MAX_M:
                    filtered = clamp(left_dist, LEFT_REF_MIN_M, LEFT_REF_MAX_M)
                    left_reference = (1.0 - LEFT_REF_ALPHA) * left_reference + LEFT_REF_ALPHA * filtered
                log_status(
                    "FOLLOW",
                    (
                        "Resuming after turn with "
                        f"front={front_dist:0.2f}m left={left_dist:0.2f}m right={right_dist:0.2f}m"
                    ),
                )
                last_corner_time = time.time()
                last_debug = time.time()
                continue

            left_drop = False
            if new_sample and last_left_sample is not None:
                drop = last_left_sample - left_dist
                enough_time = (time.time() - last_corner_time) >= LEFT_DROP_COOLDOWN_SEC
                if (
                    last_left_sample >= LEFT_DROP_PREV_MIN_M
                    and drop >= LEFT_DROP_MIN_DELTA_M
                    and left_dist <= last_left_sample * LEFT_DROP_RATIO
                    and front_dist > (FRONT_BLOCK_M + LEFT_DROP_FRONT_MARGIN_M)
                    and enough_time
                        ):
                    left_drop = True

            if left_drop:
                arc_target_left = clamp(left_reference, LEFT_REF_MIN_M, LEFT_REF_MAX_M)
                log_status(
                    "CORNER",
                    (
                        f"Left drop {last_left_sample:0.2f}m→{left_dist:0.2f}m "
                        f"(front={front_dist:0.2f}m) -> ARC_LEFT radius≈{arc_target_left:0.2f}m"
                    ),
                )
                perform_left_corner_arc(
                    bot,
                    radius_m=arc_target_left,
                    desired_left_m=arc_target_left,
                    context="ARC_LEFT",
                )

                # Allow the lidar stream to catch up after the arc completes.
                refresh_lidar_stream(bot)
                refreshed = poll_distances(bot)
                if refreshed is None:
                    log_status("LIDAR", "Arc complete but waiting for fresh scan")
                    set_wheel_rpms(bot, 0.0, 0.0)
                    last_valid = None
                    last_left_sample = None
                    last_corner_time = time.time()
                    last_debug = time.time()
                    continue

                front_dist, left_dist, right_dist = refreshed
                last_valid = refreshed
                last_left_sample = left_dist
                last_corner_time = time.time()
                if LEFT_REF_MIN_M <= left_dist <= LEFT_REF_MAX_M:
                    filtered = clamp(left_dist, LEFT_REF_MIN_M, LEFT_REF_MAX_M)
                    left_reference = (1.0 - LEFT_REF_ALPHA) * left_reference + LEFT_REF_ALPHA * filtered
                log_status(
                    "FOLLOW",
                    (
                        "Resuming after arc with "
                        f"front={front_dist:0.2f}m left={left_dist:0.2f}m right={right_dist:0.2f}m"
                    ),
                )
                last_debug = time.time()
                continue

            error = SIDE_TARGET_M - left_dist
            steer = compute_steering(error)
            forward_rpm = compute_forward_speed(error)

            cmd_left, cmd_right = mix_wheel_commands(forward_rpm, steer)
            set_wheel_rpms(bot, cmd_left, cmd_right)

            now = time.time()
            if now - last_debug >= 0.2:
                log_status(
                    "FOLLOW",
                    (
                        f"front={front_dist:0.2f}m left={left_dist:0.2f}m "
                        f"right={right_dist:0.2f}m target={SIDE_TARGET_M:0.2f}m "
                        f"error={error:+0.2f}m steer={steer:+0.2f} fwd={forward_rpm:0.1f} "
                        f"rpmL={cmd_left:0.1f} rpmR={cmd_right:0.1f}"
                    ),
                )
                last_debug = now

            if new_sample:
                last_left_sample = left_dist

    except KeyboardInterrupt:
        print("\n[Task2] Interrupted by user.")
    finally:
        try:
            stop_robot(bot)
        except Exception:
            pass


if __name__ == "__main__":
    main()
