import sys, os, time, math


# ======================================================================
# Section: Imports and Path Setup
# ======================================================================
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.robot_systems.robot import HamBot
from src.robot_systems.imu import IMU


# ======================================================================
# Section: Robot Constants and Encoder Configuration
# ======================================================================
R_WHEEL_M    = 0.045    # wheel radius [m]
AXLE_LEN_M   = 0.184    # wheel spacing [m]
PCT_STRAIGHT = 50.0     # base power for straight (%)
PCT_ARC      = 40.0     # base power for arcs (%)
PCT_TURN     = 10.0     # base power for in-place turns (%)
FT2M         = 0.3048

# Encoder configuration; set ENC_DEGREES=True when hardware outputs degrees.
ENC_DEGREES = False
ENC_SIGN_L  = +1.0
ENC_SIGN_R  = +1.0


def _clamp_pct(value: float) -> float:
    return max(-100.0, min(100.0, value))


# ======================================================================
# Section: IMU Helpers and Controllers
# ======================================================================
def _norm_deg_0_360(a_deg: float) -> float:
    return a_deg % 360.0

def _norm_deg_m180_180(a_deg: float) -> float:
    a = (a_deg + 180.0) % 360.0 - 180.0
    # Map -180° to +180° so headings have a single representation.
    return 180.0 if abs(a + 180.0) < 1e-9 else a

def _shortest_err_deg(target_deg: float, cur_deg: float) -> float:
    """Return signed error (target - current) in [-180, +180] deg."""
    return _norm_deg_m180_180(target_deg - cur_deg)

def _ensure_imu_on(bot, poll_hz: float = 40.0, warmup_s: float = 0.5):
    """
    Attach and start an IMU if not already present.
    Exposes bot.imu with .get_heading(...) as in imu.py.
    """
    if getattr(bot, "imu", None) is None:
        bot.imu = IMU(poll_hz=poll_hz, warmup_s=warmup_s)
        bot.imu.start()

def imu_get_heading_deg(bot, fresh_within=0.3, blocking=True, wait_timeout=0.5):
    """
    Read heading (degrees-from-East, 0..360) from IMU cache.
    Returns None if unavailable.
    """
    _ensure_imu_on(bot)
    h = bot.imu.get_heading(fresh_within=fresh_within,
                            blocking=blocking,
                            wait_timeout=wait_timeout)
    return None if h is None else float(h)

# ======================================================================
# Section: IMU-Controlled Turn Primitives
# ======================================================================
def turn_in_place_by_angle_imu(bot,
                               dtheta_rad: float,
                               fast_pct: float = 18.0,
                               slow_pct: float = 8.0,
                               kp_pct_per_deg: float = 1.8,
                               min_overcome_pct: float = 6.0,
                               settle_deg: float = 1.0,
                               settle_time_s: float = 0.20,
                               timeout_s: float = 6.0,
                               label: str = "IMU spin (relative)"):
    """
    Closed-loop in-place turn using IMU heading.
      dtheta_rad: +CCW (left), -CW (right)
      fast_pct:   max magnitude at large error
      slow_pct:   max magnitude when |err| < 8°
      kp_pct_per_deg: proportional gain -> motor % = kp * err
      min_overcome_pct: minimum |%| to overcome stiction
      settle_deg: stop when |err| <= settle_deg for settle_time_s
      timeout_s: safety cutoff
    """
    _ensure_imu_on(bot)

    # Wait for a valid current heading
    cur = imu_get_heading_deg(bot, blocking=True)
    if cur is None:
        print(f"{label}: IMU not ready; aborting")
        return

    dtheta_deg = math.degrees(dtheta_rad)
    target = _norm_deg_0_360(cur + dtheta_deg)
    _turn_to_heading_imu(bot, target, fast_pct, slow_pct, kp_pct_per_deg,
                         min_overcome_pct, settle_deg, settle_time_s,
                         timeout_s, label=label)

def _turn_to_heading_imu(bot,
                         target_deg: float,
                         fast_pct: float,
                         slow_pct: float,
                         kp_pct_per_deg: float,
                         min_overcome_pct: float,
                         settle_deg: float,
                         settle_time_s: float,
                         timeout_s: float,
                         label: str):
    """
    Core IMU closed-loop heading controller with taper & settle window.
    """
    print(f"{label}: target={target_deg:.2f}°")
    t0 = time.time()
    settle_start = None

    # Short control period keeps feedback responsive.
    dt = 0.01
    try:
        while True:
            cur = imu_get_heading_deg(bot, fresh_within=0.25, blocking=True, wait_timeout=0.3)
            if cur is None:
                # Hold still while waiting for the next heading sample.
                bot.set_left_motor_speed(0)
                bot.set_right_motor_speed(0)
                time.sleep(dt)
                if time.time() - t0 > timeout_s:
                    print(f"{label}: timeout (IMU data missing)")
                    break
                continue

            err = _shortest_err_deg(target_deg, cur)  # signed in [-180,180]
            aerr = abs(err)

            # Proportional controller converts heading error to motor power.
            cmd = kp_pct_per_deg * err

            # Reduce allowable output when the heading error is small.
            cap = slow_pct if aerr < 8.0 else fast_pct
            cmd = max(-cap, min(cap, cmd))

            # Guarantee a minimum effort whenever error exceeds the settle band.
            if abs(cmd) < min_overcome_pct and aerr > settle_deg:
                cmd = math.copysign(min_overcome_pct, cmd if cmd != 0 else err)

            # Differential drive: opposite wheel commands rotate the robot in place.
            bot.set_left_motor_speed( _clamp_pct(-cmd) )
            bot.set_right_motor_speed(_clamp_pct(+cmd) )

            # Declare success only after holding within the settle band long enough.
            if aerr <= settle_deg:
                if settle_start is None:
                    settle_start = time.time()
                elif (time.time() - settle_start) >= settle_time_s:
                    break
            else:
                settle_start = None

            if time.time() - t0 > timeout_s:
                print(f"{label}: timeout (|err|~{aerr:.2f}°); stopping at cur={cur:.2f}°")
                break

            time.sleep(dt)
    finally:
        bot.stop_motors()
        # Quick pause to bleed off residual motion.
        time.sleep(0.05)
        bot.stop_motors()
        cur = imu_get_heading_deg(bot, blocking=False) or float('nan')
        print(f"{label}: done, cur≈{cur:.2f}°, err≈{_shortest_err_deg(target_deg, cur):+.2f}°")

# ======================================================================
# Section: Waypoint Definitions (feet, radians)
# ======================================================================
WPTS = [
    (  2.0,  -2.0,  math.pi    ),   # P0
    ( -1.5,  -2.0,  math.pi    ),   # P1
    ( -2.0,  -1.5,  math.pi/2  ),   # P2
    ( -2.0,  -0.5,  math.pi/2  ),   # P3
    ( -1.0,  -0.5,  3*math.pi/2),   # P4
    ( -0.5,  -1.0,  7*math.pi/4),   # P5
    (  2.0,  -1.0,  0.0        ),   # P6
    (  2.0,   0.0,  math.pi/2  ),   # P7
    (  0.0,   0.0,  math.pi    ),   # P8
    (  0.0,   1.0,  math.pi/2  ),   # P9
    ( -2.0,   1.0,  math.pi    ),   # P10
    ( -1.0,   2.0,  0.0        ),   # P11
    (  1.5,   2.0,  0.0        ),   # P12
]



# ======================================================================
# Section: Encoder Utility Helpers
# ======================================================================
def enc_wheel_deltas_rad(bot, l0, r0):
    l, r = bot.get_encoder_readings()
    dl = ENC_SIGN_L * (l - l0)
    dr = ENC_SIGN_R * (r - r0)
    if ENC_DEGREES:
        dl = math.radians(dl); dr = math.radians(dr)
    return dl, dr

def enc_wheel_progress_m(bot, l0, r0):
    """Per-wheel distances (meters) since l0,r0."""
    dl, dr = enc_wheel_deltas_rad(bot, l0, r0)
    return R_WHEEL_M * dl, R_WHEEL_M * dr

# ======================================================================
# Section: Encoder-Only Motion Primitives
# ======================================================================
def drive_straight_feet(bot, dist_ft, pct=PCT_STRAIGHT, label="straight(ft)",
                        accel_m=0.20, decel_m=0.20, min_scale=0.35, K_bal=120.0):
    """
    Encoder-only straight with symmetric accel/decel (trapezoid) and encoder balance.
      accel_m: distance over which to ramp up to full power [m]
      decel_m: distance over which to ramp down near the end [m]
      min_scale: the minimum scale factor during ramps (0..1)
      K_bal: [% per meter] to correct (right-left) encoder drift (clamped ±8%)
    Set accel_m=0 and decel_m=0 (and min_scale=1) for constant speed.
    """
    dist_m = dist_ft * FT2M
    print(f"{label}: {dist_ft:.4f} ft ({dist_m:.3f} m) @ {pct:.1f}%  "
          f"[accel={accel_m:.2f} m, decel={decel_m:.2f} m]")

    base = +pct if dist_m >= 0 else -pct
    l0, r0 = bot.get_encoder_readings()

    # Heading correction relies solely on encoder balancing.

    while True:
        # progress (per-wheel) and center distance
        sL, sR = enc_wheel_progress_m(bot, l0, r0)         # meters
        s_center = 0.5 * (sL + sR)
        progressed = abs(s_center)
        remaining  = max(0.0, abs(dist_m) - progressed)

        # Encoder balance nudges the slower wheel to reduce drift.
        diff = sR - sL                                     # + if right > left
        turn_pct = max(-8.0, min(8.0, K_bal * diff))

        # Trapezoidal scaling for smooth acceleration and braking.
        # Ramp up based on distance already covered.
        scale_up = 1.0 if accel_m <= 0 else min(1.0, progressed / accel_m)
        # Ramp down when the remaining distance is within the decel window.
        scale_dn = 1.0 if decel_m <= 0 else min(1.0, remaining / decel_m)
        scale = max(min_scale, min(scale_up, scale_dn))    # Enforce minimum scale.

        bot.set_left_motor_speed(  _clamp_pct((base + turn_pct) * scale) )
        bot.set_right_motor_speed( _clamp_pct((base - turn_pct) * scale) )

        if progressed >= abs(dist_m) - 1e-3:
            break
        time.sleep(0.01)

    bot.stop_motors()
    print(f"{label}: done (meas ≈ {s_center:.3f} m)")

def drive_arc_by_wheel_dists(bot, sL_m, sR_m, base_pct=PCT_ARC, label="arc"):
    """
    Drive an arc by commanding per-wheel distances and finishing both together.
    Uses per-wheel taper near the end so the shorter (inner) path doesn't overshoot.
    """
    # Determine signed distances for each wheel.
    signL = 1.0 if sL_m >= 0 else -1.0
    signR = 1.0 if sR_m >= 0 else -1.0
    aL, aR = abs(sL_m), abs(sR_m)
    if aL < 1e-6 and aR < 1e-6:
        print(f"{label}: degenerate, skip"); 
        return

    # Scale motor percentages so both wheels finish simultaneously.
    smax = max(aL, aR)
    pctL = _clamp_pct(base_pct * (aL / smax)) * signL
    pctR = _clamp_pct(base_pct * (aR / smax)) * signR

    l0, r0 = bot.get_encoder_readings()
    print(f"{label}: targets L={sL_m:.4f} m, R={sR_m:.4f} m | power L={pctL:.1f}%, R={pctR:.1f}%")

    bot.set_left_motor_speed(pctL)
    bot.set_right_motor_speed(pctR)

    # Taper window is the larger of 3 cm or 35% of the shorter path.
    r_end = max(0.03, 0.35 * min(aL, aR))

    while True:
        sL, sR = enc_wheel_progress_m(bot, l0, r0)
        remL = max(0.0, aL - abs(sL))
        remR = max(0.0, aR - abs(sR))

        # Scale each wheel independently to avoid overdriving the inner path.
        scaleL = 1.0 if remL > r_end else max(0.30, remL / r_end)
        scaleR = 1.0 if remR > r_end else max(0.30, remR / r_end)

        # Stop a wheel once it has effectively reached the target distance.
        cmdL = 0.0 if remL <= 1e-3 else _clamp_pct(pctL * scaleL)
        cmdR = 0.0 if remR <= 1e-3 else _clamp_pct(pctR * scaleR)

        bot.set_left_motor_speed(cmdL)
        bot.set_right_motor_speed(cmdR)

        if (abs(sL) >= aL - 1e-3) and (abs(sR) >= aR - 1e-3):
            break
        time.sleep(0.01)

    bot.stop_motors()
    print(f"{label}: done (L≈{abs(sL):.4f} m, R≈{abs(sR):.4f} m)")

# ======================================================================
# Section: Leg-Specific Waypoint Functions
# ======================================================================
def p0_to_p1(bot):
    """Straight from P0 to P1 (encoder-only)."""
    x1_ft, y1_ft, _ = WPTS[0]
    x2_ft, y2_ft, _ = WPTS[1]
    d_ft = math.hypot(x2_ft - x1_ft, y2_ft - y1_ft)  # 3.5 ft
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P0→P1")

def p1_to_p2_arc(bot):
    """Perfect quarter RIGHT arc from P1 to P2 computed from waypoints. No trim."""
    x1, y1, th1 = WPTS[1]
    x2, y2, th2 = WPTS[2]

    # geometry from waypoints (quarter circle)
    dx_ft, dy_ft = (x2 - x1), (y2 - y1)
    c_ft  = math.hypot(dx_ft, dy_ft)         # sqrt(0.5) ft
    R_ft  = c_ft / math.sqrt(2.0)            # center radius for 90°
    R_m   = R_ft * FT2M
    Theta = math.pi / 2.0                    # 90° right

    # wheel path lengths (right wheel inner/shorter)
    inner = R_m - AXLE_LEN_M/2.0
    outer = R_m + AXLE_LEN_M/2.0
    sR = Theta * inner
    sL = Theta * outer

    print(f"\nP1→P2 geometry: chord={c_ft:.6f} ft, R={R_ft:.3f} ft ({R_m:.4f} m)")
    print(f"Wheel dists: L={sL:.5f} m, R={sR:.5f} m  (right shorter)")
    drive_arc_by_wheel_dists(bot, sL, sR, base_pct=PCT_ARC, label="P1→P2 arc")

def p2_to_p3(bot):
    """Straight up +1.0 ft from P2 to P3 (same x, y increases by 1 ft)."""
    x1_ft, y1_ft, _ = WPTS[2]
    x2_ft, y2_ft, _ = WPTS[3]
    d_ft = math.hypot(x2_ft - x1_ft, y2_ft - y1_ft)  # should be 1.0 ft
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P2→P3")

def p3_to_p4_arc(bot):
    """Perfect 180° RIGHT (clockwise) arc from P3 to P4 computed from waypoints. No trim."""
    x1, y1, th1 = WPTS[3]
    x2, y2, th2 = WPTS[4]

    # geometry from waypoints (half circle)
    dx_ft, dy_ft = (x2 - x1), (y2 - y1)     # should be +1.0 ft, 0.0 ft
    c_ft  = math.hypot(dx_ft, dy_ft)        # chord = 1.0 ft
    # For a semicircle, chord = 2R  => R = chord/2
    R_ft  = c_ft / 2.0                      # => 0.5 ft
    R_m   = R_ft * FT2M                     # 0.1524 m
    Theta = math.pi                         # 180° right

    # wheel path lengths: right = inner, left = outer (clockwise)
    inner = R_m - AXLE_LEN_M/2.0            # 0.1524 - 0.0920 = 0.0604 m
    outer = R_m + AXLE_LEN_M/2.0            # 0.1524 + 0.0920 = 0.2444 m
    sR = Theta * inner                      # ≈ 0.1898 m
    sL = Theta * outer                      # ≈ 0.7679 m

    print(f"\nP3→P4 geometry: chord={c_ft:.6f} ft, R={R_ft:.3f} ft ({R_m:.4f} m), Θ=180°")
    print(f"Radii: inner={inner:.4f} m, outer={outer:.4f} m")
    print(f"Wheel dists: L={sL:.5f} m, R={sR:.5f} m  (right shorter)")
    drive_arc_by_wheel_dists(bot, sL, sR, base_pct=PCT_ARC, label="P3→P4 arc")

def p4_to_p5(bot):
    """
    P4 → P5:
        1) Rotate from the P4 orientation to the P5 orientation.
        2) Drive the diagonal segment between the waypoints.
    """

    # 1) Pre-turn derived from waypoint headings
    heading_delta = ((WPTS[5][2] - WPTS[4][2] + math.pi) % (2 * math.pi)) - math.pi
    delta_deg = math.degrees(heading_delta)
    print(f"\nP4→P5 pre-turn: Δθ={delta_deg:+.1f}°")
    turn_in_place_by_angle_imu(bot, dtheta_rad=heading_delta, label="P4→P5 pre-turn (IMU)")

    bot.stop_motors()
    time.sleep(0.12)

    # 2) Straight along the diagonal: d = sqrt(0.5) ft
    d_ft = math.sqrt(0.5)
    print(f"P4→P5 straight: {d_ft:.4f} ft (0.7071 ft expected)")
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P4→P5 straight")

    bot.stop_motors()
    time.sleep(0.12)

def p5_to_p6(bot, margin_pct: float = 0.02):
    """
    P5→P6:
      1) IMU pre-turn using the waypoint heading difference.
      2) Straight with a small safety margin (default 2% shorter).
    """
    x1, y1, th1 = WPTS[5]   # P5
    x2, y2, th2 = WPTS[6]   # P6

    # --- Step 1: Heading change from waypoint orientations ---
    heading_delta = ((th2 - th1 + math.pi) % (2*math.pi)) - math.pi
    delta_deg = math.degrees(heading_delta)
    print(f"\nP5→P6 turn: Δθ={delta_deg:+.1f}° (relative)")
    turn_in_place_by_angle_imu(
        bot,
        dtheta_rad=heading_delta,
        # slightly gentler settle so it doesn't false-timeout
        fast_pct=16.0, slow_pct=5.0, kp_pct_per_deg=1.8,
        settle_deg=1.2, settle_time_s=0.25, timeout_s=8.0,
        label="P5→P6 pre-turn (IMU)"
    )
    bot.stop_motors()
    time.sleep(0.12)

    # --- Step 2: Straight distance with margin ---
    d_ft_nominal = math.hypot(x2 - x1, y2 - y1)
    d_ft = d_ft_nominal * (1.0 - float(margin_pct))   # 2% shorter by default
    print(f"P5→P6 straight: {d_ft:.4f} ft (nominal {d_ft_nominal:.4f} ft, margin {margin_pct*100:.1f}%)")
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P5→P6 straight (with margin)")

    bot.stop_motors()
    print(f"P5→P6: done; now at ~{WPTS[6]} (straight reduced by {margin_pct*100:.1f}%)")


def p6_to_p7(bot):
    """
    Encoder-only P6→P7:
      Turn to match target heading, then straight distance from WPTS.
    """
    x1, y1, th1 = WPTS[6]   # P6
    x2, y2, th2 = WPTS[7]   # P7

    # --- Step 1: Heading change ---
    dtheta = th2 - th1
    dtheta = ((dtheta + math.pi) % (2*math.pi)) - math.pi  # normalize
    print(f"\nP6→P7 turn: Δθ={math.degrees(dtheta):+.1f}°")
    # turn_in_place_by_angle(bot, dtheta_rad=dtheta, pct=PCT_TURN, label="P6→P7 turn")
    turn_in_place_by_angle_imu(bot, dtheta_rad=dtheta, label="P6→P7 turn (IMU)")


    bot.stop_motors()
    time.sleep(0.12)

    # --- Step 2: Straight distance ---
    d_ft = math.hypot(x2 - x1, y2 - y1)
    print(f"P6→P7 straight: {d_ft:.4f} ft")
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P6→P7 straight")

    bot.stop_motors()
    print(f"P6→P7: done; now at {WPTS[7]}")

def p7_to_p8(bot):
    """
    Encoder-only P7→P8:
      Turn to match target heading, then straight distance from WPTS.
    """
    x1, y1, th1 = WPTS[7]   # P7
    x2, y2, th2 = WPTS[8]   # P8

    # --- Step 1: Heading change ---
    dtheta = th2 - th1
    dtheta = ((dtheta + math.pi) % (2*math.pi)) - math.pi  # normalize to [-π,π]
    print(f"\nP7→P8 turn: Δθ={math.degrees(dtheta):+.1f}°")
    # turn_in_place_by_angle(bot, dtheta_rad=dtheta, pct=PCT_TURN, label="P7→P8 turn")
    turn_in_place_by_angle_imu(bot, dtheta_rad=dtheta, label="P7→P8 turn (IMU)")


    bot.stop_motors()
    time.sleep(0.12)

    # --- Step 2: Straight distance ---
    d_ft = math.hypot(x2 - x1, y2 - y1)
    print(f"P7→P8 straight: {d_ft:.4f} ft")
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P7→P8 straight")

    bot.stop_motors()
    print(f"P7→P8: done; now at {WPTS[8]}")

def p8_to_p9(bot):
    """
    Encoder-only P8→P9:
      -90° (right) turn, then 1.0 ft straight.
    """
    x1, y1, th1 = WPTS[8]   # P8
    x2, y2, th2 = WPTS[9]   # P9

    # --- Step 1: Heading change ---
    dtheta = th2 - th1
    dtheta = ((dtheta + math.pi) % (2*math.pi)) - math.pi
    print(f"\nP8→P9 turn: Δθ={math.degrees(dtheta):+.1f}°")
    # turn_in_place_by_angle(bot, dtheta_rad=dtheta, pct=PCT_TURN, label="P8→P9 turn")
    turn_in_place_by_angle_imu(bot, dtheta_rad=dtheta, label="P8→P9 turn (IMU)")


    bot.stop_motors()
    time.sleep(0.12)

    # --- Step 2: Straight distance ---
    d_ft = math.hypot(x2 - x1, y2 - y1)
    print(f"P8→P9 straight: {d_ft:.4f} ft")
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P8→P9 straight")

    bot.stop_motors()
    print(f"P8→P9: done; now at {WPTS[9]}")


def p9_to_p10(bot):
    """
    Encoder-only P9→P10:
      +90° (left) turn, then 2.0 ft straight.
    """
    x1, y1, th1 = WPTS[9]   # P9
    x2, y2, th2 = WPTS[10]  # P10

    # --- Step 1: Heading change ---
    dtheta = th2 - th1
    dtheta = ((dtheta + math.pi) % (2*math.pi)) - math.pi
    print(f"\nP9→P10 turn: Δθ={math.degrees(dtheta):+.1f}°")
    # turn_in_place_by_angle(bot, dtheta_rad=dtheta, pct=PCT_TURN, label="P9→P10 turn")
    turn_in_place_by_angle_imu(bot, dtheta_rad=dtheta, label="P9→P10 turn (IMU)")

    bot.stop_motors()
    time.sleep(0.12)

    # --- Step 2: Straight distance ---
    d_ft = math.hypot(x2 - x1, y2 - y1)
    print(f"P9→P10 straight: {d_ft:.4f} ft")
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P9→P10 straight")

    bot.stop_motors()
    print(f"P9→P10: done; now at {WPTS[10]}")

def p10_to_p11(bot):
    """
    P10 → P11:
      1) +90° CCW pre-turn to face +y (north)
      2) 90° RIGHT (clockwise) quarter-circle arc to reach (-1, 2) facing +x (east)
    Geometry is derived from WPTS; no straight segments.
    """
    # Waypoints
    x1, y1, th1 = WPTS[10]   # P10 = (-2.0, 1.0, π)
    x2, y2, th2 = WPTS[11]   # P11 = (-1.0, 2.0, 0)

    # ---------- Step 1: Pre-turn to face north (π/2) ----------
    th_mid = math.pi / 2  # north
    dtheta1 = ((th_mid - th1 + math.pi) % (2*math.pi)) - math.pi
    print(f"\nP10→P11 pre-turn: Δθ₁={math.degrees(dtheta1):+.1f}° to face north")
    # turn_in_place_by_angle(bot, dtheta_rad=dtheta1, pct=PCT_TURN, label="P10→P11 pre-turn")
    turn_in_place_by_angle_imu(bot, dtheta_rad=dtheta1, label="P10→P11 turn (IMU)")

    bot.stop_motors()
    time.sleep(0.12)

    # ---------- Step 2: Quarter-circle RIGHT arc ----------
    # From north to east with Δx=+1, Δy=+1 ⇒ radius R = 1.0 ft (quarter circle)
    dx_ft, dy_ft = (x2 - x1), (y2 - y1)              # (+1, +1)
    c_ft  = math.hypot(dx_ft, dy_ft)                 # chord = √2 ft
    R_ft  = c_ft / math.sqrt(2.0)                    # R = 1.0 ft for a 90° arc
    R_m   = R_ft * FT2M
    Theta = math.pi / 2.0                            # 90° magnitude

    # Right (inner) / Left (outer) radii for a RIGHT (clockwise) arc
    inner = R_m - AXLE_LEN_M/2.0
    outer = R_m + AXLE_LEN_M/2.0
    if inner <= 0:
        print("[WARN] Arc radius too small for axle length; increase R or adjust path.")

    sR = Theta * inner   # right wheel (shorter path)
    sL = Theta * outer   # left wheel  (longer path)

    print(f"P10→P11 arc geometry: chord={c_ft:.4f} ft, R={R_ft:.4f} ft ({R_m:.4f} m)")
    print(f"Wheel targets: L={sL:.4f} m, R={sR:.4f} m  (RIGHT turn, both forward)")
    drive_arc_by_wheel_dists(bot, sL, sR, base_pct=PCT_ARC, label="P10→P11 quarter arc")

    bot.stop_motors()
    time.sleep(0.10)

    print(f"P10→P11: done; now at {WPTS[11]} (expected)")

def p11_to_p12(bot):
    """
    Encoder-only P11→P12:
      Heading stays 0 rad (east). Drive straight |Δ| from WPTS.
    """
    x1, y1, th1 = WPTS[11]   # P11 = (-1.0, 2.0, 0)
    x2, y2, th2 = WPTS[12]   # P12 = ( 1.5, 2.0, 0)

    # Turn needed (should be ~0, but keep logic generic)
    dtheta = ((th2 - th1 + math.pi) % (2*math.pi)) - math.pi
    if abs(dtheta) > 1e-3:
        print(f"\nP11→P12 trim turn: Δθ={math.degrees(dtheta):+.1f}°")
        turn_in_place_by_angle_imu(bot, dtheta_rad=dtheta, label="P11-P12 turn (IMU)")

    bot.stop_motors()
    time.sleep(0.12)

    # Straight distance from waypoints
    d_ft = math.hypot(x2 - x1, y2 - y1)  # should be 2.5 ft
    print(f"P11→P12 straight: {d_ft:.4f} ft along +x")
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P11→P12 straight")

    bot.stop_motors()
    print(f"P11→P12: done; now at {WPTS[12]}")

def p12_to_p13(bot):
    """
    Final segment P12→P13 (constant-velocity arc for T=0.5 s):
      VR = 0.24 m/s, VL = 0.80 m/s
    We compute R, ICC side, ω, D, θ, then execute the arc by distance.
    """
    VR = 0.24  # m/s
    VL = 0.80  # m/s
    T  = 0.5   # s
    L  = AXLE_LEN_M

    # Kinematics
    v = 0.5 * (VR + VL)         # robot center linear speed
    omega = (VR - VL) / L       # rad/s  (sign: +CCW, -CW)
    R = v / omega if abs(omega) > 1e-9 else float('inf')  # m
    theta = omega * T           # radians traveled by robot
    D = v * T                   # center arc length

    # Wheel path lengths to finish in time T
    sL = VL * T                 # meters
    sR = VR * T                 # meters

    # Report
    side = "left" if R > 0 else "right"
    print("\nP12→P13 arc (spec speeds):")
    print(f"  v = {v:.3f} m/s, ω = {omega:.5f} rad/s")
    print(f"  R = {abs(R):.4f} m to the {side} (ICC)")
    print(f"  θ = {math.degrees(theta):+.2f}° ({theta:+.5f} rad),  D = {D:.3f} m")
    print(f"  Wheel distances over T={T:.2f}s: sL={sL:.3f} m, sR={sR:.3f} m")

    # Execute arc by distances (keeps the ratio so both wheels finish together)
    drive_arc_by_wheel_dists(bot, sL_m=sL, sR_m=sR, base_pct=PCT_ARC, label="P12→P13 arc")

# ======================================================================
# Section: Entry Point
# ======================================================================
def main():
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    try:
        print("\n=== P0 → P1 (straight) ===")
        p0_to_p1(bot)

        print("\n=== P1 → P2 (quarter right arc) ===")
        p1_to_p2_arc(bot)

        print("\n=== P2 → P3 (straight 1.0 ft up) ===")
        p2_to_p3(bot)

        print("\n=== P3 → P4 (half-circle right arc) ===")
        p3_to_p4_arc(bot)

        print("\n=== P4 → P5 (turn, straight, turn) ===")
        p4_to_p5(bot)

        print("\n=== P5 → P6 (turn, straight) ===")
        p5_to_p6(bot)

        print("\n=== P6 → P7 (turn, straight) ===")
        p6_to_p7(bot)

        print("\n=== P7 → P8 (turn, straight) ===")
        p7_to_p8(bot)

        print("\n=== P8 → P9 (turn, straight) ===")
        p8_to_p9(bot)

        print("\n=== P9 → P10 (turn, straight) ===")
        p9_to_p10(bot)

        print("\n=== P10 → P11 (pre-turn, quarter arc) ===")
        p10_to_p11(bot)

        print("\n=== P11 → P12 (straight) ===")
        p11_to_p12(bot)
        print("\n=== P12 → P13 (arc by given VR,VL,T) ===")
        p12_to_p13(bot)

        print("\nDone.")
    finally:
        bot.stop_motors()
        try:
            bot.disconnect_robot()
        except Exception:
            pass

if __name__ == "__main__":
    main()
