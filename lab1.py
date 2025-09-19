# ------------------------------------------------------------
# Lab 1 – P0→P1 straight, then P1→P2 90° right arc (from waypoints)
# Robot: Physical HamBot (Encoders + IMU), percent power control
#
# Run:
#   python lab1_p0_p2_arc.py
# ------------------------------------------------------------

import sys, os, time, math
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from robot_systems.robot import HamBot

# ----------------- Robot constants -----------------
R_WHEEL_M    = 0.045    # wheel radius [m]  (90 mm diameter)
AXLE_LEN_M   = 0.184    # wheel spacing [m]
PCT_STRAIGHT = 45.0     # straight drive power (%)
PCT_ARC      = 30.0     # arc drive base power (%)
PCT_TURN     = 16.0     # in-place turn if we need small correction
FT2M         = 0.3048

ENC_DEGREES  = False    # encoders are radians (from your quick check)
ENC_SIGN_L   = +1.0
ENC_SIGN_R   = +1.0

def _clamp_pct(x): return max(-100.0, min(100.0, x))
def wrap_pi(a):     return (a + math.pi) % (2.0*math.pi) - math.pi

# ----------------- Waypoints (feet, radians) -----------------
# (x_ft, y_ft, theta_rad) – use exactly what the professor gave.
WPTS = [
    (  2.0,  -2.0,  math.pi    ),   # P0
    ( -1.5,  -2.0,  math.pi    ),   # P1
    ( -2.0,  -1.5,  math.pi/2  ),   # P2
    ( -2.0,  -0.5,  math.pi/2  ),   # P3 ...
]

# ----------------- Encoder helpers -----------------
def enc_wheel_deltas_rad(bot, l0, r0):
    l, r = bot.get_encoder_readings()
    dl = ENC_SIGN_L * (l - l0)
    dr = ENC_SIGN_R * (r - r0)
    if ENC_DEGREES:  # not expected for you, but supported
        dl = math.radians(dl); dr = math.radians(dr)
    return dl, dr

def enc_distance_center_m(bot, l0, r0):
    dl, dr = enc_wheel_deltas_rad(bot, l0, r0)
    return R_WHEEL_M * 0.5 * (dl + dr)

# ----------------- Straight with heading hold (from your working code) -----------------
def drive_straight_feet(bot, dist_ft, pct=PCT_STRAIGHT, kp_heading=1.6, label="straight(ft)"):
    dist_m = dist_ft * FT2M
    print(f"{label}: target {dist_ft:.3f} ft ({dist_m:.3f} m) at {pct:.1f}%")

    yaw0 = bot.get_heading() or 0.0           # degrees
    l0, r0 = bot.get_encoder_readings()       # radians (your hardware)

    base = +pct if dist_m >= 0 else -pct
    traveled = 0.0

    while True:
        # heading error (radians), wrapped to [-pi, pi]
        yaw = bot.get_heading()
        if yaw is None: yaw = yaw0
        e = math.radians(((yaw0 - yaw + 180) % 360) - 180)

        # ~0.1% per degree, clamped ±8%
        turn_pct = max(-8.0, min(8.0, math.degrees(e) * 0.10))

        # distance & a short ramp inside last 0.25 m
        s = enc_distance_center_m(bot, l0, r0)
        traveled = s if dist_m >= 0 else -s
        remain = max(0.0, abs(dist_m) - traveled)
        scale = 1.0 if remain > 0.25 else max(0.28, remain / 0.25)

        bot.set_left_motor_speed(  _clamp_pct((base - turn_pct) * scale) )
        bot.set_right_motor_speed( _clamp_pct((base + turn_pct) * scale) )

        if traveled >= abs(dist_m) - 1e-3:
            break
        time.sleep(0.01)

    bot.stop_motors()
    print(f"{label}: done (meas ≈ {traveled:.3f} m)")

# ----------------- In-place turn (small correction only) -----------------
def turn_in_place(bot, dtheta_rad, pct=PCT_TURN, label="turn"):
    dtheta = wrap_pi(dtheta_rad)
    if abs(dtheta) < math.radians(2):  # ignore tiny errors
        print(f"{label}: skip (|Δθ| < 2°)")
        return
    pct = abs(pct)
    if dtheta > 0:  # CCW
        bot.set_left_motor_speed(_clamp_pct(-pct))
        bot.set_right_motor_speed(_clamp_pct(+pct))
    else:           # CW
        bot.set_left_motor_speed(_clamp_pct(+pct))
        bot.set_right_motor_speed(_clamp_pct(-pct))

    # simple IMU integration
    last = math.radians(bot.get_heading() or 0.0)
    acc = 0.0
    while True:
        h = bot.get_heading() or math.degrees(last)
        rad = math.radians(h)
        acc += wrap_pi(rad - last)
        last = rad
        if abs(acc) >= abs(dtheta):
            break
        time.sleep(0.005)
    bot.stop_motors()
    print(f"{label}: target {math.degrees(dtheta):+.1f}°, meas ≈ {math.degrees(acc):+.1f}°")

# ----------------- Arc driver (time-synced wheel distances) -----------------
def drive_arc_by_wheel_dists(bot, sL_m, sR_m, base_pct=PCT_ARC, label="arc"):
    """
    Drive an arc by commanding both wheels forward/backward in proportion
    to their target distances so they finish together. Uses encoders only.
    sL_m, sR_m are signed path lengths for left/right wheels [m].
    """
    # directions & proportional speeds
    signL = 1.0 if sL_m >= 0 else -1.0
    signR = 1.0 if sR_m >= 0 else -1.0
    aL, aR = abs(sL_m), abs(sR_m)
    if aL < 1e-6 and aR < 1e-6:
        print(f"{label}: degenerate, skip"); return

    # ratio so both finish at the same time (speed ∝ distance)
    smax = max(aL, aR)
    pctL = _clamp_pct(base_pct * (aL / smax)) * signL
    pctR = _clamp_pct(base_pct * (aR / smax)) * signR

    # encoder targets (radians)
    l0, r0 = bot.get_encoder_readings()
    target_dphi_L = (aL / R_WHEEL_M) * (1 if signL > 0 else -1)
    target_dphi_R = (aR / R_WHEEL_M) * (1 if signR > 0 else -1)

    print(f"{label}: target wheel distances L={sL_m:.4f} m, R={sR_m:.4f} m")
    print(f"{label}: initial power L={pctL:.1f}%, R={pctR:.1f}%")

    bot.set_left_motor_speed(pctL)
    bot.set_right_motor_speed(pctR)

    # ramp down near the end, stop when both wheels meet targets
    while True:
        dl, dr = enc_wheel_deltas_rad(bot, l0, r0)
        # progress in meters (signed)
        progL = R_WHEEL_M * dl
        progR = R_WHEEL_M * dr

        remL = abs(aL - abs(progL))
        remR = abs(aR - abs(progR))
        rem_min = min(remL, remR)

        # gentle common slowdown in the last 0.10 m
        scale = 1.0 if rem_min > 0.10 else max(0.30, rem_min / 0.10)
        bot.set_left_motor_speed(_clamp_pct(pctL * scale))
        bot.set_right_motor_speed(_clamp_pct(pctR * scale))

        if (abs(progL) >= aL - 1e-3) and (abs(progR) >= aR - 1e-3):
            break
        time.sleep(0.01)

    bot.stop_motors()
    print(f"{label}: done (L≈{abs(progL):.4f} m, R≈{abs(progR):.4f} m)")

# ----------------- Build arc from waypoints Pk→Pk+1 -----------------
def arc_from_waypoints(bot, p_from, p_to, label="Pk→Pk+1"):
    """
    Use the professor's (x,y,θ) waypoints to compute a 90° arc.
    For P1→P2 we expect: right turn (clockwise), |Δx|=|Δy| => quarter circle.
    """
    (x1_ft, y1_ft, th1), (x2_ft, y2_ft, th2) = p_from, p_to
    dx_ft, dy_ft = (x2_ft - x1_ft), (y2_ft - y1_ft)
    c_ft = math.hypot(dx_ft, dy_ft)               # chord length [ft]
    dtheta = wrap_pi(th2 - th1)                    # signed turn

    # Expect 90° turn here:
    if abs(abs(dtheta) - math.pi/2) > math.radians(5):
        print(f"{label}: WARNING – Δθ not ≈ 90° (Δθ={math.degrees(dtheta):.1f}°). Continuing anyway.")
    # For a 90° arc, R = chord / √2
    R_ft = c_ft / math.sqrt(2.0)
    R_m  = R_ft * FT2M
    turn_right = (dtheta < 0)                      # CW if θ decreases

    # Wheel path lengths for 90° arc
    Theta = abs(dtheta)                            # positive 90°
    inner = R_m - AXLE_LEN_M/2.0
    outer = R_m + AXLE_LEN_M/2.0

    # If inner < 0, inner wheel must reverse (tight arc); here it's positive.
    if inner < 0:
        print(f"{label}: tight arc, inner radius < 0 → inner wheel reverses")

    if turn_right:
        sL = Theta * outer      # left wheel = outer (longer)
        sR = Theta * inner      # right wheel = inner (shorter)
    else:
        sL = Theta * inner      # left wheel = inner (left turn)
        sR = Theta * outer

    print(f"\n=== {label} (arc) ===")
    print(f"dx={dx_ft:+.3f} ft, dy={dy_ft:+.3f} ft, chord={c_ft:.3f} ft")
    print(f"Δθ={math.degrees(dtheta):+.1f}°  → {'RIGHT/CW' if turn_right else 'LEFT/CCW'} 90°")
    print(f"Arc radius R={R_ft:.3f} ft ({R_m:.4f} m); inner={inner:.4f} m, outer={outer:.4f} m")
    print(f"Wheel distances: Left={sL:.4f} m, Right={sR:.4f} m (left should be longer on RIGHT turn)")

    drive_arc_by_wheel_dists(bot, sL, sR, base_pct=PCT_ARC, label=f"{label} arc")

    # Optional: clean up final heading to exactly θ_to with a small in-place tweak
    yaw_now_deg = bot.get_heading() or 0.0
    err = wrap_pi(th2 - math.radians(yaw_now_deg))
    turn_in_place(bot, err, pct=PCT_TURN, label=f"{label} heading trim")

# ----------------- Main -----------------
def main():
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    try:
        # P0 -> P1 (straight, as you already verified)
        print("\n=== P0 → P1 (straight) ===")
        x1_ft, y1_ft, _ = WPTS[0]
        x2_ft, y2_ft, _ = WPTS[1]
        d_ft = math.hypot(x2_ft - x1_ft, y2_ft - y1_ft)  # 3.5 ft
        drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, kp_heading=1.6, label="P0→P1")

        # P1 -> P2 (quarter-circle RIGHT arc computed from waypoints)
        arc_from_waypoints(bot, WPTS[1], WPTS[2], label="P1→P2")

        print("\nDone.")
    finally:
        bot.stop_motors()
        try:
            bot.disconnect_robot()
        except Exception:
            pass

if __name__ == "__main__":
    main()
