# ------------------------------------------------------------
# Lab 1 – P0→P1 straight, P1→P2 90° right arc, P2→P3 straight, P3→P4 180° right arc
# Robot: Physical HamBot (Encoders + IMU), percent power control
#
# Run:
#   python lab1_p0_p4.py
# ------------------------------------------------------------

import sys, os, time, math
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from robot_systems.robot import HamBot

# ----------------- Robot constants -----------------
R_WHEEL_M    = 0.045    # wheel radius [m]  (90 mm diameter)
AXLE_LEN_M   = 0.184    # wheel spacing [m]
PCT_STRAIGHT = 60.0     # straight drive base power (%)
PCT_ARC      = 55.0     # arc drive base power (%)
FT2M         = 0.3048

# Encoders: your hardware reports radians (from quick_check)
ENC_DEGREES  = False
ENC_SIGN_L   = +1.0
ENC_SIGN_R   = +1.0

def _clamp_pct(x): return max(-100.0, min(100.0, x))
def wrap_pi(a):     return (a + math.pi) % (2.0*math.pi) - math.pi

# ----------------- Waypoints (feet, radians) -----------------
# (x_ft, y_ft, theta_rad) – exactly as given by professor
WPTS = [
    (  2.0,  -2.0,  math.pi    ),   # P0
    ( -1.5,  -2.0,  math.pi    ),   # P1
    ( -2.0,  -1.5,  math.pi/2  ),   # P2
    ( -2.0,  -0.5,  math.pi/2  ),   # P3
    ( -1.0,  -0.5,  3*math.pi/2),   # P4
]

# ----------------- Encoder helpers -----------------
def enc_wheel_deltas_rad(bot, l0, r0):
    l, r = bot.get_encoder_readings()
    dl = ENC_SIGN_L * (l - l0)
    dr = ENC_SIGN_R * (r - r0)
    if ENC_DEGREES:  # supported, but you have radians
        dl = math.radians(dl); dr = math.radians(dr)
    return dl, dr

def enc_distance_center_m(bot, l0, r0):
    dl, dr = enc_wheel_deltas_rad(bot, l0, r0)
    return R_WHEEL_M * 0.5 * (dl + dr)

# ----------------- Straight with heading hold -----------------
def drive_straight_feet(bot, dist_ft, pct=PCT_STRAIGHT, kp_heading=1.6, label="straight(ft)"):
    dist_m = dist_ft * FT2M
    print(f"{label}: target {dist_ft:.3f} ft ({dist_m:.3f} m) at {pct:.1f}%")

    yaw0 = bot.get_heading() or 0.0           # degrees
    l0, r0 = bot.get_encoder_readings()       # radians on your hardware

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

# ----------------- Arc driver (time-synced wheel distances) -----------------
def drive_arc_by_wheel_dists(bot, sL_m, sR_m, base_pct=PCT_ARC, label="arc"):
    """
    Drive an arc by commanding both wheels forward in proportion
    to their target distances so they finish together. Encoders only.
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

    # start
    l0, r0 = bot.get_encoder_readings()
    print(f"{label}: target wheel distances L={sL_m:.4f} m, R={sR_m:.4f} m")
    print(f"{label}: initial power L={pctL:.1f}%, R={pctR:.1f}%")
    bot.set_left_motor_speed(pctL)
    bot.set_right_motor_speed(pctR)

    # ramp down near the end, stop when both wheels meet targets
    while True:
        dl, dr = enc_wheel_deltas_rad(bot, l0, r0)
        progL = R_WHEEL_M * dl   # meters
        progR = R_WHEEL_M * dr

        remL = abs(aL - abs(progL))
        remR = abs(aR - abs(progR))
        rem_min = min(remL, remR)

        # gentle common slowdown in last 0.10 m
        scale = 1.0 if rem_min > 0.10 else max(0.30, rem_min / 0.10)
        bot.set_left_motor_speed(_clamp_pct(pctL * scale))
        bot.set_right_motor_speed(_clamp_pct(pctR * scale))

        if (abs(progL) >= aL - 1e-3) and (abs(progR) >= aR - 1e-3):
            break
        time.sleep(0.01)

    bot.stop_motors()
    print(f"{label}: done (L≈{abs(progL):.4f} m, R≈{abs(progR):.4f} m)")

# ----------------- Leg-specific functions -----------------
def p0_to_p1(bot):
    """Straight from P0 to P1 using heading hold."""
    x1_ft, y1_ft, _ = WPTS[0]
    x2_ft, y2_ft, _ = WPTS[1]
    d_ft = math.hypot(x2_ft - x1_ft, y2_ft - y1_ft)  # 3.5 ft
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, kp_heading=1.6, label="P0→P1")

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
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, kp_heading=1.6, label="P2→P3")

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

# ----------------- Main -----------------
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

        print("\nDone.")
    finally:
        bot.stop_motors()
        try:
            bot.disconnect_robot()
        except Exception:
            pass

if __name__ == "__main__":
    main()
