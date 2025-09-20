import sys, os, time, math
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from robot_systems.robot import HamBot

# ----------------- Robot constants -----------------
R_WHEEL_M    = 0.045    # wheel radius [m]
AXLE_LEN_M   = 0.184    # wheel spacing [m]
PCT_STRAIGHT = 60.0     # base power for straight (%)
PCT_ARC      = 60.0     # base power for arcs (%)
PCT_TURN     = 40.0     # base power for in-place turns (%)
FT2M         = 0.3048

# Encoders: your hardware reports radians (from your quick check)
ENC_DEGREES  = False
ENC_SIGN_L   = +1.0
ENC_SIGN_R   = +1.0

def _clamp_pct(x): return max(-100.0, min(100.0, x))

# ----------------- Waypoints (feet, radians) -----------------
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
]


# ----------------- Encoder helpers -----------------
def enc_wheel_deltas_rad(bot, l0, r0):
    l, r = bot.get_encoder_readings()
    dl = ENC_SIGN_L * (l - l0)
    dr = ENC_SIGN_R * (r - r0)
    if ENC_DEGREES:
        dl = math.radians(dl); dr = math.radians(dr)
    return dl, dr

def enc_distance_center_m(bot, l0, r0):
    """Average of wheel distances (meters)."""
    dl, dr = enc_wheel_deltas_rad(bot, l0, r0)
    return R_WHEEL_M * 0.5 * (dl + dr)

def enc_wheel_progress_m(bot, l0, r0):
    """Per-wheel distances (meters) since l0,r0."""
    dl, dr = enc_wheel_deltas_rad(bot, l0, r0)
    return R_WHEEL_M * dl, R_WHEEL_M * dr

# ----------------- Motion primitives (ENCODER-ONLY) -----------------
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

    while True:
        # progress (per-wheel) and center distance
        sL, sR = enc_wheel_progress_m(bot, l0, r0)         # meters
        s_center = 0.5 * (sL + sR)
        progressed = abs(s_center)
        remaining  = max(0.0, abs(dist_m) - progressed)

        # encoder balance (push the slower side a bit)
        diff = sR - sL                                     # + if right > left
        turn_pct = max(-8.0, min(8.0, K_bal * diff))

        # ----- trapezoid scaling -----
        # ramp-up based on how far we've gone
        scale_up = 1.0 if accel_m <= 0 else min(1.0, progressed / accel_m)
        # ramp-down based on how much is left
        scale_dn = 1.0 if decel_m <= 0 else min(1.0, remaining / decel_m)
        scale = max(min_scale, min(scale_up, scale_dn))    # smooth & bounded

        bot.set_left_motor_speed(  _clamp_pct((base + turn_pct) * scale) )
        bot.set_right_motor_speed( _clamp_pct((base - turn_pct) * scale) )

        if progressed >= abs(dist_m) - 1e-3:
            break
        time.sleep(0.01)

    bot.stop_motors()
    print(f"{label}: done (meas ≈ {s_center:.3f} m)")

def turn_in_place_by_angle(bot, dtheta_rad, pct=PCT_TURN, label="spin"):
    """
    Encoder-only in-place spin by dtheta (rad). + = CCW (left), − = CW (right).
    Targets per-wheel distances: sR = + (L/2)*dθ, sL = − (L/2)*dθ.
    Equal & opposite motor commands to avoid forward creep.
    """
    dtheta = ((dtheta_rad + math.pi) % (2*math.pi)) - math.pi
    if abs(dtheta) < math.radians(1.0):
        print(f"{label}: skip (|Δθ| < 1°)")
        return

    sR_target =  (AXLE_LEN_M/2.0) * dtheta
    sL_target = -(AXLE_LEN_M/2.0) * dtheta
    dir = 1.0 if dtheta > 0 else -1.0  # +CCW: left back, right fwd

    print(f"{label}: target Δθ={math.degrees(dtheta):+.1f}°, sL={sL_target:+.4f} m, sR={sR_target:+.4f} m")

    l0, r0 = bot.get_encoder_readings()
    bot.set_left_motor_speed( _clamp_pct(-dir * abs(pct)) )
    bot.set_right_motor_speed(_clamp_pct(+dir * abs(pct)) )

    # taper in last 0.02 m of either wheel
    while True:
        sL, sR = enc_wheel_progress_m(bot, l0, r0)
        remL = abs(sL_target) - abs(sL)
        remR = abs(sR_target) - abs(sR)
        rem_min = max(0.0, min(remL, remR))

        scale = 1.0 if rem_min > 0.02 else max(0.35, rem_min / 0.02)
        bot.set_left_motor_speed( _clamp_pct(-dir * abs(pct) * scale) )
        bot.set_right_motor_speed(_clamp_pct(+dir * abs(pct) * scale) )

        if (abs(sL) >= abs(sL_target) - 1e-3) and (abs(sR) >= abs(sR_target) - 1e-3):
            break
        time.sleep(0.005)

    bot.stop_motors()
    print(f"{label}: done (L≈{sL:+.4f} m, R≈{sR:+.4f} m)")

def drive_arc_by_wheel_dists(bot, sL_m, sR_m, base_pct=PCT_ARC, label="arc"):
    """
    Encoder-only arc: command wheels with proportional speeds so they finish together.
    """
    signL = 1.0 if sL_m >= 0 else -1.0
    signR = 1.0 if sR_m >= 0 else -1.0
    aL, aR = abs(sL_m), abs(sR_m)
    if aL < 1e-6 and aR < 1e-6:
        print(f"{label}: degenerate, skip"); return

    smax = max(aL, aR)
    pctL = _clamp_pct(base_pct * (aL / smax)) * signL
    pctR = _clamp_pct(base_pct * (aR / smax)) * signR

    l0, r0 = bot.get_encoder_readings()
    print(f"{label}: targets L={sL_m:.4f} m, R={sR_m:.4f} m | power L={pctL:.1f}%, R={pctR:.1f}%")
    bot.set_left_motor_speed(pctL)
    bot.set_right_motor_speed(pctR)

    # end ramp in last 0.10 m
    while True:
        sL, sR = enc_wheel_progress_m(bot, l0, r0)
        remL = aL - abs(sL)
        remR = aR - abs(sR)
        rem_min = max(0.0, min(remL, remR))
        scale = 1.0 if rem_min > 0.10 else max(0.30, rem_min / 0.10)

        bot.set_left_motor_speed( _clamp_pct(pctL * scale) )
        bot.set_right_motor_speed(_clamp_pct(pctR * scale) )

        if (abs(sL) >= aL - 1e-3) and (abs(sR) >= aR - 1e-3):
            break
        time.sleep(0.01)

    bot.stop_motors()
    print(f"{label}: done (L≈{abs(sL):.4f} m, R≈{abs(sR):.4f} m)")

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

def p4_to_p5(bot):
    """
    EXACT sequence with encoders only (no IMU):
      +45° left (π/4) -> straight 0.7071 ft -> +45° left (π/4).
    """
    # 1) +45° CCW pre-turn (encoder-based spin)
    print("\nP4→P5 pre-turn: +45° CCW")
    turn_in_place_by_angle(bot, dtheta_rad=math.pi/4, pct=PCT_TURN, label="P4→P5 pre-turn")

    # brief settle
    bot.stop_motors(); time.sleep(0.12)

    # 2) straight along the diagonal: d = sqrt(0.5) ft
    d_ft = math.sqrt(0.5)
    print(f"P4→P5 straight: {d_ft:.4f} ft (0.7071 ft expected)")
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P4→P5 straight")

    # brief settle
    bot.stop_motors(); time.sleep(0.12)

    # 3) +45° CCW post-turn to face +x
    print("P4→P5 post-turn: +45° CCW to face +x")
    turn_in_place_by_angle(bot, dtheta_rad=math.pi/4, pct=PCT_TURN, label="P4→P5 post-turn")

def p5_to_p6(bot):
    """
    Encoder-only P5→P6:
      Turn to match target heading, then straight distance from WPTS.
    """
    x1, y1, th1 = WPTS[5]   # P5
    x2, y2, th2 = WPTS[6]   # P6

    # --- Step 1: Heading change ---
    dtheta = th2 - th1
    dtheta = ((dtheta + math.pi) % (2*math.pi)) - math.pi  # normalize to [-π, π]
    print(f"\nP5→P6 turn: Δθ={math.degrees(dtheta):+.1f}°")
    turn_in_place_by_angle(bot, dtheta_rad=dtheta, pct=PCT_TURN, label="P5→P6 turn")

    bot.stop_motors(); time.sleep(0.12)

    # --- Step 2: Straight distance ---
    d_ft = math.hypot(x2 - x1, y2 - y1)
    print(f"P5→P6 straight: {d_ft:.4f} ft")
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P5→P6 straight")

    bot.stop_motors()
    print(f"P5→P6: done; now at {WPTS[6]}")


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
    turn_in_place_by_angle(bot, dtheta_rad=dtheta, pct=PCT_TURN, label="P6→P7 turn")

    bot.stop_motors(); time.sleep(0.12)

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
    turn_in_place_by_angle(bot, dtheta_rad=dtheta, pct=PCT_TURN, label="P7→P8 turn")

    bot.stop_motors(); time.sleep(0.12)

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
    turn_in_place_by_angle(bot, dtheta_rad=dtheta, pct=PCT_TURN, label="P8→P9 turn")

    bot.stop_motors(); time.sleep(0.12)

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
    turn_in_place_by_angle(bot, dtheta_rad=dtheta, pct=PCT_TURN, label="P9→P10 turn")

    bot.stop_motors(); time.sleep(0.12)

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
    turn_in_place_by_angle(bot, dtheta_rad=dtheta1, pct=PCT_TURN, label="P10→P11 pre-turn")
    bot.stop_motors(); time.sleep(0.12)

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

    bot.stop_motors(); time.sleep(0.10)

    # (Optional) tiny final trim to th2 if you want:
    # dtheta2 = ((th2 - 0.0 + math.pi) % (2*math.pi)) - math.pi  # expected 0
    # turn_in_place_by_angle(bot, dtheta_rad=dtheta2, pct=PCT_TURN*0.8, label="P10→P11 final trim")

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
        turn_in_place_by_angle(bot, dtheta_rad=dtheta, pct=PCT_TURN, label="P11→P12 turn")
        bot.stop_motors(); time.sleep(0.12)

    # Straight distance from waypoints
    d_ft = math.hypot(x2 - x1, y2 - y1)  # should be 2.5 ft
    print(f"P11→P12 straight: {d_ft:.4f} ft along +x")
    drive_straight_feet(bot, d_ft, pct=PCT_STRAIGHT, label="P11→P12 straight")

    bot.stop_motors()
    print(f"P11→P12: done; now at {WPTS[12]}")

# ----------------- Main -----------------
def main():
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    try:
        # print("\n=== P0 → P1 (straight) ===")
        # p0_to_p1(bot)

        # print("\n=== P1 → P2 (quarter right arc) ===")
        # p1_to_p2_arc(bot)

        # print("\n=== P2 → P3 (straight 1.0 ft up) ===")
        # p2_to_p3(bot)

        # print("\n=== P3 → P4 (half-circle right arc) ===")
        # p3_to_p4_arc(bot)

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

        print("\nDone.")
    finally:
        bot.stop_motors()
        try:
            bot.disconnect_robot()
        except Exception:
            pass

if __name__ == "__main__":
    main()
