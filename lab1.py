# ------------------------------------------------------------
# Lab 1 – Waypoints (feet in), execute P0→P1 safely and slowly
# Author: Asadbek Nematov
# Robot: Physical HamBot (Encoders + IMU only, no LiDAR/Camera)
#
# Run:
#   python AsadbekNematov_Lab1_P0toP1.py
# ------------------------------------------------------------

import sys, os, time, math
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from robot_systems.robot import HamBot

# ----------------- Geometry & limits (robot constants) -----------------
R_WHEEL_M   = 0.045   # wheel radius [m] (90 mm diameter)
AXLE_LEN_M  = 0.184   # track width [m]
PCT_SAFE    = 22.0    # percent power (gentle)
ENC_DEGREES = False   # your encoders are in radians (from quick_check)
ENC_SIGN_L  = +1.0    # encoder sign (leave +1.0 unless your quick_check showed opposite)
ENC_SIGN_R  = +1.0
FT2M        = 0.3048  # feet to meters

def _clamp_pct(x: float) -> float:
    """Clamp motor percent command to [-100, 100]."""
    return max(-100.0, min(100.0, x))

def wrap_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

# ----------------- Waypoints (feet, radians) -----------------
# NOTE: We store (x_ft, y_ft, theta_rad) exactly as given.
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
    # P13 = (x, y, θ)  # keep for later
]

# ----------------- Helpers -----------------
def stop(bot: HamBot) -> None:
    bot.stop_motors()

def get_heading_deg(bot: HamBot) -> float:
    return bot.get_heading()
 
def enc_distance_m(l_now: float, r_now: float, l0: float, r0: float) -> float:
    """Linear distance [m] from encoder deltas, with sign and unit handling."""
    dl = ENC_SIGN_L * (l_now - l0)   # wheel angle delta (radians)
    dr = ENC_SIGN_R * (r_now - r0)   # wheel angle delta (radians)
    if ENC_DEGREES:                  # (not expected, but supported)
        dl = math.radians(dl)
        dr = math.radians(dr)
    return R_WHEEL_M * 0.5 * (dl + dr)  # average wheel arc length

def drive_straight_feet(bot: HamBot, dist_ft: float, pct: float = PCT_SAFE,
                        kp_heading: float = 1.6, label: str = "straight(ft)") -> None:
    """
    Drive straight for given FEET using:
      - IMU heading hold (P-only)
      - gentle end ramp to avoid overshoot
    """
    dist_m = dist_ft * FT2M
    print(f"{label}: target {dist_ft:.3f} ft ({dist_m:.3f} m) at {pct:.1f}%")

    yaw0 = bot.get_heading() or 0.0
    l0, r0 = bot.get_encoder_readings()

    base = +pct if dist_m >= 0 else -pct
    traveled = 0.0

    while True:
        # heading error (radians), wrap to [-pi, pi]
        yaw = bot.get_heading()
        if yaw is None:
            yaw = yaw0
        e = math.radians(((yaw0 - yaw + 180) % 360) - 180)

        # tiny differential to keep it straight (~0.1% per deg, clamp ±8%)
        turn_pct = max(-8.0, min(8.0, math.degrees(e) * 0.10))

        # distance traveled and a short linear ramp in last 0.25 m
        l, r = bot.get_encoder_readings()
        s = enc_distance_m(l, r, l0, r0)
        traveled = s if dist_m >= 0 else -s
        remain = max(0.0, abs(dist_m) - traveled)
        scale = 1.0 if remain > 0.25 else max(0.28, remain / 0.25)

        left_cmd  = _clamp_pct((base - turn_pct) * scale)
        right_cmd = _clamp_pct((base + turn_pct) * scale)
        bot.set_left_motor_speed(left_cmd)
        bot.set_right_motor_speed(right_cmd)

        if traveled >= abs(dist_m) - 1e-3:
            break
        time.sleep(0.01)

    bot.stop_motors()
    print(f"{label}: done (meas ≈ {traveled:.3f} m)")

def turn_in_place(bot: HamBot, dtheta_rad: float, pct: float = 18.0, label: str = "turn") -> None:
    """
    In-place rotation using IMU feedback (+ = CCW, − = CW), percent power.
    Kept here for next segment; not used in P0→P1 straight.
    """
    if abs(dtheta_rad) < 1e-6:
        print(f"{label}: skip (|Δθ| small)")
        return

    pct = abs(pct)
    if dtheta_rad > 0:    # CCW
        bot.set_left_motor_speed(_clamp_pct(-pct))
        bot.set_right_motor_speed(_clamp_pct(+pct))
    else:                 # CW
        bot.set_left_motor_speed(_clamp_pct(+pct))
        bot.set_right_motor_speed(_clamp_pct(-pct))

    h0 = get_heading_deg(bot) or 0.0
    last = math.radians(h0)
    acc = 0.0
    while True:
        h = get_heading_deg(bot)
        if h is None:
            time.sleep(0.01)
            continue
        rad = math.radians(h)
        acc += wrap_pi(rad - last)
        last = rad
        if abs(acc) >= abs(dtheta_rad):
            break
        time.sleep(0.005)

    stop(bot)
    print(f"{label}: target {dtheta_rad:+.3f} rad, meas ≈ {acc:+.3f} rad")

# ----------------- Main -----------------
def main():
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    try:
        print("\n=== P0 → P1 (feet, slow, IMU hold) ===")
        x1_ft, y1_ft, _ = WPTS[0]
        x2_ft, y2_ft, _ = WPTS[1]
        d_ft = math.hypot(x2_ft - x1_ft, y2_ft - y1_ft)  # == 3.5 ft
        drive_straight_feet(bot, d_ft, pct=PCT_SAFE, kp_heading=1.6, label="P0→P1")
    finally:
        stop(bot)
        try:
            bot.disconnect_robot()
        except Exception:
            pass

if __name__ == "__main__":
    main()
