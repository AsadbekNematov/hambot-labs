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
R_WHEEL_M   = 0.045             # wheel radius [m]
AXLE_LEN_M  = 0.184             # track width [m] (not needed for P0→P1)
RPM_MAX     = 75.0              # datasheet
RPM_SAFE    = 25.0              # << go slow first
ENC_DEGREES = True              # << set True if encoders report degrees

FT2M = 0.3048

def wrap_pi(a): 
    return (a + math.pi) % (2.0*math.pi) - math.pi

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
def stop(bot): 
    bot.stop_motors()

def get_heading_deg(bot): 
    return bot.get_heading()

def enc_distance_m(l_now, r_now, l0, r0):
    """Compute linear distance traveled [m] from encoder deltas."""
    if ENC_DEGREES:
        dl_rad = math.radians(l_now - l0)
        dr_rad = math.radians(r_now - r0)
    else:
        dl_rad = (l_now - l0)
        dr_rad = (r_now - r0)
    # average wheel arc length
    return R_WHEEL_M * 0.5 * (dl_rad + dr_rad)

def drive_straight_feet(bot, dist_ft, rpm=RPM_SAFE, label="straight(ft)"):
    """Drive straight for given distance in FEET (positive forward)."""
    dist_m = dist_ft * FT2M
    print(f"{label}: target {dist_ft:.3f} ft ({dist_m:.3f} m) at {rpm:.1f} RPM")
    if dist_m < 0:
        # reverse
        bot.set_left_motor_speed(-rpm)
        bot.set_right_motor_speed(-rpm)
    else:
        bot.set_left_motor_speed(+rpm)
        bot.set_right_motor_speed(+rpm)

    l0, r0 = bot.get_encoder_readings()  # assumes per-wheel angles
    traveled = 0.0
    while True:
        l, r = bot.get_encoder_readings()
        s = enc_distance_m(l, r, l0, r0)
        traveled = s if dist_m >= 0 else -s
        if traveled >= abs(dist_m) - 1e-3:
            break
        time.sleep(0.01)

    stop(bot)
    print(f"{label}: done (meas ≈ {traveled:.3f} m)")

# (Keeping a simple IMU-based in-place turn for later segments)
def turn_in_place(bot, dtheta_rad, rpm=RPM_SAFE, label="turn"):
    """In-place rotation using IMU feedback (+ = CCW, − = CW)."""
    if abs(dtheta_rad) < 1e-6: 
        print(f"{label}: skip (|Δθ| small)")
        return

    # set opposite wheel directions for spin turn
    if dtheta_rad > 0:    # CCW
        bot.set_left_motor_speed(-rpm)
        bot.set_right_motor_speed(+rpm)
    else:                 # CW
        bot.set_left_motor_speed(+rpm)
        bot.set_right_motor_speed(-rpm)

    # integrate IMU heading until Δθ reached
    h0 = get_heading_deg(bot) or 0.0
    last = math.radians(h0)
    acc = 0.0
    while True:
        h = get_heading_deg(bot)
        if h is None: 
            time.sleep(0.01); 
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
        # -------- Segment P0 → P1 (ignore θ for now) --------
        print("\n=== P0 → P1 (feet, slow) ===")
        x1_ft, y1_ft, th1 = WPTS[0]
        x2_ft, y2_ft, th2 = WPTS[1]

        d_ft = math.hypot(x2_ft - x1_ft, y2_ft - y1_ft)  # in feet
        # your example: from -1 to 0.5 → 1.5 ft — same idea here on 2D
        drive_straight_feet(bot, d_ft, rpm=RPM_SAFE, label="P0→P1")

        print("\nP0→P1 complete at safe speed.")
        # For later: pre-turn to segment heading, post-turn to th2 if needed.
        # seg_heading = math.atan2((y2_ft - y1_ft), (x2_ft - x1_ft))
        # turn_in_place(bot, wrap_pi(seg_heading - current_yaw_rad))
        # drive_straight_feet(bot, d_ft)
        # turn_in_place(bot, wrap_pi(th2 - seg_heading))

    finally:
        stop(bot)
        try:
            bot.disconnect_robot()
        except Exception:
            pass

if __name__ == "__main__":
    main()
