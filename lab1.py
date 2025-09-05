# ------------------------------------------------------------
# Lab 1 – Segments P0→P1 and P1→P2 (sequential)
# Author: Asadbek Nematov
# Robot: Physical HamBot (Encoders + IMU only, no LiDAR/Camera)
#
# Usage:
#   python AsadbekNematov_Lab1_P0toP2.py
# ------------------------------------------------------------

import sys, os, time, math
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from robot_systems.robot import HamBot

# ---- Geometry & limits ----
R_WHEEL = 0.045
RPM_MAX = 75.0

def wrap_pi(a): return (a + math.pi) % (2.0*math.pi) - math.pi

# ---- Waypoints ----
P = [
    ( 2.0, -2.0,  math.pi/2),    # P0
    ( 2.0, -0.5,  math.pi/2),    # P1
    ( 1.0, -0.5,  3*math.pi/2),  # P2
    # (other waypoints unused for now)
]

# ---- Helpers ----
def stop(bot): bot.stop_motors()
def get_heading_deg(bot): return bot.get_heading()

def turn_in_place(bot, dtheta_rad, rpm=75.0, label="turn"):
    """In-place rotation using IMU feedback (+ = CCW, − = CW)."""
    if abs(dtheta_rad) < 1e-6: return
    if dtheta_rad > 0:   # CCW
        bot.set_left_motor_speed(-rpm)
        bot.set_right_motor_speed(+rpm)
    else:                # CW
        bot.set_left_motor_speed(+rpm)
        bot.set_right_motor_speed(-rpm)

    # integrate IMU heading until Δθ reached
    h0 = get_heading_deg(bot) or 0.0
    last = math.radians(h0)
    acc = 0.0
    while True:
        h = get_heading_deg(bot)
        if h is None: time.sleep(0.01); continue
        rad = math.radians(h)
        acc += wrap_pi(rad - last)
        last = rad
        if abs(acc) >= abs(dtheta_rad): break
        time.sleep(0.005)
    stop(bot)
    print(f"{label}: target {dtheta_rad:+.3f} rad, meas ≈ {acc:+.3f} rad")

def drive_straight(bot, dist_m, rpm=75.0, label="straight"):
    """Encoder-based straight drive forward for given distance (m)."""
    print(f"{label}: target {dist_m:.3f} m")
    bot.set_left_motor_speed(rpm)
    bot.set_right_motor_speed(rpm)
    l0, r0 = bot.get_encoder_readings()
    while True:
        l, r = bot.get_encoder_readings()
        s = R_WHEEL * ((l - l0) + (r - r0)) * 0.5
        if s >= dist_m: break
        time.sleep(0.01)
    stop(bot)
    print(f"{label}: done (meas ≈{s:.3f} m)")

# ---- Main ----
def main():
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    try:
        # -------- Segment P0 → P1 --------
        x1,y1,th1 = P[0]
        x2,y2,th2 = P[1]
        dist = math.hypot(x2-x1, y2-y1)   # should be 1.50 m
        print("\n=== P0 → P1 ===")
        drive_straight(bot, dist, rpm=RPM_MAX, label="P0→P1")

        # -------- Segment P1 → P2 --------
        x1,y1,th1 = P[1]
        x2,y2,th2 = P[2]
        seg_heading = math.atan2(y2-y1, x2-x1)   # should be π (west)
        pre  = wrap_pi(seg_heading - th1)        # +90° left
        dist = math.hypot(x2-x1, y2-y1)          # 1.00 m
        post = wrap_pi(th2 - seg_heading)        # −90° right

        print("\n=== P1 → P2 ===")
        if abs(pre) > 1e-3:
            turn_in_place(bot, pre, rpm=RPM_MAX, label="P1 pre-turn")
        drive_straight(bot, dist, rpm=RPM_MAX, label="P1→P2 straight")
        if abs(post) > 1e-3:
            turn_in_place(bot, post, rpm=RPM_MAX, label="P2 post-turn")

        print("\nSegments P0→P1 and P1→P2 complete.")
    finally:
        stop(bot)
        bot.disconnect_robot()

if __name__ == "__main__":
    main()
