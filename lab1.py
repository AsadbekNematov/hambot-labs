# ------------------------------------------------------------
# Lab 1 – Step 2: Execute segment P1 → P2 (pre-turn, straight, post-turn)
# Author: Asadbek Nematov
# Robot: Physical HamBot (Encoders + IMU only)
#
# Usage:
#   python AsadbekNematov_Lab1_P1toP2.py
# ------------------------------------------------------------

import sys, os, time, math
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from robot_systems.robot import HamBot

# -------- Constants (from your spec) --------
R_WHEEL = 0.045   # m
L_AXLE  = 0.184   # m
RPM_MAX = 75.0

def wrap_pi(a):  # -> [-pi, pi]
    return (a + math.pi) % (2.0*math.pi) - math.pi

# -------- Waypoints (x, y, theta) --------
P = [
    ( 2.0, -2.0,  math.pi/2),    # P0
    ( 2.0, -0.5,  math.pi/2),    # P1
    ( 1.0, -0.5,  3*math.pi/2),  # P2
    (-2.0, -0.5,  math.pi/2),    # P3
    (-2.0,  2.0,  math.pi/2),    # P4
    (-2.0,  2.0,  0.0),          # P5
    ( 1.5,  2.0,  0.0),          # P6
    ( 1.5,  2.0,  7*math.pi/4),  # P7
    ( 2.0,  1.5,  7*math.pi/4),  # P8
    ( 2.0,  1.5,  5*math.pi/4),  # P9
    ( 1.5,  1.0,  5*math.pi/4),  # P10
    ( 1.5,  1.0,  math.pi),      # P11
    ( 0.0,  1.0,  math.pi),      # P12
]

# -------- Helpers --------
def stop(bot):
    bot.stop_motors()

def get_heading_deg(bot):
    # IMU heading in degrees from East (0..360)
    return bot.get_heading()

def turn_in_place(bot, dtheta_rad, rpm=75.0, label="turn"):
    """+dθ = CCW, -dθ = CW. Use IMU integration to stop precisely."""
    if abs(dtheta_rad) < 1e-6: return
    sgn = 1.0 if dtheta_rad >= 0 else -1.0
    # CCW: left backward, right forward  (HamBot flips left internally, so just set signs logically)
    if sgn > 0:        # CCW
        bot.set_left_motor_speed(-rpm)
        bot.set_right_motor_speed(+rpm)
    else:              # CW
        bot.set_left_motor_speed(+rpm)
        bot.set_right_motor_speed(-rpm)

    # integrate IMU yaw change
    h0 = get_heading_deg(bot)
    if h0 is None: h0 = 0.0
    last = math.radians(h0)
    acc  = 0.0
    while True:
        h = get_heading_deg(bot)
        if h is None: time.sleep(0.01); continue
        rad = math.radians(h)
        acc += wrap_pi(rad - last)
        last = rad
        if abs(acc) >= abs(dtheta_rad):
            break
        time.sleep(0.005)
    stop(bot)
    print(f"{label}: target {dtheta_rad:+.3f} rad, measured ≈ {acc:+.3f} rad")

def align_to_heading(bot, target_rad, tol_deg=2.0):
    """Rotate robot to target global heading (rad), using coarse+fine passes."""
    def deg_err(a_deg, b_deg):
        return math.degrees(wrap_pi(math.radians(a_deg - b_deg)))

    cur = get_heading_deg(bot)
    if cur is None:
        print("IMU not ready; skip align."); return

    target_deg = math.degrees(target_rad % (2*math.pi))
    err = deg_err(target_deg, cur)

    # coarse
    if abs(err) > tol_deg*2:
        turn_in_place(bot, math.radians(err), rpm=75.0, label="align coarse")
        cur = get_heading_deg(bot) or cur
        err = deg_err(target_deg, cur)
    # fine
    if abs(err) > tol_deg:
        turn_in_place(bot, math.radians(err), rpm=30.0, label="align fine")

def drive_straight(bot, distance_m, rpm=75.0, label="straight"):
    """Encoder-based straight drive forward for a given distance (m)."""
    print(f"{label}: d={distance_m:.3f} m @ {rpm:.0f} RPM")
    bot.set_left_motor_speed(rpm)   # both +RPM = forward (HamBot flips left internally)
    bot.set_right_motor_speed(rpm)
    l0, r0 = bot.get_encoder_readings()
    while True:
        l, r = bot.get_encoder_readings()
        s = R_WHEEL * ((l - l0) + (r - r0)) * 0.5
        if s >= distance_m:
            break
        time.sleep(0.01)
    stop(bot)

# -------- Main: execute exactly P1 → P2 --------
def main():
    # P1 and P2 from the list
    x1, y1, th1 = P[1]
    x2, y2, th2 = P[2]

    # Compute desired segment: face along line P1->P2, drive, then align to th2
    seg_heading = math.atan2(y2 - y1, x2 - x1)     # should be π (west)
    pre = wrap_pi(seg_heading - th1)               # rotate from th1 to path heading
    dist = math.hypot(x2 - x1, y2 - y1)            # should be 1.000 m
    post = wrap_pi(th2 - seg_heading)              # rotate to desired final heading

    print("\n=== P1 → P2 ===")
    print(f"P1={P[1]}, P2={P[2]}")
    print(f"pre-turn={pre:+.3f} rad, dist={dist:.3f} m, post-turn={post:+.3f} rad")

    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    try:
        # 1) Pre-turn (to face west)
        if abs(pre) > 1e-3:
            turn_in_place(bot, pre, rpm=75.0, label="P1 pre-turn")

        # 2) Straight (1.0 m)
        drive_straight(bot, dist, rpm=75.0, label="P1→P2 straight")

        # 3) Post-turn (to face south)
        if abs(post) > 1e-3:
            turn_in_place(bot, post, rpm=75.0, label="P2 post-turn")

        print("P1 → P2 complete.")
    finally:
        stop(bot)
        bot.disconnect_robot()

if __name__ == "__main__":
    main()
