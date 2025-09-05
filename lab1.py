 # ------------------------------------------------------------
# Lab 1 – Step 1: Drive from P0 to P1 (straight line only)
# Author: Asadbek Nematov
# Robot: Physical HamBot (encoders + IMU only; no LiDAR or Camera)
#
# Usage:
#   python AsadbekNematov_Lab1_P0toP1.py
# ------------------------------------------------------------

import sys, os, time, math
# Make sure we can import from src/robot_systems/
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from robot_systems.robot import HamBot

# ------------------ Robot geometry constants ------------------
R_WHEEL = 0.045   # wheel radius [m]
L_AXLE  = 0.184   # axle length [m] (not used here but needed for turns)
RPM_MAX = 75.0    # maximum wheel speed [RPM]

def wrap_pi(a):
    """Wrap any angle (rad) into [-π, π] for consistency."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

# ------------------ Waypoints list ------------------
# Each waypoint is (x, y, θ) in meters and radians
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

# ------------------ Motion helper ------------------
def drive_straight(bot, distance_m, rpm=RPM_MAX, label="straight"):
    """
    Drive forward for a given distance (meters) using encoder feedback.

    - Both motors commanded with the same positive RPM (HamBot handles left inversion).
    - Distance is computed as the average wheel travel.
    """
    print(f"{label}: target d={distance_m:.3f} m at {rpm:.1f} RPM")

    # Start both motors forward
    bot.set_left_motor_speed(rpm)
    bot.set_right_motor_speed(rpm)

    # Record starting encoder values (in radians)
    l0, r0 = bot.get_encoder_readings()

    while True:
        l, r = bot.get_encoder_readings()
        # Distance traveled = wheel radius * average wheel rotation
        s = R_WHEEL * ((l - l0) + (r - r0)) * 0.5
        if s >= distance_m:
            break
        time.sleep(0.01)

    # Stop motors once target distance reached
    bot.stop_motors()
    print(f"{label}: done (meas d≈{s:.3f} m)")

# ------------------ Main ------------------
def main():
    # Segment: P0 → P1
    x1, y1, th1 = P[0]
    x2, y2, th2 = P[1]
    # Compute straight-line distance between waypoints
    dist = math.hypot(x2 - x1, y2 - y1)

    bot = HamBot(lidar_enabled=False, camera_enabled=False)  # IMU+motors only
    try:
        print("\n=== Lab 1: Segment P0 → P1 ===")
        print(f"From P0={P[0]} to P1={P[1]}")
        print(f"Planned straight distance = {dist:.3f} m")

        # Execute straight drive
        drive_straight(bot, dist, rpm=RPM_MAX, label="P0→P1")

        print("Segment P0 → P1 complete.")
    finally:
        # Always ensure safe shutdown
        bot.disconnect_robot()

if __name__ == "__main__":
    main()
