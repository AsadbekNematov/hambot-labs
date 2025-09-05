# AsadbekNematov_Lab1.py
# Physical HamBot — Lab 1 Waypoint Navigation
# Encoders + IMU only; no LiDAR is allowed in this lab.

import sys, os
# Extend Python path to include "src" directory so that robot library can be imported
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))

import math, time, argparse
from robot_systems.robot import HamBot   # Main robot control class for HamBot

# ---- HamBot geometry / motor limits (from physical spec sheet) ----
R_WHEEL = 0.045   # [m] radius of each wheel
L_AXLE  = 0.184   # [m] distance between left and right wheels (axle length)
RPM_MAX = 75.0    # maximum allowed wheel speed in revolutions per minute

# Convert specs into useful angular and linear velocities
OMEGA_MAX = RPM_MAX * 2*math.pi/60.0    # [rad/s] max angular wheel velocity
V_MAX     = OMEGA_MAX * R_WHEEL         # [m/s] max straight-line velocity
YAW_MAX   = 2*R_WHEEL*OMEGA_MAX / L_AXLE  # [rad/s] max yaw rate when spinning in place

# Wrap any angle into [-π, π]
def wrap_pi(a): 
    return (a + math.pi) % (2*math.pi) - math.pi

# ---- Waypoints (x, y, θ) in meters and radians ----
# Each point is a target pose: (x position, y position, heading angle)
P = [
    ( 2.0, -2.0,  math.pi/2),    # P0
    ( 2.0, -0.5,  math.pi/2),    # P1
    ( 1.0, -0.5,  3*math.pi/2),  # P2
    (-2.0, -0.5,  math.pi/2),    # P3
    (-2.0,  2.0,  math.pi/2),    # P4
    (-2.0,  2.0,  0.0),          # P5 (pure rotate only)
    ( 1.5,  2.0,  0.0),          # P6
    ( 1.5,  2.0,  7*math.pi/4),  # P7 (rotate only)
    ( 2.0,  1.5,  7*math.pi/4),  # P8
    ( 2.0,  1.5,  5*math.pi/4),  # P9 (rotate only)
    ( 1.5,  1.0,  5*math.pi/4),  # P10
    ( 1.5,  1.0,  math.pi),      # P11 (rotate only)
    ( 0.0,  1.0,  math.pi),      # P12
]

# ---------- Motion Planning ----------
def plan_segments(P):
    """
    Generate a kinematic motion plan between waypoints.
    Each segment can be either:
      - a straight leg with pre-turn and post-turn
      - or a turn-only move if distance between waypoints is zero.
    Also estimates execution times at saturated speeds.
    """
    plan = []
    total_len = 0.0
    straight_time = 0.0
    turn_time = 0.0

    for k in range(len(P)-1):
        x1,y1,th1 = P[k]
        x2,y2,th2 = P[k+1]
        dx,dy = x2-x1, y2-y1
        dist = math.hypot(dx,dy)  # straight-line distance

        if dist < 1e-9:
            # Pure turn (no translation)
            dtheta = wrap_pi(th2 - th1)
            t_turn = abs(dtheta)/YAW_MAX
            plan.append(dict(kind="turn_only", k=k, dtheta=dtheta, t_turn=t_turn))
            turn_time += t_turn
        else:
            # Segment with translation
            h = math.atan2(dy,dx)             # desired heading for this leg
            pre  = wrap_pi(h - th1)           # pre-rotation needed
            post = wrap_pi(th2 - h)           # post-rotation at arrival
            t_pre  = abs(pre)/YAW_MAX
            t_post = abs(post)/YAW_MAX
            t_lin  = dist / V_MAX             # straight-line travel time

            plan.append(dict(kind="leg", k=k, pre=pre, dist=dist, post=post,
                             t_pre=t_pre, t_straight=t_lin, t_post=t_post))
            total_len    += dist
            straight_time += t_lin
            turn_time    += (t_pre + t_post)

    return plan, total_len, straight_time, turn_time, straight_time + turn_time

# ---------- Low-level motor commands for physical HamBot ----------
def set_forward_rpm(bot, rpm):
    """
    Command both wheels forward at same RPM.
    Note: convention is left motor negative, right motor positive.
    """
    bot.set_left_motor_speed(+rpm)
    bot.set_right_motor_speed(+rpm)

def stop(bot):
    """Stop both motors safely."""
    bot.stop_motors()

def imu_heading_deg(bot):
    """
    Return current robot heading from IMU (in degrees).
    IMU returns heading relative to East, range [0, 360).
    """
    return bot.get_heading()

def turn_in_place(bot, dtheta_rad, label="turn"):
    """
    Execute an in-place rotation using IMU feedback.
    - Positive dtheta = CCW
    - Negative dtheta = CW
    Uses maximum RPM until the accumulated IMU angle matches target.
    """
    if abs(dtheta_rad) < 1e-6: return
    print(f"{label}: target dθ={dtheta_rad:+.3f} rad")

    rpm = RPM_MAX
    sgn = 1.0 if dtheta_rad >= 0 else -1.0

    # Apply opposite wheel directions to spin in place
    bot.set_left_motor_speed(+sgn*rpm)
    bot.set_right_motor_speed(+sgn*-rpm)

    # Integrate IMU heading until desired rotation is achieved
    h0 = math.radians(imu_heading_deg(bot) or 0.0)
    last = h0
    acc = 0.0
    t0 = time.time()

    while True:
        h = imu_heading_deg(bot)
        if h is None:   # IMU might fail momentarily
            time.sleep(0.01); continue
        h = math.radians(h)
        dh = wrap_pi(h - last)   # small angle increment
        acc += dh                # accumulate rotation
        last = h
        if abs(acc) >= abs(dtheta_rad):
            break
        time.sleep(0.005)

    stop(bot)
    print(f"{label}: done in {time.time()-t0:.3f}s (meas dθ≈{acc:+.3f} rad)")

def drive_straight(bot, distance_m, label="straight"):
    """
    Drive straight for a given distance using encoders.
    Uses average wheel displacement to estimate traveled distance.
    """
    if abs(distance_m) < 1e-6: return
    print(f"{label}: target d={distance_m:.3f} m")

    # Start moving forward at maximum RPM
    set_forward_rpm(bot, RPM_MAX)

    # Record starting encoder readings
    l0, r0 = bot.get_encoder_readings()
    t0 = time.time()

    while True:
        l, r = bot.get_encoder_readings()
        # Distance traveled = wheel radius * average encoder rotation
        s = R_WHEEL * (abs(l - l0) + abs(r - r0)) * 0.5
        if s >= abs(distance_m):
            break
        time.sleep(0.005)

    stop(bot)
    print(f"{label}: done in {time.time()-t0:.3f}s (meas d≈{s:.3f} m)")

# ---------- Main control loop ----------
def main():
    # Command-line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", action="store_true", help="print kinematics only (no execution)")
    ap.add_argument("--run",  action="store_true", help="execute on physical HamBot")
    args = ap.parse_args()
    if not (args.plan or args.run):
        args.plan = True   # default to just planning

    # Generate plan
    plan, total_len, t_lin, t_turn, t_tot = plan_segments(P)

    # Print planning summary
    print("\n=== Lab 1 – Plan at saturated speeds ===")
    print(f"r={R_WHEEL} m, L={L_AXLE} m, ω_max={OMEGA_MAX:.4f} rad/s, v_max={V_MAX:.4f} m/s, yaw_max={YAW_MAX:.4f} rad/s")
    for item in plan:
        if item["kind"] == "leg":
            k = item["k"]
            print(f"P{k}→P{k+1}: pre={item['pre']:+.3f} rad | dist={item['dist']:.3f} m | post={item['post']:+.3f} rad"
                  f" | t_pre={item['t_pre']:.3f}s t_lin={item['t_straight']:.3f}s t_post={item['t_post']:.3f}s")
        else:
            print(f"P{item['k']}→P{item['k']+1}: TURN ONLY dθ={item['dtheta']:+.3f} rad | t_turn={item['t_turn']:.3f}s")

    print(f"\nTotal straight length = {total_len:.3f} m")
    print(f"Straight time = {t_lin:.3f} s | Turn time = {t_turn:.3f} s | Total ≈ {t_tot:.3f} s\n")

    if args.plan and not args.run:
        # Only printing plan, no robot execution
        return

    # Otherwise, execute plan on physical robot
    print("=== RUN on physical HamBot (encoders + IMU) ===")
    bot = HamBot(lidar_enabled=False, camera_enabled=False)  # disable unused sensors

    try:
        for item in plan:
            k = item["k"]
            if item["kind"] == "leg":
                # Pre-turn to align with segment
                if abs(item["pre"]) > 1e-6:
                    turn_in_place(bot, item["pre"], label=f"P{k} pre-turn")
                # Drive straight for the distance
                drive_straight(bot, item["dist"], label=f"P{k} straight")
                # Post-turn to match final desired heading
                if abs(item["post"]) > 1e-6:
                    turn_in_place(bot, item["post"], label=f"P{k} post-turn")
            else:
                # Pure turn-only move
                turn_in_place(bot, item["dtheta"], label=f"P{k} in-place")

        stop(bot)
        print("Done.")
    finally:
        # Ensure robot shuts down motors and communications safely
        bot.disconnect_robot()

if __name__ == "__main__":
    main()
