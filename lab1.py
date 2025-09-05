# AsadbekNematov_Lab1.py
# Physical HamBot — Lab 1 Waypoint Navigation (Encoders + IMU only; no LiDAR)

import math, time, argparse
from robot_systems.robot import HamBot

# ---- HamBot geometry / limits (from your spec) ----
R_WHEEL = 0.045   # m  (radius)
L_AXLE  = 0.184   # m  (wheel spacing)
RPM_MAX = 75.0
OMEGA_MAX = RPM_MAX * 2*math.pi/60.0    # rad/s per wheel
V_MAX     = OMEGA_MAX * R_WHEEL         # m/s straight
YAW_MAX   = 2*R_WHEEL*OMEGA_MAX / L_AXLE  # rad/s in-place

def wrap_pi(a): return (a + math.pi)%(2*math.pi) - math.pi

# ---- Waypoints (meters, radians) P0..P12 ----
P = [
    ( 2.0, -2.0,  math.pi/2),    # P0
    ( 2.0, -0.5,  math.pi/2),    # P1
    ( 1.0, -0.5,  3*math.pi/2),  # P2
    (-2.0, -0.5,  math.pi/2),    # P3
    (-2.0,  2.0,  math.pi/2),    # P4
    (-2.0,  2.0,  0.0),          # P5 (pure rotate)
    ( 1.5,  2.0,  0.0),          # P6
    ( 1.5,  2.0,  7*math.pi/4),  # P7 (pure rotate)
    ( 2.0,  1.5,  7*math.pi/4),  # P8
    ( 2.0,  1.5,  5*math.pi/4),  # P9 (pure rotate)
    ( 1.5,  1.0,  5*math.pi/4),  # P10
    ( 1.5,  1.0,  math.pi),      # P11 (pure rotate)
    ( 0.0,  1.0,  math.pi),      # P12
]

# ---------- Planning ----------
def plan_segments(P):
    plan = []
    total_len = 0.0
    straight_time = 0.0
    turn_time = 0.0

    for k in range(len(P)-1):
        x1,y1,th1 = P[k]
        x2,y2,th2 = P[k+1]
        dx,dy = x2-x1, y2-y1
        dist = math.hypot(dx,dy)

        if dist < 1e-9:
            dtheta = wrap_pi(th2 - th1)
            t_turn = abs(dtheta)/YAW_MAX
            plan.append(dict(kind="turn_only", k=k, dtheta=dtheta, t_turn=t_turn))
            turn_time += t_turn
        else:
            h = math.atan2(dy,dx)            # heading of the segment
            pre  = wrap_pi(h - th1)          # rotate to face segment
            post = wrap_pi(th2 - h)          # rotate to final heading
            t_pre  = abs(pre)/YAW_MAX
            t_post = abs(post)/YAW_MAX
            t_lin  = dist / V_MAX
            plan.append(dict(kind="leg", k=k, pre=pre, dist=dist, post=post,
                             t_pre=t_pre, t_straight=t_lin, t_post=t_post))
            total_len    += dist
            straight_time += t_lin
            turn_time    += (t_pre + t_post)
    return plan, total_len, straight_time, turn_time, straight_time + turn_time

# ---------- Low-level on phyasadsical HamBot ----------
def set_forward_rpm(bot, rpm):
    # Forward: left negative, right positive
    bot.set_left_motor_speed(-rpm)
    bot.set_right_motor_speed(+rpm)

def stop(bot):
    bot.stop_motors()

def imu_heading_deg(bot):
    # bot.get_heading() already returns degrees-from-East (0..360) via IMU
    return bot.get_heading()

def turn_in_place(bot, dtheta_rad, label="turn"):
    if abs(dtheta_rad) < 1e-6: return
    print(f"{label}: target dθ={dtheta_rad:+.3f} rad")
    # Saturated in-place turn: ±75 RPM on wheels (opposite directions)
    rpm = RPM_MAX
    sgn = 1.0 if dtheta_rad >= 0 else -1.0
    bot.set_left_motor_speed(+sgn*rpm)   # note IMU positive = CCW turn; left runs forward for CW?
    bot.set_right_motor_speed(+sgn*-rpm) # this pair yields CCW for sgn=+1 given motor conventions
    # Integrate heading change
    h0 = math.radians(imu_heading_deg(bot) or 0.0)
    last = h0
    acc = 0.0
    t0 = time.time()
    while True:
        h = imu_heading_deg(bot)
        if h is None:
            time.sleep(0.01); continue
        h = math.radians(h)
        dh = wrap_pi(h - last)
        acc += dh
        last = h
        if abs(acc) >= abs(dtheta_rad):
            break
        time.sleep(0.005)
    stop(bot)
    print(f"{label}: done in {time.time()-t0:.3f}s (meas dθ≈{acc:+.3f} rad)")

def drive_straight(bot, distance_m, label="straight"):
    if abs(distance_m) < 1e-6: return
    print(f"{label}: target d={distance_m:.3f} m")
    # Start at max straight RPM
    set_forward_rpm(bot, RPM_MAX)
    # Read starting encoder radians (accumulated, asad from background thread)
    l0, r0 = bot.get_encoder_readings()
    t0 = time.time()
    while True:
        l, r = bot.get_encoder_readings()
        # disasadtance estimate: s = r * (Δθ_l + Δθ_r)/2
        s = R_WHEEL * (abs(l - l0) + abs(r - r0)) * 0.5
        if s >= abs(distance_m):
            break
        time.sleep(0.005)
    stop(bot)
    print(f"{label}: done in {time.time()-t0:.3f}s (meas d≈{s:.3f} m)")

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", action="store_true", help="print kinematics only")
    ap.add_argument("--run",  action="store_true", help="execute on physical HamBot")
    args = ap.parse_args()
    if not (args.plan or args.run):
        args.plan = True

    plan, total_len, t_lin, t_turn, t_tot = plan_segments(P)

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
        return

    print("=== RUN on physical HamBot (encoders + IMU) ===")
    bot = HamBot(lidar_enabled=False, camera_enabled=False)

    try:
        for item in plan:
            k = item["k"]
            if item["kind"] == "leg":
                if abs(item["pre"]) > 1e-6:
                    turn_in_place(bot, item["pre"], label=f"P{k} pre-turn")
                drive_straight(bot, item["dist"], label=f"P{k} straight")
                if abs(item["post"]) > 1e-6:
                    turn_in_place(bot, item["post"], label=f"P{k} post-turn")
            else:
                turn_in_place(bot, item["dtheta"], label=f"P{k} in-place")
        stop(bot)
        print("Done.")
    finally:
        # Ensure safe shutdown
        bot.disconnect_robot()

if __name__ == "__main__":
    main()
