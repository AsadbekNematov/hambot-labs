# ---- save as quick_check.py and run: python quick_check.py ----
import sys, os, time, math
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from robot_systems.robot import HamBot

def quick_encoder_sanity(power=20.0, t=1.0):
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    try:
        print(f"\nRunning forward at +{power}% for {t}s ...")
        l0, r0 = bot.get_encoder_readings()
        yaw0 = bot.get_heading() or 0.0
        bot.set_left_motor_speed(+power)
        bot.set_right_motor_speed(+power)
        time.sleep(t)
        bot.stop_motors()
        l1, r1 = bot.get_encoder_readings()
        yaw1 = bot.get_heading() or yaw0

        dL_f = l1 - l0
        dR_f = r1 - r0
        dyaw_f = ((yaw1 - yaw0 + 180) % 360) - 180
        print(f"FORWARD ΔL={dL_f:.2f}, ΔR={dR_f:.2f}, Δyaw={dyaw_f:.2f}°")

        time.sleep(0.5)

        print(f"\nRunning backward at -{power}% for {t}s ...")
        l0, r0 = bot.get_encoder_readings()
        yaw0 = bot.get_heading() or 0.0
        bot.set_left_motor_speed(-power)
        bot.set_right_motor_speed(-power)
        time.sleep(t)
        bot.stop_motors()
        l1, r1 = bot.get_encoder_readings()
        yaw1 = bot.get_heading() or yaw0

        dL_b = l1 - l0
        dR_b = r1 - r0
        dyaw_b = ((yaw1 - yaw0 + 180) % 360) - 180
        print(f"BACKWARD ΔL={dL_b:.2f}, ΔR={dR_b:.2f}, Δyaw={dyaw_b:.2f}°")

    finally:
        bot.stop_motors()
        try: bot.disconnect_robot()
        except: pass

if __name__ == "__main__":
    quick_encoder_sanity(power=20.0, t=1.0)
