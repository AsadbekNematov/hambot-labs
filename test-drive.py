import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))

from robot_systems.robot import HamBot
import time

# Initialize HamBot with only motors + IMU (disable sensors for first test)
bot = HamBot(lidar_enabled=False, camera_enabled=False)

print("Starting forward drive for 2 seconds...")

# Convention: left motor = negative speed, right motor = positive speed
bot.set_left_motor_speed(-50)   # left reversed
bot.set_right_motor_speed(50)   # right forward

time.sleep(2)  # drive forward for 2 seconds

print("Stopping motors.")
bot.stop_motors()
