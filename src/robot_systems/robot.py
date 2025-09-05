import math, time, threading, sys, signal
from buildhat import Motor
from robot_systems.imu import IMU
from robot_systems.lidar import Lidar
# from robot_systems.camera import Camera

class HamBot:
    def __init__(self, lidar_enabled=True, camera_enabled=True):
        # Initializes IMU
        self.imu = IMU(poll_hz=20.0)
        self.imu.start()

        # Initializes Motors and Encoders
        self.left_motor = Motor('B')
        self.left_motor.set_speed_unit_rpm(rpm=True)

        self.right_motor = Motor('A')
        self.right_motor.set_speed_unit_rpm(rpm=True)

        # Initialize rotation tracking
        self.left_motor_radians = 0.0
        self.last_left_position = self.left_motor.get_position()

        self.right_motor_radians = 0.0
        self.last_right_position = self.right_motor.get_position()

        # Initializes Lidar
        if lidar_enabled:
            self.lidar = Lidar()
        else :
            self.lidar = None

        # if camera_enabled:
        #     self.camera = Camera()
        # else:
        #     self.camera = None

        # Start the thread to update motor positions
        self.stop_thread = False
        self.position_thread = threading.Thread(target=self.update_motor_positions)
        self.position_thread.start()

        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.shutdown)

    def get_range_image(self):
        """
        Retrieve the current range image from the Lidar.

        Returns:
            list: A list of 360 distance measurements corresponding to each degree,
                  where 0° is towards the back of the Lidar, 90° is to the left,
                  180° is to the front, and 270° is to the right.
                  Returns -1 if the Lidar is not enabled.

        This function checks if the Lidar is enabled and then retrieves the latest
        scan data, which represents the distance measurements at each degree of rotation.
        If the Lidar is not enabled, it prints an error message and returns -1.
        """
        if self.lidar is not None:
            return self.lidar.get_current_scan()
        else:
            print("Lidar is not enabled.")
            return -1

    def get_heading(self, fresh_within=0.5, blocking=False, wait_timeout=0.3):
        return self.imu.get_heading(fresh_within=fresh_within,
                                    blocking=blocking,
                                    wait_timeout=wait_timeout)
    # def get_heading(self):
    #     """
    #     Retrieve the current heading of the robot in degrees.
    #
    #     Returns:
    #         float: The heading of the robot relative to East (0° to 360°).
    #
    #     This function fetches the current heading from the IMU, which indicates
    #     the direction the robot is facing. The heading is adjusted to reflect the
    #     orientation relative to East, with 0° representing East, 90° representing North,
    #     180° representing West, and 270° representing South.
    #     """
    #     return self.imu.get_heading()

    def update_motor_positions(self):
        # TODO flip right motor backwards
        """Threaded method to update the motor positions continuously."""
        while not self.stop_thread:
            # Update left motor
            current_left_position = self.left_motor.get_position()
            delta_left_degrees = current_left_position - self.last_left_position

            # Handle wrap-around for the left motor
            if delta_left_degrees > 180:
                delta_left_degrees -= 360
            elif delta_left_degrees < -180:
                delta_left_degrees += 360

            # Adjust the accumulation by inverting the direction for the left motor
            self.left_motor_radians -= math.radians(delta_left_degrees)  # Negate to account for motor configuration
            self.last_left_position = current_left_position

            # Update right motor
            current_right_position = self.right_motor.get_position()
            delta_right_degrees = current_right_position - self.last_right_position

            # Handle wrap-around for the right motor
            if delta_right_degrees > 180:
                delta_right_degrees -= 360
            elif delta_right_degrees < -180:
                delta_right_degrees += 360

            # Convert to radians and accumulate
            self.right_motor_radians += math.radians(delta_right_degrees)
            self.last_right_position = current_right_position

            # Sleep to avoid excessive CPU usage
            time.sleep(0.05)

    def reset_encoders(self):
        """Reset the encoder readings and accumulated radians to zero."""
        self.left_motor_radians = 0.0
        self.right_motor_radians = 0.0
        self.last_left_position = self.left_motor.get_position()
        self.last_right_position = self.right_motor.get_position()

    def check_speed(self,input_speed):
        if -75 <= input_speed <= 75:
            return input_speed
        elif input_speed < -75:
            print("Speed must be between -75 and 75 revolutions per minute.")
            return -75
        elif input_speed > 75:
            print("Speed must be between -75 and 75 revolutions per minute.")
            return 75
    def set_left_motor_speed(self, speed_rpm):
        """
        Set the left motor speed in revolutions per minute (RPM).

        Args:
            speed_rpm (float): Desired speed in revolutions per minute.
                               Positive values for forward, negative for reverse.
        """
        speed_rpm *= -1  # Inverting speed to match motor configuration
        speed_rpm = self.check_speed(speed_rpm)
        self.left_motor.start(speed=speed_rpm)


    def run_left_motor_for_seconds(self, seconds, speed=75, blocking=True):
        """
        Run the left motor for a specified number of seconds.

        Args:
            seconds (float): The duration to run the motor.
            speed (float): The speed in RPM.
        """
        speed *= -1  # Inverting speed to match motor configuration
        speed = self.check_speed(speed)
        self.left_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_left_motor_for_rotations(self, rotations, speed=75, blocking=True):
        """
        Run the left motor for a specified number of rotations.

        Args:
            rotations (float): The number of rotations to run the motor.
            speed (float): The speed in RPM.
        """
        speed *= -1  # Inverting speed to match motor configuration
        speed = self.check_speed(speed)
        self.left_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_left_motor_to_position(self, position, speed=100, blocking=True):
        """
        Run the left motor to a specified position.

        Args:
            position (float): The target position in degrees.
            speed (float): The speed in RPM.
        """
        speed *= -1  # Inverting speed to match motor configuration
        speed = self.check_speed(speed)
        self.left_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_left_motor(self):
        """
        Stop the left motor.
        """
        self.left_motor.stop()

    def set_right_motor_speed(self, speed_rpm):
        """
        Set the right motor speed in revolutions per minute (RPM).

        Args:
            speed_rpm (float): Desired speed in revolutions per minute.
                               Positive values for forward, negative for reverse.
        """
        speed_rpm = self.check_speed(speed_rpm)
        self.right_motor.start(speed_rpm)

    def run_right_motor_for_seconds(self, seconds, speed=75, blocking=True):
        """
        Run the right motor for a specified number of seconds.

        Args:
            seconds (float): The duration to run the motor.
            speed (float): The speed in RPM.
            blocking (bool): Whether the function should block until the operation is complete.
        """
        speed = self.check_speed(speed)
        self.right_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_right_motor_for_rotations(self, rotations, speed=75, blocking=True):
        """
        Run the right motor for a specified number of rotations.

        Args:
            rotations (float): The number of rotations to run the motor.
            speed (float): The speed in RPM.
            blocking (bool): Whether the function should block until the operation is complete.
        """
        speed = self.check_speed(speed)
        self.right_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_right_motor_to_position(self, position, speed=75, blocking=True):
        """
        Run the right motor to a specified position.

        Args:
            position (float): The target position in degrees.
            speed (float): The speed in RPM.
            blocking (bool): Whether the function should block until the operation is complete.
        """
        speed = self.check_speed(speed)
        self.right_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_right_motor(self):
        """
        Stop the right motor.
        """
        self.right_motor.stop()

    def stop_motors(self):
        """
        Stop both the left and right motors.

        This method stops both motors simultaneously, ensuring that the robot halts all movement.
        """
        self.right_motor.stop()
        self.left_motor.stop()

    def run_motors_for_rotations(self, rotations, left_speed=50, right_speed=50):
        """
        Run both motors for a specified number of rotations.

        Args:
            rotations (float): The number of rotations for each motor to perform.
            left_speed (float): The speed for the left motor in RPM (default is 50).
            right_speed (float): The speed for the right motor in RPM (default is 50).

        This method runs both motors for the given number of rotations, with the left motor running
        asynchronously and the right motor running synchronously to ensure accurate movement.
        """
        left_speed = self.check_speed(left_speed)
        right_speed = self.check_speed(right_speed)

        abs_left_speed = abs(left_speed)
        abs_right_speed = abs(right_speed)

        if abs_right_speed >= abs_left_speed:
            self.run_right_motor_for_rotations(rotations, speed=right_speed, blocking=False)
            self.run_left_motor_for_rotations(rotations, speed=left_speed, blocking=True)

        else:
            self.run_left_motor_for_rotations(rotations, speed=left_speed, blocking=False)
            self.run_right_motor_for_rotations(rotations, speed=right_speed, blocking=True)



    def run_motors_for_seconds(self, seconds, left_speed=50, right_speed=50):
        """
        Run both motors for a specified number of seconds.

        Args:
            seconds (float): The duration in seconds to run both motors.
            left_speed (float): The speed for the left motor in RPM (default is 50).
            right_speed (float): The speed for the right motor in RPM (default is 50).

        This method runs both motors for the given duration, with the left motor running
        asynchronously and the right motor running synchronously to ensure consistent movement.
        """
        left_speed = self.check_speed(left_speed)
        right_speed = self.check_speed(right_speed)
        self.run_left_motor_for_seconds(seconds, speed=left_speed, blocking=False)
        self.run_right_motor_for_seconds(seconds, speed=right_speed, blocking=True)

    def get_encoder_readings(self):
        """
        Get the current encoder readings for both motors.

        Returns:
            list: A list containing the accumulated radians for the left and right motors [left, right].

        This method returns the current accumulated rotation in radians for both motors, providing
        an easy way to access the total rotation since the last reset.
        """
        return [self.left_motor_radians, self.right_motor_radians]

    def get_left_encoder_reading(self):
        """
        Get the current encoder reading for the left motor.

        Returns:
            float: The accumulated radians for the left motor.

        This method returns the current accumulated rotation in radians for the left motor, allowing
        users to monitor the left motor's movement specifically.
        """
        return self.left_motor_radians

    def get_right_encoder_reading(self):
        """
        Get the current encoder reading for the right motor.

        Returns:
            float: The accumulated radians for the right motor.

        This method returns the current accumulated rotation in radians for the right motor, allowing
        users to monitor the right motor's movement specifically.
        """
        return self.right_motor_radians

    def disconnect_robot(self):
        """
        Safely disconnect the robot by stopping all motors and terminating the position tracking thread.

        This method performs the following actions:
        1. Stops the Lidar if it is enabled.
        2. Stops both motors to ensure the robot halts all movement.
        3. Stops the thread that tracks motor positions, ensuring it terminates cleanly.

        This function is typically called during shutdown or when the robot needs to be safely disconnected
        from its operational state.
        """
        self.stop_motors()
        if self.lidar is not None:
            self.lidar.stop_lidar()
        if self.camera is not None:
            self.camera.stop_camera()
        self.stop_thread = True
        self.position_thread.join()
        self.imu.stop()


    def shutdown(self, signum, frame):
        """
        Gracefully shutdown HamBot.
        """
        print("Shutdown signal received. Stopping motors...")
        self.disconnect_robot()
        sys.exit(0)