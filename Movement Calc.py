import math
class MobileRobot:

    def __init__(self, wheel_radius, axel_distance):
            self.r = wheel_radius
            self.axel = axel_distance

    def straight_line(self):
        xi = float(input("Enter x initial value: "))
        yi = float(input("Enter y initial value: "))
        xf = float(input("Enter x final value: "))
        yf = float(input("Enter y final value: "))

        d = math.sqrt((xf - xi)**2 + (yf - yi)**2)

        WAVL = float(input("Enter left wheel angular velocity (rad/s): "))
        WAVR = float(input("Enter right wheel angular velocity (rad/s): "))

        w = (WAVL + WAVR)/2
        wvl = WAVL * self.r
        wvr = WAVR * self.r
        v = w * self.r
        t = d/v

        print(f'Segement Distance: {d} m')
        print(f'Segement Time: {t} s')
        print(f'Velocity: {v} m/s')
        print(f'Left Wheel Velocity: {wvl} rad/s')
        print(f'Right Wheel Velocity: {wvr} rad/s')

    def curved_line_left(self):
        WAVR = float(input("Enter right wheel angular velocity (rad/s): "))
        v_r = WAVR * self.r

        xi = float(input("Enter x initial value: "))
        yi = float(input("Enter y initial value: "))
        xf = float(input("Enter x final value: "))
        yf = float(input("Enter y final value: "))

        𝜃si = float(input("Enter robot initial direction (rad): "))
        𝜃sf = float(input("Enter robot final direction (rad): "))
        𝜃d = 𝜃sf - 𝜃si

        R = math.sqrt((xi - xf)**2 + (yi - yf)**2) / (2 * math.sin(abs(𝜃d)/2))

        w = v_r / (R + self.axel/2)

        v_l = w * (R - self.axel/2)
        WAVL = v_l / self.r

        s = abs(R * 𝜃d)
        v = (v_r + v_l) / 2
        t = abs(s / v)

        print(f"Turning radius: {R} m")
        print(f"Left wheel angular velocity: {WAVL} rad/s")
        print(f"Arc length: {s} m")
        print(f"Travel time: {t} s")

    def curved_line_right(self):
        WAVL = float(input("Enter left wheel angular velocity (rad/s): "))
        v_l = WAVL * self.r

        xi = float(input("Enter x initial value: "))
        yi = float(input("Enter y initial value: "))
        xf = float(input("Enter x final value: "))
        yf = float(input("Enter y final value: "))

        𝜃si = float(input("Enter robot initial direction: ")) 
        𝜃sf = float(input("Enter robot final direction: ")) 
        𝜃d = 𝜃sf - 𝜃si

        R = math.sqrt((xi - xf)**2 + (yi - yf)**2) / (2 * math.sin(abs(𝜃d)/2))

        w = v_l / (R - self.axel/2)

        v_r = w * (R + self.axel/2)
        WAVR = v_r / self.r

        s = abs(R * 𝜃d)
        v = (v_r + v_l) / 2
        t = abs(s / v)

        print(f"Turning radius: {R} m")
        print(f"Right wheel angular velocity: {WAVR} rad/s")
        print(f"Arc length: {s} m")
        print(f"Travel time: {t} s")

    def turn_in_place(self):
        WAVL = float(input("Enter left wheel angular velocity (rad/s): "))
        WAVR = float(input("Enter right wheel angular velocity (rad/s): "))
        𝜃si = float(input("Enter robot initial direction: ")) 
        𝜃sf = float(input("Enter robot final direction: "))

        𝜃d = 𝜃sf - 𝜃si

        wvl = WAVL * self.r
        wvr = WAVR * self.r

        w = (wvr - wvl)/self.axel
        v = (abs(w) * self.axel)/2
        t = abs(𝜃d)/w

        print(f"Travel time: {t} s")
        print(f"Velocity: {v} m/s")
        print(f"Angular Velocity: {w} rad/s")


def main():
    r = float(input("Enter wheel radius (m): "))
    axel = float(input("Enter axel distance (m): "))

    robot = MobileRobot(r, axel)
    robot.straight_line()
    robot.curved_line_left()
    robot.curved_line_right()
    robot.turn_in_place()




if __name__ == "__main__":
    main()

    