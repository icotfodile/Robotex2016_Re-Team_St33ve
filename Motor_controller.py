__author__ = 'Ikechukwu Ofodile -- ikechukwu.ofodile@estcube.eu'


from time import sleep
import numpy as np

#3 is left, 2 is right, 1 is back
#for cpp 2 is back, 1 is right, 3 is left

class Motors_controller:

    def __init__(self, port):
        self.port = port
        print "motors initialized"

    def drive_to_ball(self, b_x, b_y):
        x = 320 - b_x
        y = 480 - b_y

        z = np.math.sqrt(x * x + y * y)

        rel = y / z
        if x < 0:

            self.speed(-rel)
        else:

            self.speed(rel)

    def speed(self, rel):
        def_speed = 48
        max_speed = 60

        abs_angle = abs(rel)

        speed_left = def_speed
        speed_right = int(speed_left / abs_angle)


        if speed_right > max_speed:
            speed_left = (speed_left * max_speed) / speed_right
            speed_right = max_speed

        if rel < 0:  # Compare speed for trade off
            var = speed_right
            speed_right = speed_left
            speed_left = var

        string = "<1wl0;2wl" + str(speed_right) + ";3wl" + str(speed_left) + ">\n"
        # print(string_left +"        "+ string_right)
        self.port.write(string)

        # find and align with the goal

    def goal(self, G_x):
        if G_x == 0:
            self.port.write("<1wl53;2wl0;3wl0>\n")
            return False
        elif G_x > 350:
            self.port.write("<1wl-53;2wl0;3wl0>\n")
            return False
        elif G_x < 290:
            self.port.write("<1wl53;2wl0;3wl0>\n")
            return False
        else:
            print("GOAL ALIGNED")
            self.turn('stop')
            return True

    def turn(self, s):
        if s == 'right':
            self.port.write("<1wl-27;2wl-27;3wl27>\n")
            sleep(0.05)
            # print("right")
        elif s == 'left':
            self.port.write("<1wl27;2wl27;3wl-27>\n")
            sleep(0.05)
            # print("left")
        elif s == 'stop':
            self.port.write("<3wl0;2wl0;1wl0>\n")
            sleep(0.05)
            # print("stop")

    # Review
    def drive_straight(self):
        self.port.write("<1wl0;2wl54;3wl50>\n")

    def shut_down(self):
        self.port.write("<1wl0;2wl0;3wl0>\n")
        sleep(0.05)
        print("motors shut down")