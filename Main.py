__author__ = 'Ikechukwu Ofodile -- ikechukwu.ofodile@estcube.eu'

import cv2
import numpy as np
import math
import serial
import time
from time import sleep

import Detect as Detect
import Dribble as Drib
import Motor_controller as Motor
import Coilgun as C
#import File


'''
# states[STATE]()
    states = {END: end,
              FIND_BALL: drive_to_ball,
              FIND_GOAL: find_goal,
              HIT_GOAL: hit_goalf
              }
'''

class Main:

    def __init__(self):
        print "INITIALIZING..."
        self.detect = Detect.Detect(h_low_B, s_low_B, v_low_B, h_high_B, s_high_B, v_high_B, h_low_Gb, s_low_Gb, v_low_Gb, h_high_Gb, s_high_Gb, v_high_Gb,
                        h_low_Gy, s_low_Gy, v_low_Gy, h_high_Gy, s_high_Gy, v_high_Gy)
        self.dribbler = Drib.Dribbler(port)
        self.motors = Motor.Motors_controller(port)
        self.coilgun = C.Coilgun(port)
        print "ALL INITIALIZED"

    def check_ref(self):
        outr = ''
        while connection_ref.inWaiting() > 0:
            outr += connection_ref.read(1)

        if (outr != '') & (len(outr) >= 12):
            if (outr[0] == 'a') & (outr[1] == Robotiv2ljak):
                if outr[2] == 'X':
                    if outr[3:7] == 'STOP':
                        Main.__exit__(self)
                elif outr[2] == Robotit2his:
                    if outr[3:7] == 'STOP':
                        Main.__exit__(self)

    def __exit__(self):
        print "SHUTTING DOWN..."
        main.motors.shut_down()
        sleep(1)
        main.dribbler.shut_down()
        # sleep(1)
        main.coilgun.shut_down()
        main.detect.shut_down()
        sleep(0.3)
        port.close()
        print "SHUT DOWN"

if __name__ == '__main__':
    connection_ref= serial.Serial(
        port='COM4',  # make global
        baudrate=9600,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    connection_ref.isOpen()

    port = serial.Serial('COM3', baudrate=9600, timeout=None)  # make global
    print("connected to: " + port.portstr)

    END = 0
    FIND_BALL = 1
    FIND_GOAL = 2
    NO_BALLS = 3
    PREP_GOAL = 4
    DRIVE_AROUND = 5
    WAIT_REF = 6

    TEST = 11

    #GLOBAL VARIABLES
    '''h_low_B, s_low_B, v_low_B, h_high_B, s_high_B, v_high_B = File.readfile('ball_values.txt')
    h_low_Gb, s_low_Gb, v_low_Gb, h_high_Gb, s_high_Gb, v_high_Gb = File.readfile('goal_values_blue.txt')
    h_low_Gy, s_low_Gy, v_low_Gy, h_high_Gy, s_high_Gy, v_high_Gy = File.readfile('goal_values_yellow.txt')'''
    h_low_B = 5
    s_low_B =158
    v_low_B = 132
    h_high_B = 12
    s_high_B = 200
    v_high_B = 253
    h_low_Gb = 86
    s_low_Gb = 63
    v_low_Gb = 44,
    h_high_Gb = 126
    s_high_Gb = 131
    v_high_Gb = 131
    h_low_Gy = 10
    s_low_Gy = 80
    v_low_Gy = 178
    h_high_Gy = 51
    s_high_Gy = 116
    v_high_Gy = 213

    _enemy_goal = 'y'  # 'b' - blue; 'y' - yellow
    _driveto_goal = 'y'
    _camera_buffer = 2
    Robotiv2ljak = 'B'
    Robotit2his = 'A'

    ###     ALGV22RTUSTAMINE     ###
    B_x = 0.0
    B_y = 0.0
    # Ballradius = 0.0
    Gx = 0.0
    t_start = time.time()
    t_end = time.time()

    main = Main()

    STATE = FIND_BALL

    while True:
        main.detect.read_frame(_camera_buffer)

        '''        if STATE != WAIT_REF:
            Main.check_ref(main)

        if STATE == WAIT_REF:
            print('WAIT_REF')
            out = ''
            while connection_ref.inWaiting() > 0:
                out += connection_ref.read(1)

            if (out != '') & (len(out) >= 12):
                if (out[0] == 'a') & (out[1] == Robotiv2ljak):
                    if out[2] == 'X':
                        if out[3:8] == 'START':
                            STATE = FIND_BALL
                            main.dribbler.activate(True)

                    elif out[2] == Robotit2his:
                        if out[3:8] == 'START':
                            connection_ref.write('a' + Robotiv2ljak + Robotit2his + "ACK-----")
                            STATE = FIND_BALL
                            main.dribbler.activate(True)
                        if out[3:7] == 'PING':
                            connection_ref.write('a' + Robotiv2ljak + Robotit2his + "ACK-----")'''
        if STATE == FIND_BALL:
            print('FIND_BALL')
            if not main.dribbler.ball_in():
                B_x, B_y = main.detect.detectball()
                if B_x in range(310, 330) and B_y < 440:
                    main.motors.drive_straight()
                    print("__DRIVE STRAIGHT")
                elif B_x == 0 and B_y == 0:
                    t_start = time.time()
                    STATE = NO_BALLS
                    # main.motors.turn('left')
                    # print("no balls :(")
                else:
                    main.motors.drive_to_ball(B_x, B_y)
            else:
                main.motors.turn('stop')
                print("ball in")
                STATE = FIND_GOAL

        elif STATE == FIND_GOAL:
            print('FIND_GOAL')
            if main.dribbler.ball_in():
                Gx, goal_area = main.detect.detectgoal(_enemy_goal)
                if main.motors.goal(Gx):
                    main.coilgun.kick()
                    STATE = FIND_BALL
            else:
                main.motors.turn('stop')
                STATE = FIND_BALL
                print("_BALL LOST")

        elif STATE == NO_BALLS:
            print('NO BALLS')
            t_end = time.time()
            if (t_end - t_start) > 4:
                STATE = PREP_GOAL
            else:
                B_x, B_y = main.detect.detectball()
                if B_x == 0 and B_y == 0:
                    main.motors.turn('left')
                else:
                    STATE = FIND_BALL

        elif STATE == PREP_GOAL:
            print('PREP GOAL')
            Gx, goal_area = main.detect.detectgoal(_driveto_goal)
            if goal_area > 20000:
                if _driveto_goal == 'b':
                    _driveto_goal = 'y'
                else:
                    _driveto_goal = 'b'
            if main.motors.goal(Gx):
                t_start = time.time()
                STATE = DRIVE_AROUND

        elif STATE == DRIVE_AROUND:
            print('DRIVE AROUND')
            B_x, B_y = main.detect.detectball()
            Gx, goal_area = main.detect.detectgoal(_driveto_goal)
            if goal_area > 20000:
                print(goal_area)
                if _driveto_goal == 'b':
                    _driveto_goal = 'y'
                else:
                    _driveto_goal = 'b'
                STATE = FIND_BALL  # v6ib panna PREP_GOAL
            else:
                t_end = time.time()
                if (B_x != 0 and B_y != 0) or (t_end - t_start) > 3:
                    STATE = FIND_BALL
                else:
                    main.motors.drive_straight()


        elif STATE == TEST:
            print('TEST')
            # main.motors.turn('left')
            # Goalx = main.vision.find_goal()
            # main.motors.center_goal(Goalx)
            # main.motors.turn('left')
            # main.motors.center_goal(Goalx)

        main.detect.show_frame()

        k = cv2.waitKey(1) & 0xFF

        if k == 27:
            main.__exit__()
            break