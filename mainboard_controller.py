import serial
import time
from datetime import datetime


class MainboardController:
    port = "COM3"
    motor = ""
    ball_catch_cmd = "<bl>"
    ball_release_cmd = "<b0>"
    with_ball = False
    # l+002r-112b+502

    leftwheelbuf=0
    rightwheelbuf=0
    backwheelbuf=0


    def __init__(self):
        self.motor = serial.Serial(self.port, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        self.currentforwardspeed = 0

    def circlearound(self, speed=50):
        self.motor.write('ca' + str(speed) + '\n')

    def forwardspeed(self, speed=150):
        self.motor.write('mf' + str(speed) + '\n')
        self.currentforwardspeed = speed

    def turnleft(self, speed=150):
        self.backwheel(speed)

    def turnright(self, speed=150):
        self.backwheel(-speed)

    def backwheel(self, speed=150):
        self.motor.write('t' + str(speed) + '\n')
        self.currentturningspeed = speed

    def sendwheelcommand(self):
        cmd= 'l' + str(self.leftwheelbuf).zfill(4) + 'r' + str(self.rightwheelbuf).zfill(4) + 'b' + str(self.backwheelbuf).zfill(4) + '\n'
        self.motor.write(cmd)
        print (cmd)

    def dribbler_init(self):
        #self.motor.write('d10\n')
        self.motor.write('b0.2\n')

    def dribbler_on(self):
        #self.motor.write('d150\n')
        self.motor.write('b6\n')

    def charge_kicker(self):
        self.motor.write('j\n')

    def kick(self):
        self.motor_shut_down()
        self.charge_kicker()
        time.sleep(2)
        self.motor.write('l\n')
        time.sleep(0.1)

    def checkforball(self):
        self.motor.write('i\n')
        line = self.motor.readline().strip()
        print(line)
        return line == "<b1>"

    def stopwheels(self):
        self.fspeedbuf = 0
        self.tspeedbuf = 0
        self.sendwheelcommand()

    def stopall(self):
        self.fspeedbuf = 0
        self.tspeedbuf = 0
        self.sendwheelcommand()
        self.dribbler_init()

    def motor_shut_down(self, speed = 0):
        self.motor.write("ca" + str(speed) + '\n')
        time.sleep(0.05)
        print("motors shut down")

    def dribbler_shut_down(self, speed=0):
        #self.motor.write('d0\n')
        self.motor.write('b0\n')
        time.sleep(0.05)
        print("dribbler shut down")

    def detect_ball_catch(self):
        # Use this to bypass having to use the ID
        time.sleep(2)
        self.with_ball = True
        return self.with_ball

        # Use the lines below once you confirm the IR works
        self.with_ball = False
        start = datetime.now()
        timeout = 3
        while False:
            delta = datetime.now() - start
            line = self.motor.readline().strip()
            if line == self.ball_catch_cmd or delta.seconds > timeout:
                self.with_ball = True
                break
        return self.with_ball
