import serial
from time import sleep

class MainboardController:
    port = "COM3"
    motor = ""
    # l+002r-112b+502

    def __init__(self):
        self.motor = serial.Serial(self.port, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        self.currentforwardspeed = 0

    def circlearound(self, speed=150):
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

    def dribbler_init(self):
        self.motor.write('d10\n')

    '''    def dribbler_init(self):
        self.'''

    def dribbler_on(self):
        self.motor.write('d150\n')

    def charge_kicker(self):
        self.currentturningspeed = 0

    def kick(self):
        self.currentturningspeed = 0

    def motor_shut_down(self, speed = 0):
        self.motor.write("ca" + str(speed) + '\n')
        sleep(0.05)
        print("motors shut down")

    def dribbler_shut_down(self, speed=0):
        self.motor.write("d" + str(speed) + '\n')
        sleep(0.05)
        print("motors shut down")