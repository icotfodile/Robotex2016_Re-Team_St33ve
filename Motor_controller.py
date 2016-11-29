mainboard=''

class Motor:
    def __init__(self,mainboard):
        self.mainboard=mainboard

    def movetoball(self, balldetails):
        self.mainboard.forwardspeed()

        if balldetails[0] > 340:
            self.mainboard.turnleft()
        elif balldetails[0] < 300:
            self.mainboard.turnright()
        elif (balldetails[0] > 300 and balldetails[0] < 340):
            self.mainboard.backwheel(0)