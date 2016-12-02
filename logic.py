mainboard = ''


class Logic:
    leftT = 330
    rightT = 310
    goalMin = 110
    goalMax = 200
    goal_find_cnt = 0

    def __init__(self, mainboard):
        self.mainboard = mainboard

    def isBallFound(self, balldetails):
        return balldetails != [0, 0, 0]

    def movetoball(self, balldetails):
        if balldetails == [0,0,0]:
            self.mainboard.circlearound(speed=55)
        else:
            self.mainboard.forwardspeed()
            if balldetails[0] > self.leftT:
                self.mainboard.turnleft(50)
            elif balldetails[0] < self.rightT:
                self.mainboard.turnright(50)
            elif balldetails[0] >= self.rightT and balldetails[0] <= self.leftT:
                self.mainboard.backwheel(0)
                self.mainboard.forwardspeed()

    def aimandshoot(self, goaldetails):
        print ('goal:',goaldetails)
        goal_scored = False

        if goaldetails==[0, 0]:
            self.mainboard.circlearound()
        else:
            self.mainboard.forwardspeed()
            if goaldetails[0] > 325:
                self.mainboard.turnleft()
            elif goaldetails[0] < 315:
                self.mainboard.turnright()
            elif (goaldetails[0] > 315 and goaldetails[0] < 325):
                self.mainboard.kick()
                goal_scored = True
        '''        if goaldetails == [0, 0]:
            self.goal_find_cnt = 0
            self.mainboard.circlearound(35)
            #self.mainboard.turnleft(30)
        else:
            #self.mainboard.forwardspeed()
            #if goaldetails[0] > self.goalMax:
            #    self.mainboard.turnleft(40)
            #elif goaldetails[0] < self.goalMin:
            #    self.mainboard.turnright(40)
            #elif (goaldetails[1] >= self.goalMin):
            if goaldetails[0] < self.goalMin:
                self.mainboard.forwardspeed(30)
            elif (goaldetails[1] >= self.goalMin and goaldetails[1] <= self.goalMax):
                self.goal_find_cnt += 1
                if self.goal_find_cnt > 7:
                    self.mainboard.kick()
                    goal_scored = True
            else:
                self.goal_find_cnt = 0
                self.mainboard.turnright(30)'''
        return goal_scored
