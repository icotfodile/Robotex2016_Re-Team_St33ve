__author__ = 'Ikechukwu Ofodile -- ikechukwu.ofodile@estcube.eu'

from time import sleep
import cv2
import numpy as np


class Detect:

    def __init__(self, h_low_B, s_low_B, v_low_B, h_high_B, s_high_B, v_high_B, h_low_Gb, s_low_Gb, v_low_Gb, h_high_Gb, s_high_Gb, v_high_Gb,
                        h_low_Gy, s_low_Gy, v_low_Gy, h_high_Gy, s_high_Gy, v_high_Gy):
        self.cap = cv2.VideoCapture(1)
        sleep(0.3)
        self.frame = self.cap.read()
        sleep(0.3)
        self.frame = self.cap.read()
        sleep(0.1)
        self.hsv = self.frame
        self.lowerballs = np.array([h_low_B, s_low_B, v_low_B])
        self.upperballs = np.array([h_high_B, s_high_B, v_high_B])
        self.lowergoalblue = np.array([h_low_Gb, s_low_Gb, v_low_Gb])
        self.uppergoalblue = np.array([h_high_Gb, s_high_Gb, v_high_Gb])
        self.lowergoalyellow = np.array([h_low_Gy, s_low_Gy, v_low_Gy])
        self.uppergoalyellow = np.array([h_high_Gy, s_high_Gy, v_high_Gy])
        print "camera initialized"

    def read_frame(self, buffer):
        for i in range(buffer):
            _, self.frame = self.cap.read()
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.hsv = cv2.GaussianBlur(self.hsv, (25, 25), 0)

    def show_frame(self):
        # frame 640x480
        cv2.imshow('frame', self.frame)

    def detectball(self):
        # Ballradius = 0
        mask = cv2.inRange(self.hsv, self.lowerballs, self.upperballs)
        erodeBalls = cv2.erode(mask, None, iterations=2)
        contours = cv2.findContours(erodeBalls.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        B_x = 0
        B_y = 0

        if len(contours) > 0:
            contoursMax = max(contours, key=cv2.contourArea)
            ((B_x, B_y), B_r) = cv2.minEnclosingCircle(contoursMax)
            # circle: image, center, radius, color, thickness
            # cv2.circle(self.frame, (int(Ballx), int(Bally)), int(Ballradius), (0, 255, 0), 2)
            cv2.circle(self.frame, (int(B_x), int(B_y)), 2, (0, 0, 255), 5)
        if B_x is None or B_y is None:
            return 0, 0
        else:
            return B_x, B_y


    def detectgoal(self, goal):
        if goal == 'b':
            mask = cv2.inRange(self.hsv, self.lowergoalblue, self.uppergoalblue)
        else:
            mask = cv2.inRange(self.hsv, self.lowergoalyellow, self.uppergoalyellow)
        erodeGoal = cv2.erode(mask, None, iterations=2)
        contours = cv2.findContours(erodeGoal.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) [-2]

        cx = 0
        if len(contours) > 0:
            contoursMax = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contoursMax)
            if area > 500:
                cv2.drawContours(self.frame, contoursMax, -1, (255, 105, 180), 3)
                M = cv2.moments(contoursMax)
                if M["m00"] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(self.frame, (int(cx), int(cy)), 2, (0, 0, 233), 5)
            return cx, area
        else:
            return 0, 0

    def shut_down(self):
        self.cap.release()
        cv2.destroyAllWindows()
        print("detect shut down")