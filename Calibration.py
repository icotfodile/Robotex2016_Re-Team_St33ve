import cv2
import numpy as np
import ConfigParser
import imutils

config = ConfigParser.RawConfigParser()


def nothing(x):
    pass


camera = cv2.VideoCapture(1)

config.read('example.cfg')
ballmin1 = config.getint('Ball', 'min1')
ballmin2 = config.getint('Ball', 'min2')
ballmin3 = config.getint('Ball', 'min3')
ballmax1 = config.getint('Ball', 'max1')
ballmax2 = config.getint('Ball', 'max2')
ballmax3 = config.getint('Ball', 'max3')

config.read('example.cfg')
goalmin1 = config.getint('Goal', 'min1')
goalmin2 = config.getint('Goal', 'min2')
goalmin3 = config.getint('Goal', 'min3')
goalmax1 = config.getint('Goal', 'max1')
goalmax2 = config.getint('Goal', 'max2')
goalmax3 = config.getint('Goal', 'max3')

cv2.namedWindow('ballcalibration')
cv2.createTrackbar('HueMin', 'ballcalibration', ballmin1, 255, nothing)
cv2.createTrackbar('SatMin', 'ballcalibration', ballmin2, 255, nothing)
cv2.createTrackbar('ValMin', 'ballcalibration', ballmin3, 255, nothing)
cv2.createTrackbar('HueMax', 'ballcalibration', ballmax1, 255, nothing)
cv2.createTrackbar('SatMax', 'ballcalibration', ballmax2, 255, nothing)
cv2.createTrackbar('ValMax', 'ballcalibration', ballmax3, 255, nothing)

cv2.namedWindow('goalcalibration')
cv2.createTrackbar('HueMin', 'goalcalibration', goalmin1, 255, nothing)
cv2.createTrackbar('SatMin', 'goalcalibration', goalmin2, 255, nothing)
cv2.createTrackbar('ValMin', 'goalcalibration', goalmin3, 255, nothing)
cv2.createTrackbar('HueMax', 'goalcalibration', goalmax1, 255, nothing)
cv2.createTrackbar('SatMax', 'goalcalibration', goalmax2, 255, nothing)
cv2.createTrackbar('ValMax', 'goalcalibration', goalmax3, 255, nothing)

while True:

    # Take each frame
    (grabbed, frame) = camera.read()
    frame = imutils.resize(frame, width=600)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (11, 11), 0)

    ballmin1 = cv2.getTrackbarPos('HueMin', 'ballcalibration')
    ballmax1 = cv2.getTrackbarPos('HueMax', 'ballcalibration')
    ballmin2 = cv2.getTrackbarPos('SatMin', 'ballcalibration')
    ballmax2 = cv2.getTrackbarPos('SatMax', 'ballcalibration')
    ballmin3 = cv2.getTrackbarPos('ValMin', 'ballcalibration')
    ballmax3 = cv2.getTrackbarPos('ValMax', 'ballcalibration')

    goalmin1 = cv2.getTrackbarPos('HueMin', 'goalcalibration')
    goalmax1 = cv2.getTrackbarPos('HueMax', 'goalcalibration')
    goalmin2 = cv2.getTrackbarPos('SatMin', 'goalcalibration')
    goalmax2 = cv2.getTrackbarPos('SatMax', 'goalcalibration')
    goalmin3 = cv2.getTrackbarPos('ValMin', 'goalcalibration')
    goalmax3 = cv2.getTrackbarPos('ValMax', 'goalcalibration')

    # define range of orange color in HSV
    lower_ball = np.array([ballmin1, ballmin2, ballmin3])
    upper_ball = np.array([ballmax1, ballmax2, ballmax3])

    lower_goal = np.array([goalmin1, goalmin2, goalmin3])
    upper_goal = np.array([goalmax1, goalmax2, goalmax3])

    # Threshold the HSV image to get only orange colors
    ballmask = cv2.inRange(hsv, lower_ball, upper_ball)
    ballmask = cv2.erode(ballmask, None, iterations=2)
    ballmask = cv2.dilate(ballmask, None, iterations=2)

    goalmask = cv2.inRange(hsv, lower_goal, upper_goal)
    goalmask = cv2.erode(goalmask, None, iterations=2)
    goalmask = cv2.dilate(goalmask, None, iterations=2)

    # Bitwise-AND mask and original image
    # res = cv2.bitwise_and(frame, frame, mask=mask)

    ballcontours = cv2.findContours(ballmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    goalcontours = cv2.findContours(goalmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    # res = cv2.drawContours(res, cnts, -1, (0, 255, 0), 3)

    if len(ballcontours) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(ballcontours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    if len(goalcontours) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(goalcontours, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

    cv2.imshow('frame', frame)
    cv2.imshow('ballmask', ballmask)
    cv2.imshow('goalmask', goalmask)
    # cv2.imshow('res', res)

    k = cv2.waitKey(5) & 0xFF

    if k == 27:
        # config.add_section('Ball')
        config.set('Ball', 'min1', ballmin1)
        config.set('Ball', 'min2', ballmin2)
        config.set('Ball', 'min3', ballmin3)
        config.set('Ball', 'max1', ballmax1)
        config.set('Ball', 'max2', ballmax2)
        config.set('Ball', 'max3', ballmax3)

        config.set('Goal', 'min1', goalmin1)
        config.set('Goal', 'min2', goalmin2)
        config.set('Goal', 'min3', goalmin3)
        config.set('Goal', 'max1', goalmax1)
        config.set('Goal', 'max2', goalmax2)
        config.set('Goal', 'max3', goalmax3)

        with open('example.cfg', 'wb') as configfile:
            config.write(configfile)
        break

camera.release()
cv2.destroyAllWindows()
