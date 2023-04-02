import cv2
import numpy as np

cap = cv2.VideoCapture(0)

def getColorMask(img):
    lowerBound = np.array([0,200,130])
    upperBound = np.array([30,255,255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2H5V)
    return cv2.inRange(hsv,lowerBound,upperBound)

#returns x,y,w,h of center of the orange blob
def findBall(imgMask):
    contours, hierarchy = cv2.findContours(imgMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_cnt_area = 0
    valid_x = 1
    valid_y = 1
    valid_w = 0
    valid_h = 0
    for i,cntr in enumerate(contours):
        x,y,w,h = cv2.boundingRect(cntr)
        if (cv2.contourArea(cntr) >= max_cnt_area):
            max_cnt_area = cv2.contourArea(cntr)
            valid_x = x
            valid_y = y
            valid_w = w
            valid_h = h

    return [valid_x, valid_y, valid_w, valid_h]

while 1:
    ret, img = cap.read()
    mask = getColorMask(img)

    blob = findBall(mask)
    start_point = (int(blob[0]), int(blob[1]))
    end_point = (int(blob[0]+blob[2]), int(blob[1]+blob[3]))

    color = (127,255,0)
    thickness = 2

    img = cv2.rectangle(img, start_point, end_point, color, thickness)

    cv2.imshow('imageMask',mask)
    cv2.imshow('image',img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

