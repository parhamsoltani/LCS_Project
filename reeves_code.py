import cv2
import numpy as np

cv2.namedWindow('image')
cap = cv2.VideoCapture(0)

#def processImageMask
#    erode, dilate = 
#    pass

def getColorMask(img):
    lowerBound = np.array([0,180,255])
    upperBound = np.array([20,255,255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv,lowerBound, upperBound)

#returns x,y of center of the orange blob
def findBall():
    cv2.namedWindow('imgmask')
    img = open("img.jpg", 'r')
    cv2.imshow('imgmask',getColorMask(img))
    return 0

while 1:
    ret, img = cap.read()
    cv2.imshow('image',img)