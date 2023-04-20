import RPi.GPIO as GPIO
from time import sleep
import cv2
import numpy as np

# Setting up video input
cap = cv2.VideoCapture(0)

# Connected pins of GPIO (https://pinout.xyz for ref)
servo_pin = 16
stepper_dir_pin = 8 
stepper_step_pin = 10

def getColorMask(img):
    lowerBound = np.array([0,180,255])
    upperBound = np.array([20,255,255])
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BG2HSV)
    return cv2.inRange(hsv,loweBound, upperBound)


# Takes 'servo' obj and an angle in degrees, turns the servo to that angle, angle must be between 0 and 180
def turnServo(angle,Stime):
    
    # Setting up the GPIO board and servo with 50hz data transfer mode
    global servo_pin
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(servo_pin,GPIO.OUT)
    servo = GPIO.PWM(servo_pin,50)
    servo.start(0)

    # const. values from trial and error
    duty = (angle)/18.9 + 1.8
    servo.ChangeDutyCycle(duty)
    sleep(Stime)


# Takes angle and a direction and rotates stepepr, dir = 1 is counter-clock-wise, dir = 0 is clock-wise
def turnStepper(angle, dir):
    # Setting up stepper GPIO pins for output
    global stepper_dir_pin, stepper_step_pin
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([stepper_dir_pin, stepper_step_pin], GPIO.OUT)
    
    # dir output
    GPIO.output(stepper_dir_pin, dir)
    for i in range(int(angle//1.8)):
        GPIO.output(stepper_step_pin,1)
        sleep(0.01)
        GPIO.output(stepper_step_pin,0)


def getColorMask(img):
    lowerBound = np.array([0,200,130])
    upperBound = np.array([30,255,255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
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






def main():
    GPIO.setwarnings(False)
    
    turnServo(90,1)
    servo_angle = 90
    while 1:
        ret, img = cap.read()
        mask = getColorMask(img)

        blob = findBall(mask)
        start_point = (int(blob[0]), int(blob[1]))
        end_point = (int(blob[0]+blob[2]), int(blob[1]+blob[3]))
        center_point = (int(blob[0]+blob[2]/2),int(blob[1]+blob[3]/2))

        color = (127,255,0)
        thickness = 2
        img = cv2.rectangle(img, start_point, end_point, color, thickness)
        cv2.imshow('videoOutput',img)
        ball_found = True
        if center_point[0]==1 or center_point[1]==1:
            ball_found = False
        
        # picture is 640X480
        
        if ball_found:
            if center_point[1] <= 180:
                turnServo(servo_angle - 2,0.05)
                servo_angle = servo_angle - 2
            if center_point[1] >= 300:
                turnServo(servo_angle + 2,0.05)
                servo_angle = servo_angle + 2
            if center_point[0] <= 240:
                turnStepper(1.8,1)
            if center_point[0] >= 400:
                turnStepper(1.8,0)
                
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    
    # clean up

if __name__ == '__main__':
    main()

cap.release()
cv2.destroyAllWindows()


