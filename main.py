import RPi.GPIO as GPIO
from time import sleep
#import cv2
#import numpy as np

# Connected pins of GPIO (https://pinout.xyz for ref)
servo_pin = 16
stepper_dir_pin = 8
stepper_step_pin = 10

def getColorMask(img):
    lowerBound = np.array([0,180,255])
    upperBound = np.array([20,255,255])
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BG2HSV)
    return cv2.inRange(hsv,loweBound, upperBound)

#returns x,y of center of the orange blob
def findBall():
    cv2.namedWindow('imgmask')
    img = open("img.jpg",'r')
    cv2.imshow('imgmask',getColorMask(img))
    return 0    


# Takes 'servo' obj and an angle in degrees, turns the servo to that angle, angle must be between 0 and 180
def turnServo(angle):
    
    # Setting up the GPIO board and servo with 50hz data transfer mode
    global servo_pin
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(servo_pin,GPIO.OUT)
    servo = GPIO.PWM(servo_pin,50)
    servo.start(0)

    # const. values from trial and error
    duty = (angle)/18.947368421052631578947368421053 + 1.8
    servo.ChangeDutyCycle(duty)
    sleep(0.5)


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

def main():
    GPIO.setwarnings(False)
    
    # Main code goes here
    for i in range(10):
        a = int(input())
        turnServo(a)
        turnStepper(180,1)
        a = int(input())
        turnServo(a)
        turnStepper(180,0)

    # clean up

if __name__ == '__main__':
    main()