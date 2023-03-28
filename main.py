import RPi.GPIO as GPIO

# Connected pins of GPIO (https://pinout.xyz for ref)
servo_pin = 12
stepper_dir_pin = 8
stepper_step_pin = 10

# Takes 'servo' obj and an angle in degrees, turns the servo to that angle, angle must be between 0 and 180
def turnServo(servo, angle):
	# const. values from trial and error
	duty = (angle)/18.947368421052631578947368421053 + 1.8
	servo.ChangeDutyCycle(duty)

# Takes angle and a direction and rotates stepepr, dir = 1 is counter-clock-wise, dir = 0 is clock-wise
def turnStepper(angle, dir):
    global stepper_dir_pin, stepper_step_pin
    # dir output
    GPIO.output(stepper_dir_pin, dir)
    for i in range(angle//1.8):
        GPIO.output(stepper_step_pin,1)
        GPIO.output(stepper_step_pin,0)


def main():
    global stepper_step_pin,stepper_dir_pin, stepper_dir_GPIO, stepper_step_GPIO
    
    # Setting up the GPIO board and servo with 50hz data transfer mode
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(servo_pin,GPIO.OUT)
    servo = GPIO.PWM(servo_pin,50)
    servo.start(0)

    # Setting up stepper GPIO pins for output
    GPIO.setup([stepper_dir_pin, stepper_step_pin], GPIO.OUT)


    # Main code goes here





    # clean up
    servo.stop()
    GPIO.cleanup()

if __name__ == '__main__':
    main()