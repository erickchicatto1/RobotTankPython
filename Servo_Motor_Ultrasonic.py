import RPi.GPIO as GPIO  
import time 
import  ,.m- 

#Input pins Ultrasonic
Tr = 11
Ec = 8
GPIO.setmode(GPIO.BCM)  
GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(Ec, GPIO.IN)  


#For the servo and DC motor
Motor_A_EN = 4
Motor_B_EN = 17
Motor_A_Pin1 = 14
Motor_A_Pin2 = 15
Motor_B_Pin1 = 27
Motor_B_Pin2 = 18
Dir_forward = 0 
Dir_backward = 1
left_forward = 0 
left_backward = 1
right_fordward = 0 
right_backward = 1
pwm_A = 0
pwm_B = 0

#DC Motor
def motorStop():#motor stops
    GPIO.output(Motor_A_Pin1,GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_B_Pin1, GPIO.LOW)
	GPIO.output(Motor_B_Pin2, GPIO.LOW)
	GPIO.output(Motor_A_EN, GPIO.LOW)
	GPIO.output(Motor_B_EN, GPIO.LOW)    

def setup(): #Motor Initialization
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
	GPIO.setup(Motor_B_EN, GPIO.OUT)
	GPIO.setup(Motor_A_Pin1, GPIO.OUT)
	GPIO.setup(Motor_A_Pin2, GPIO.OUT)
	GPIO.setup(Motor_B_Pin1, GPIO.OUT)
	GPIO.setup(Motor_B_Pin2, GPIO.OUT)
    motorStop()

    try:
      pwm_A = GPIO.PWM(Motor_A_EN,1000)
      pwm_B = GPIO.PWM(Motor_B_EN,1000)
    except:
        pass

def motor_left(status,direction,speed):
    if status == 0:
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)
    else:
        if direction == Dir_backward:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
			GPIO.output(Motor_B_Pin2, GPIO.LOW)
			pwm_B.start(100)
			pwm_B.ChangeDutyCycle(speed)
        elif direction  == Dir_forward:
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
			GPIO.output(Motor_B_Pin2, GPIO.HIGH)
			pwm_B.start(0)
			pwm_B.ChangeDutyCycle(speed)

def motor_right(status,direction,speed):
    if status == 0:
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
    else:
        if direction == Dir_backward:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
			GPIO.output(Motor_A_Pin2, GPIO.LOW)
			pwm_A.start(100)
			pwm_A.ChangeDutyCycle(speed)
        elif direction  == Dir_forward:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
			GPIO.output(Motor_A_Pin2, GPIO.HIGH)
			pwm_A.start(0)
			pwm_A.ChangeDutyCycle(speed)
return direction


def move(speed,direction,turn,radius=0.6):
    #speed = 100
    if direction == 'forward':
        if turn == 'right':
            motor_left(0, left_backward, int(speed*radius))  
            motor_right(1, right_forward, speed)
        elif turn == 'left':
            motor_left(1, left_forward, speed)
			motor_right(0, right_backward, int(speed*radius))
        else:
			motor_left(1, left_forward, speed)
			motor_right(1, right_forward, speed)
	elif direction == 'backward':
		if turn == 'right':
			motor_left(0, left_forward, int(speed*radius))
			motor_right(1, right_backward, speed)
		elif turn == 'left':
			motor_left(1, left_backward, speed)
			motor_right(0, right_forward, int(speed*radius))
		else:
			motor_left(1, left_backward, speed)
			motor_right(1, right_backward, speed)
	elif direction == 'no':
		if turn == 'right':
			motor_left(1, left_backward, speed)
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(1, right_backward, speed)
		else:
			motorStop()
	else:
		pass


def destroy():
    motorStop()
    GPIO.cleanup()

#Servo Motor
#servomotor
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)  

#Need other function for the servomotor

def servoMove():
    #for the servo motor
    while 1:
    for i in range(0,100):
        pwm.set_pwm(0,0,(300+i))
        time.sleep(0.05)
    for i in range(0,100):
        pwm.set_pwm(0,0,(400-i))
        time.sleep(0.05)


#ultrasonic sensor 
def checkdist():
    GPIO.output(Tr, GPIO.HIGH)
    time.sleep(0.000015)  
    GPIO.output(Tr, GPIO.LOW)  

    while not GPIO.input(Ec):
        pass
    t1 = time.time()
    while GPIO.input(Ec): 
        pass   
    t2 = time.time()
    return round((t2-t1)*340/2,2) 


#use ultrasonic and servomotor state machine ?
def servo_with_ultrasonic():
    #1.
    get_distance_ultrasonic =  checkdist()
    print(get_distance_ultrasonic)

    #2.classify the distance in terms of the grades with the servomotor
    if get_distance_ultrasonic < 2:
        #3. Move servo 90 degrees
        print(get_distance_ultrasonic)
        pwm.start(0)
        pwm.ChangeDutyCycle(30)
        pwm.stop()
    elif get_distance_ultrasonic < 1:
        print(get_distance_ultrasonic)
        pwm.start(0)
        pwm.ChangeDutyCycle(60)
        pwm.stop()
    else:
        print(get_distance_ultrasonic)
        pwm.start(0)
        pwm.ChangeDutyCycle(0)
        pwm.stop()


#main
if __name__ == '__main__':
    try:
        speed_set = 60
        setup()
        move(speed_set, 'forward', 'no', 0.8)
		time.sleep(1.3)
		motorStop()
		destroy()
        while 1:
            print(checkdist())
            time.sleep(1)
            #servo_with_ultrasonic()

	except KeyboardInterrupt:
		destroy()