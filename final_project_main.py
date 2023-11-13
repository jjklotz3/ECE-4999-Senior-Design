import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
import time
import os 
from state_3 import blue_line_tracking, calculate_angle, calculate_centerline

os.environ['QT_QPA_PLATFORM']= 'xcb'
state_string = ""
inverted=False
current_state = 3
start_time_state_3 = None
start_time_state_4 = None

#STATES DESCRIPTIONS
#S1: DRIVE OFF TABLE
#S2: IMPLEMENT PATH FINDING ALGORITHM TO DRIVE OFF TABLE
#S3: FOLLOW THE LINE
#S4: DRIVE TO CALCULATED LAUNCH POSITION, START SERVO, RELEASE LATCH
#S5: IMPLEMENT PATH FINDING ALGORITHM TO FIND BOX AND STOP 

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
# Disable GPIO warnings
GPIO.setwarnings(False)
# Define the motor pins
    
IN1 = 5 #right motor direction pin1 
IN2 = 6 #right motor direction pin2 
IN3 = 17 #left motor direction pin1 
IN4 = 23 #left motor direction pin2 
ENA = 27 #right motor PWM speed pin 
ENB = 22 #left motor PWM speed pin 

# Set the motor pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
    

# Create PWM objects for speed control
pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)

pwm_a.start(0)
pwm_b.start(0)

# Function to drive the motors
def drive(right_speed, left_speed): 
    if left_speed > 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    elif left_speed < 0:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
    if right_speed > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif right_speed < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

    pwm_a.ChangeDutyCycle(abs(right_speed))
    pwm_b.ChangeDutyCycle(abs(left_speed))


def display_state1():
    while True:
        ret, frame = cap.read()
        cv2.imshow("State 1", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

def display_state2():
    while True:
        ret, frame = cap.read()
        # You can modify the frame or perform different operations for state 2
        # For example, you can apply image processing to the frame
        processed_frame = frame  # Replace this with your actual processing
        cv2.imshow("State 2", processed_frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

def state3(frame):   
        global start_time_state_3
        global current_state
        
        if start_time_state_3 is None:
            start_time_state_3 = time.perf_counter()
            
        kp_corner = 3.4# Proportional gain for corners and cornering lines
        ki_corner=  0.0 # Integral gain for corners and cornering lines
        kd_corner = 0.1 # Derivative gain for corners and cornering lines

        kp_straight = 0.4 # Proportional gain for straight lines
        ki_straight= 0.0  # Integral gain for straight lines
        kd_straight = 0.0  # Derivative gain for straight lines

        integral = 0
        prev_error = 0
        error = 0 

        slow_speed = 80 #Speed for corners, cornering lines and intersections
        fast_speed =  100 #Speed for corners, cornering line
        intersection_speed = 50 #Speed for intersections


        #Initalize varaibles for PID loop
        line_track_return = blue_line_tracking(frame)
        processed_frame = line_track_return[0]
        distance = line_track_return[1]
        line_type = line_track_return[2]
        area = line_track_return[3]
        direction = line_track_return[4]
        right_speed,left_speed = None,None
        angle = line_track_return[5]
        
        
    
    

        if processed_frame is not None and processed_frame.shape[0] > 0 and processed_frame.shape[1] > 0:
           #while current_state == 3: 
                #If the line is lost, back up until line is found again
                if distance == -500:
                        elapsed_time = (time.perf_counter() - start_time_state_3)
                        if elapsed_time > 19:
                            drive(0,0)
                            current_state = 4
                            print("Moving to State 4",elapsed_time)
                        else:
                            right_speed =  -1 * slow_speed
                            left_speed =  -1 * slow_speed
                            drive(right_speed,left_speed)
                            print("Backing Up",(right_speed,left_speed),elapsed_time)
                else:    
                    
                    #Proportional variable
                    error = distance

                    if error is None:
                        
                        #Integral variable
                        integral = 0
                            # Rest of your PID controller logic
                    else:
                        
                        #PID Loop
                        integral += error
                        derivative = error - prev_error
                        pid = lambda error,integral,derivative,kp,ki,kd: kp * error + ki * integral + kd * derivative
                        
                        #PID Loop for Corners and Cornering Lines
                        if line_type == "Right Corner" or line_type == "Cornering Line" or line_type == "Left Corner":
                            pid_output = pid(error,integral,derivative,kp_corner,ki_corner,kd_corner)
                            right_speed = (slow_speed) - pid_output
                            left_speed = (slow_speed) + pid_output

                            # Ensure motor speeds are within a valid range (0-100)
                            right_speed = max(0,min((slow_speed),right_speed))
                            left_speed = max(0, min((slow_speed), left_speed))
                            drive(right_speed, left_speed)
                        
                        #PID loop for Straight Line
                        elif line_type == "Straight Line":
                            pid_output = pid(error,integral,derivative,kp_straight,ki_straight,kd_straight)
                            right_speed = (fast_speed) - pid_output
                            left_speed = (fast_speed) + pid_output

                            # Ensure motor speeds are within a valid range (0-100)
                            right_speed = max(0,min((fast_speed),right_speed))
                            left_speed = max(0, min((fast_speed), left_speed))
                            drive(right_speed, left_speed)
                        
                        #Motor control for Intersection
                        else:
                            if direction == "Left":
                                right_speed,left_speed = -1,intersection_speed
                                drive(right_speed,left_speed)
                            
                            right_speed,left_speed = intersection_speed,-1
                            drive(right_speed,left_speed)
                            print(direction)
                            
                        #Derrivative Variable
                        prev_error = error

                        #Print Helpful Variables
                        print(right_speed,left_speed,angle,line_type)
                    

def display_state4():
    global current_state
    global start_time_state_4
    if start_time_state_4 is None:
            start_time_state_4 = time.perf_counter()
        
    elapsed_time = (time.perf_counter() - start_time_state_4)
    if elapsed_time < 4:
       drive(-40,-40)
    else:
        drive(0,0)
    print("In State 4")

def display_state5():
    print("State")


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,180 )
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 200)


while True:
    
    ret, frame = cap.read()
    if not ret:
        break
    # Check if the robot is upside down (replace this condition with your detection logic)

    if inverted:
        # Rotate the frame by 180 degrees
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        
    if current_state == 1:
        display_state1()
    if current_state ==2:
        display_state2()
    if current_state ==3:
        state3(frame)
    if current_state ==4:
        display_state4()
    if current_state ==5:
        display_state5()
    

    print(current_state) 
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()