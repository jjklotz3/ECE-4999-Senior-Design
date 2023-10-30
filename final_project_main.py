import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
from time import sleep
import os 
from state_3 import *

os.environ['QT_QPA_PLATFORM']= 'xcb'
state_string = ""
inverted=False
current_state = 3
exit_loop=False




#STATES DESCRIPTIONS
#S1: DRIVE OFF TABLE
#S2: IMPLEMENT PATH FINDING ALGORITHM TO DRIVE OFF TABLE
#S3: FOLLOW THE LINE
#S4: DRIVE TO CALCULATED LAUNCH POSITION, START SERVO, RELEASE LATCH
#S5: IMPLEMENT PATH FINDING ALGORITHM TO FIND BOX AND STOP 



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
def display_state3(inverted):
    while True:
        kp_corner = 1.4# Proportional gain
        ki_corner=  0.08 # Integral gain
        kd_corner = 1.0 # Derivative gain

        kp_straight = 0.4 # Proportional gain
        ki_straight= 0.0  # Integral gain
        kd_straight = 0.2  # Derivative gain

        integral = 0
        prev_error = 0
        error = 0 

        slow_speed = 60
        fast_speed =  100
        ret, frame = cap.read()
        # You can modify the frame or perform different operations for state 2
        # For example, you can apply image processing to the frame
        if not ret:
            break

        if inverted:
            # Rotate the frame by 180 degrees
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        # Check if the robot is upside down (replace this condition with your detection logic)
        line_track_return = blue_line_tracking(frame)
        processed_frame = line_track_return[0]
        distance = line_track_return[1]
        angle = line_track_return[2]
        right_speed,left_speed = None,None
        #p1,p2 =  line_track_return[3], line_track_return[4]
        x_midpoint = line_track_return[3]

        if processed_frame is not None and processed_frame.shape[0] > 0 and processed_frame.shape[1] > 0:
            cv2.imshow("State 3", processed_frame)
            if distance == -500:
                right_speed =  -1 * slow_speed
                left_speed =  -1 * slow_speed
                drive(right_speed,left_speed)
                print("Backing Up",(right_speed,left_speed))
            else:    
                error = distance

                if error is None:
                    integral = 0
                        # Rest of your PID controller logic
                else:
                    
                    integral += error
                    derivative = error - prev_error
                    pid = lambda error,integral,derivative,kp,ki,kd: kp * error + ki * integral + kd * derivative
                    if angle >112 or angle < 70:
                        pid_output = pid(error,integral,derivative,kp_corner,ki_corner,kd_corner)
                        right_speed = (slow_speed) - pid_output
                        left_speed = (slow_speed) + pid_output

                        # Ensure motor speeds are within a valid range (0-100)
                        right_speed = max(0,min((slow_speed),right_speed))
                        left_speed = max(0, min((slow_speed), left_speed))
                        drive(right_speed, left_speed)
                    
                    else:
                        pid_output = pid(error,integral,derivative,kp_straight,ki_straight,kd_straight)
                        right_speed = (fast_speed) - pid_output
                        left_speed = (fast_speed) + pid_output

                        # Ensure motor speeds are within a valid range (0-100)
                        right_speed = max(0,min((fast_speed),right_speed))
                        left_speed = max(0, min((fast_speed), left_speed))
                        drive(right_speed, left_speed)
                    
                    prev_error = error
                    print(right_speed,left_speed,angle,(x_midpoint))
                
               
        if cv2.waitKey(1) & 0xFF == 27:
                break


def display_state4():
    while True:
        ret, frame = cap.read()
        # You can modify the frame or perform different operations for state 2
        # For example, you can apply image processing to the frame
        processed_frame = frame  # Replace this with your actual processing
        cv2.imshow("State 4", processed_frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

def display_state5():
    while True:
        ret, frame = cap.read()
        # You can modify the frame or perform different operations for state 2
        # For example, you can apply image processing to the frame
        processed_frame = frame  # Replace this with your actual processing
        cv2.imshow("State 5", processed_frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break


cap = cv2.VideoCapture(0)
while True:

    if current_state == 1:
        display_state1()
    if current_state ==2:
        display_state2()
    if current_state ==3:
        display_state3(inverted)
    if current_state ==4:
        display_state4()
    if current_state ==5:
        display_state5()
    

            
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()