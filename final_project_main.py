import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
from time import sleep,perf_counter
import os 
import imutils
from state_2 import blue_line_tracking, calculate_angle, calculate_centerline
from state_5 import finding_box, calculate_angle, calculate_centerline, finding_contour

os.environ['QT_QPA_PLATFORM']= 'xcb'

current_state = 3
state_3_state,state_2_state = 'initial','initial'
start_time_state_2,launch_start_time,start_time_state_3 = None,None,None

#STATES DESCRIPTIONS
#S1: DRIVE OFF TABLE
#S2: COMPLETE COURSE AND JUMP ONTO TABLE
#S3: GET INSIDE THE BOX ON TABLE
 

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


def state1():
    global current_state
    print("Driving Off Table")
    drive(-100,-100)
    sleep(2.0)
    drive(0,0)
    sleep(4.0)
    print("Leaving State 1")
    current_state = 2

def state2(frame):
   
   global start_time_state_2
   global current_state
   global state_2_state
   global launch_start_time
   
    
   kp_corner = 1.8# Proportional gain for corners and intersections
   ki_corner=  0.05 # Integral gain for corners and intersections
   kd_corner = 0.6 # Derivative gain for corners and intersections


   kp_straight = 0.7 # Proportional gain for lines
   ki_straight= 0.0  # Integral gain for lines
   kd_straight = 0.4  # Derivative gain for lines

    #Varaibles for On the Ground State Machine
   integral = 0
   prev_error = 0
   error = 0
   backup_count = 0
   backup_state,launch_phase,wrong_way = False, False,False
   slow_speed = 80 #Speed for corners and intersections
   fast_speed =  100 #Speed for lines
   right_corner_count,left_corner_count = 0,1
   direction = ""
   corner_count,turn, intersection_count,back_up_launch = 0,0,0,0
   backup,line_up = False,False

    #Initialize On the Ground State Machine variables
   line_track_return = blue_line_tracking(frame)
   processed_frame = line_track_return[0]
   distance = line_track_return[1]
   line_type = line_track_return[2]
   area = line_track_return[3]
   direction = line_track_return[4]
   line_found,center_point,roi_center = line_track_return[5],line_track_return[6],line_track_return[7]
   right_speed,left_speed= None,None
 

   #On the Ground State Machine
   if processed_frame is not None and processed_frame.shape[0] > 0 and processed_frame.shape[1] > 0:
       
       
       #Caclulate the total time the state machine runs when a line is found
       if state_2_state != 'initial':
           total_time = perf_counter() - start_time_state_2
           #FOR DEBUGGING
           #print(total_time,right_corner_count,left_corner_count)
           #print(state_2_state,turn,backup_count,distance)
           #print(state_2_state,total_time,line_type)
           #print(right_speed,left_speed)
    
       #Start the state specific launch phase timer
       if launch_start_time is None and state_2_state == '3b':
            launch_start_time = perf_counter()

       #Turn to te right until a line is found
       if state_2_state == "initial" and not line_found:
            drive(-slow_speed,slow_speed)
            print("Finding Line: Turning Right")

       #Once a line is found, start the total state machine start time and move to line following state
       elif state_2_state == "initial" and line_found and start_time_state_2 is None:
           start_time_state_2 = perf_counter()
           state_2_state = "1b"     

       #If the robot turns the opposite direction of the track, 
       #Turn around to start following the track in the right directon
       elif state_2_state == "1c":
           turn = 0
           wrong_way = True
           drive(-slow_speed,slow_speed)
           print("Turning")
           sleep(1.4)
           backup_count = 0
           state_2_state = "1b"
       
       #Line following state
       elif state_2_state == "1b" and line_found:
           
           #Once the robot completes the course and is at the end of the line,
           #Move into launch phase
           if turn <= 3 and backup_count >= 10:
               state_2_state = '3a'

           #Count how many times the robot backs up
           if backup_state:
               backup_count +=1
               backup_state = False

           #Triggers the robot to turn around if starts following the track in 
           #The wrong direction
           if backup_count>2 and total_time<5:
               state_2_state = '1c' 
           
           error = distance

           if error is None:
               integral = 0

           else:

            #PID controller implementation             
            integral += error
            derivative = error - prev_error
            pid = lambda error,integral,derivative,kp,ki,kd: kp * error + ki * integral + kd * derivative
            
            #If the robot has not completed the track, take into account all line types and 
            #Follow them accordingly
            if not launch_phase:
               if line_type == "Line":
                   
                   corner_count = 0
                   intersection_count = 0
                   pid_output = pid(error,integral,derivative,kp_straight,ki_straight,kd_straight)
                   right_speed = (fast_speed) - pid_output
                   left_speed = (fast_speed) + pid_output
                   right_speed = max(0,min(fast_speed,right_speed))
                   left_speed = max(0, min(fast_speed, left_speed))
                   drive(right_speed, left_speed)

               elif line_type == "Right Corner" or line_type == "Left Corner":
                   corner_count += 1
                   intersection_count = 0
                   if corner_count >= 9:
                       turn +=1
                       corner_count = 0
                   pid_output = pid(error,integral,derivative,kp_corner,ki_corner,kd_corner)
                   right_speed = (slow_speed) - pid_output
                   left_speed = (slow_speed) + pid_output
                   right_speed = max(0,min((slow_speed),right_speed))
                   left_speed = max(0, min((slow_speed), left_speed))
                   drive(right_speed, left_speed)

               elif line_type == "Intersection":
                   corner_count = 0
                   intersection_count += 1
                   if turn >= 4 and intersection_count >= 3:
                        state_2_state = '2'
                        intersection_count = 0
                   if direction == "Left":
                       right_speed,left_speed = slow_speed,-slow_speed
                   else:
                      right_speed,left_speed = -slow_speed,slow_speed
                   drive(right_speed,left_speed)
               #print(right_speed,left_speed)
            else:
                   #If the robot has completed the track, just follow the line withut accounting for line type
                   pid_output = pid(error,integral,derivative,kp_straight,ki_straight,kd_straight)
                   right_speed = (fast_speed) - pid_output
                   left_speed = (fast_speed) + pid_output
                   right_speed = max(0,min((fast_speed),right_speed))
                   left_speed = max(0, min((fast_speed), left_speed))
                   drive(right_speed,left_speed)
            prev_error = error
            #print(right_speed,left_speed,line_type,total_time)

       #Once the course is completed and the final intersection is encoutered,
       #Make sure te robot turns down the intersection
       elif state_2_state == "2":
           launch_phase = True
           if direction == "Left":
                right_speed,left_speed = slow_speed,-slow_speed                 
           else: 
                right_speed,left_speed = -slow_speed,slow_speed
           print(f'Turning: {direction}')
           drive(right_speed,left_speed)
           if line_type == "Line":
             state_2_state = '1b'
       #Launch Phase that lines robot up as staright as possible and backs up to 1.5 ft
       #Away from the end of the line
       elif state_2_state == '3a':   
            if direction == "Right":
                right_speed,left_speed = -slow_speed,slow_speed
            else:
                right_speed,left_speed = slow_speed,-slow_speed
            drive(right_speed,left_speed)
            sleep(0.2)
            drive(-slow_speed,-slow_speed)
            sleep(1.0)
            print("Ready to Launch ")
            current_state = 3
            
        
       #If the robot loses the line while following the course,
       #Backup until Line is Found
       else:
          if launch_phase:
                state_2_state = '3a' 
        
          backup_state = True
          drive(-slow_speed,-slow_speed)
          print("Backing Up")

"""
def state3_test(frame):   
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
 """                   

def state3(frame):
   global current_state
   global start_time_state_3 
   global state_3_state 

   kp_box = 1.7# Proportional gain for corners and cornering lines
   ki_box =  0.05 # Integral gain for corners and cornering lines
   kd_box = 0.2 # Derivative gain for corners and cornering lines


   integral = 0
   prev_error = 0
   error = 0


   slow_speed = 50#Speed for corners, cornering lines and intersections
   edge_detected = 0

   line_track_return = finding_box(frame,state_3_state)
   processed_frame = line_track_return[0]
   area = line_track_return[1]
   line_found = line_track_return[2]
   line_type = line_track_return[3]
   distance = line_track_return[4]
   edge_detector = line_track_return[5]
   right_speed,left_speed = None,None

  
   if processed_frame is not None and processed_frame.shape[0] > 0 and processed_frame.shape[1] > 0:
      
       print(state_3_state)
       if state_3_state == "initial" and not line_found:
           #drive(-70,70)
           if start_time_state_3 is None:
              start_time_state_3 = perf_counter()
           if edge_detector >= 1:
               drive(slow_speed,-slow_speed)
               print("Finding Line: Turning Left")
           else:
               drive(-slow_speed,slow_speed)
               print("Finding Line: Turning Right")
           
           elapsed_time = (perf_counter() - start_time_state_3)
           if elapsed_time > 3:
               state_3_state = "done"
           print(elapsed_time)
           
       elif state_3_state == "initial" and line_found:
           state_3_state = "2"      
       elif state_3_state == "2" and line_type == "Line":
           error = distance
           if error is None:
               integral = 0
                   # Rest of your PID controller logic
           else:
              
               integral += error
               derivative = error - prev_error
               pid_output = kp_box * error + ki_box * integral + kd_box * derivative
               right_speed = (slow_speed) - pid_output
               left_speed = (slow_speed) + pid_output
               # Ensure motor speeds are within a valid range (0-100)
               right_speed = max(0,min((slow_speed),right_speed))
               left_speed = max(0, min((slow_speed), left_speed))
               print(right_speed,left_speed)
               drive(right_speed, left_speed)
       elif state_3_state == "2" and line_type == "Right Corner":
           state_3_state = "3a"     
       elif state_3_state == "2" and line_type == "Left Corner":
           state_3_state = "3b" 
       elif state_3_state == "3a":          
           if line_found:
             drive(-slow_speed,slow_speed)
           else:
               drive(-slow_speed,slow_speed)
               sleep(0.3)
               state_3_state = "4"
           print("Turning Right")
       elif state_3_state == "3b":
           if line_found:
             drive(slow_speed,-slow_speed)
           else:
               drive(slow_speed,-slow_speed)
               sleep(0.3)
               state_3_state = "4"
           print("Turning Left")
       elif state_3_state == "4":
           drive(slow_speed,slow_speed)
           sleep(0.4)
           state_3_state = "done"
       elif state_3_state == "done":
           print("In Box!")
           current_state = 4
       else:
           drive(-slow_speed,-slow_speed)
           print("Backing Up")

    

cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,280 )
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)


while True:
    
    ret, frame = cap.read()
    if not ret:
        break

    frame = imutils.resize(frame,width=280,height = 300)

    if current_state == 1:
        state1()
    if current_state ==2:
        state2(frame)
    if current_state ==3:
        state3(frame)
    if current_state ==4:
        break

    

    print(current_state) 
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()