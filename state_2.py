import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
from time import sleep
import os
from time import sleep,perf_counter
state_3_state = "initial"
start_time_state_3,launch_start_time = None,None


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

"""
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

#Initalize PWM Signal At 0
pwm_a.start(0)
pwm_b.start(0)

#L289N Motor Driver Code
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
"""

#Calculate centerline of contour by using moments
def calculate_centerline(contours):
   moments = cv2.moments(contours)
   if moments["m00"] != 0:
       cx = int(moments["m10"] / moments["m00"])
       cy = int(moments["m01"] / moments["m00"])
       return cx,cy
   return None


#Caclulate angle based on two points
def calculate_angle(A, B):
   angle_radians = math.atan2(B[1]-A[1], B[0]-A[0])
   angle_degrees = math.degrees(angle_radians)
   angle_degrees *=  -1
   if angle_degrees < 0:
       angle_degrees += 360
   return angle_degrees
      


#Track blue line(state 3)
def blue_line_tracking(frame):
  roi = frame
  roi_height, roi_width = roi.shape[:2]     #Get poistion of center of current frame
  roi_center = (roi_width // 2, roi_height // 2)  # Fix the order of width and height
  hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)   #Convert frame to HSV color space

  #Define upper and lower value of line color for mask
  #Specific values for competition lighting
  #Taken at 10:44 AM November 21 2023
  #lower_blue = np.array([90, 70, 80])
  #upper_blue = np.array([130, 255, 255])

  #Define upper and lower value of line color for mask
  #Generally good values for all lghting conditions
  lower_blue = np.array([90, 100, 100])
  upper_blue = np.array([130, 255, 255])

  # Create a mask to isolate the blue color
  mask = cv2.inRange(hsv, lower_blue, upper_blue)
 
  #Peform morphological operations in order to remove small features findContours might pick up and smooth main features to blobs
  #So it is easier to find the contours
  kernel = np.ones((3,3), np.uint8) #We typically use 3x3 as the output will be smaller than the i
  mask = cv2.erode(mask, kernel, iterations=3+2)
  mask = cv2.dilate(mask, kernel, iterations=9)
 
  #Find contours
  contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
 
  #Initalize variables
  global right_corner_count
  global left_corner_count
  global direction
  center_point,error,angle,x_midpoint,y_midpoint,highest_point,box,line_found = None,None,None,None,None,None,None,None
  largest_area = 1
  line_type = ""
  max_distance_threshold = 200.0
  contours_list,approx= [],[]
  highest_point = (0,0)
  highest_distance = 0

  #Filter out contours by area to reduce noise
  for cnt in contours:
      area_c = cv2.contourArea(cnt)
      if area_c > 1000.0 and area_c < 37000.0:
          contours_list.append(cnt)

  if contours_list:  # Check if contours list is not empty
     
      # Find the largest contour and approximate the shape of the contour
      largest_contour = max(contours_list, key=cv2.contourArea)     
      largest_area = cv2.contourArea(largest_contour)
      approx = cv2.approxPolyDP(largest_contour,0.01*cv2.arcLength(largest_contour,True),True)
     
      #Classify lines based on how many vertcies approx returns
      if len(approx) <= 5:
           line_type = "Line"
      elif 6 <= len(approx) <= 7:
           line_type = "Corner"
      else:
           line_type = "Intersection"

      #Find minimum area rectangle of the largest contour
      blackbox = cv2.minAreaRect(largest_contour)
      box = cv2.boxPoints(blackbox)
      box = np.intp(box)

      #Find centorid of largest contour
      center_point = calculate_centerline(largest_contour)

      #Find vector tip for lines
      #Find furthest point away from the centeroid that is at most 200.0 pixels away
      if line_type == 'Line':            
          for point in largest_contour:
                  distance = np.linalg.norm(point - center_point)
                  if distance > max_distance_threshold:
                      continue  
                  if distance > highest_distance:
                      highest_point = point[0] 
    
      #Find vector tip for corners and intersections 
      #Find the furthest point away from the centroid between 
      #The two verticies that are less than the centorid on the y-axis
      if line_type == 'Corner' or line_type == "Intersection":            
          for point in box:
              if point[1] < center_point[1]:
                   distance = math.sqrt((point[0] - center_point[0])**2 + (point[1] - center_point[1])**2)
                   if distance > highest_distance:
                       highest_point = point
      
      #Decide which way the robot should turn when it encounters the 
      #Final intersecton based on the most frewuent corner direction
      if right_corner_count < left_corner_count:
            direction = "Right"
      else:
            direction = "Left"
      
      #Calculate the mid point of the line vector
      #center_point is the tail and highest_point is the tip of the vector
      if highest_point is not None:  
            x_midpoint = int((center_point[0] + highest_point[0]) // 2)         
            y_midpoint = int((center_point[1] + highest_point[1]) // 2)
            angle = int(calculate_angle(center_point,highest_point))

     
      #Define direction of corner based on angle of vector
      if angle > 90 and line_type == "Corner":
          line_type = "Left " + line_type
          left_corner_count += 1
      if 90 > angle and line_type == "Corner":
          line_type = "Right " + line_type
          right_corner_count += 1
         


      #If centorid of line can be calculated
      if center_point:
          line_found = True

          #CODE FOR DEBUGGING ON IMREAD WINDOW
          cv2.circle(roi, (roi_center[0],roi_center[1]), 10,(255, 255, 255),-1)
          #cv2.circle(roi, (center_point[0],center_point[1]), 4, (0, 0, 255),-1)
          cv2.circle(roi,(x_midpoint,y_midpoint), 10, (0, 0, 255), -1)
          cv2.arrowedLine(roi,(center_point[0],center_point[1]),highest_point,(60,255,50),10)
          cv2.drawContours(roi, [largest_contour], -1, (255, 255, 0), 5)  # You can change the color and thickness
          cv2.putText(roi,f'Angle: {str(angle)}',(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
          cv2.putText(roi,f'Line Type: {line_type}',(10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
          
          #Horizontal shift is calculated for PID loop     
          if highest_point is not None:  
                error = x_midpoint - roi_center[0]

   #Return necessary variables for On the Ground State Machine             
  return roi, error, line_type, largest_area, direction, line_found,right_corner_count,left_corner_count



"""
#FOR TESTING AND DEBUGING
cap = cv2.VideoCapture(0)




while True:
   ret, frame = cap.read()
   if not ret:
       break
   frame = imutils.resize(frame,width=280,height = 300)
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
       cv2.imshow("Blue Tape Detection", processed_frame)
       
       #Caclulate the total time the state machine runs when a line is found
       if state_3_state != 'initial':
           total_time = perf_counter() - start_time_state_3
           #FOR DEBUGGING
           #print(total_time,right_corner_count,left_corner_count)
           #print(state_3_state,turn,backup_count,distance)
           #print(state_3_state,total_time,line_type)
           #print(right_speed,left_speed)
    
       #Start the state specific launch phase timer
       if launch_start_time is None and state_3_state == '3b':
            launch_start_time = perf_counter()

       #Turn to te right until a line is found
       if state_3_state == "initial" and not line_found:
            drive(-slow_speed,slow_speed)
            print("Finding Line: Turning Right")

       #Once a line is found, start the total state machine start time and move to line following state
       elif state_3_state == "initial" and line_found and start_time_state_3 is None:
           start_time_state_3 = perf_counter()
           state_3_state = "1b"     

       #If the robot turns the opposite direction of the track, 
       #Turn around to start following the track in the right directon
       elif state_3_state == "1c":
           turn = 0
           wrong_way = True
           drive(-slow_speed,slow_speed)
           print("Turning")
           sleep(1.4)
           backup_count = 0
           state_3_state = "1b"
       
       #Line following state
       elif state_3_state == "1b" and line_found:
           
           #Once the robot completes the course and is at the end of the line,
           #Move into launch phase
           if turn <= 3 and backup_count >= 10:
               state_3_state = '3a'

           #Count how many times the robot backs up
           if backup_state:
               backup_count +=1
               backup_state = False

           #Triggers the robot to turn around if starts following the track in 
           #The wrong direction
           if backup_count>2 and total_time<5:
               state_3_state = '1c' 
           
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
                        state_3_state = '2'
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
       elif state_3_state == "2":
           launch_phase = True
           if direction == "Left":
                right_speed,left_speed = slow_speed,-slow_speed                 
           else: 
                right_speed,left_speed = -slow_speed,slow_speed
           print(f'Turning: {direction}')
           drive(right_speed,left_speed)
           if line_type == "Line":
             state_3_state = '1b'
       #Launch Phase that lines robot up as staright as possible and backs up to 1.5 ft
       #Away from the end of the line
       elif state_3_state == '3a':   
            if direction == "Right":
                right_speed,left_speed = -slow_speed,slow_speed
            else:
                right_speed,left_speed = slow_speed,-slow_speed
            drive(right_speed,left_speed)
            sleep(0.2)
            drive(-slow_speed,-slow_speed)
            sleep(1.0)
            print("Ready to Launch ")
            break
        
       #If the robot loses the line while following the course,
       #Backup until Line is Found
       else:
          if launch_phase:
                state_3_state = '3a' 
        
          backup_state = True
          drive(-slow_speed,-slow_speed)
          print("Backing Up")
               
   if cv2.waitKey(1) & 0xFF == 27:
       break


cap.release()
cv2.destroyAllWindows()
"""
