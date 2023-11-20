import cv2
import numpy as np
import math
from time import sleep,perf_counter
import RPi.GPIO as GPIO
import os
import imutils



kp_box = 1.7# Proportional gain for corners and cornering lines
ki_box =  0.05 # Integral gain for corners and cornering lines
kd_box = 0.2 # Derivative gain for corners and cornering lines


integral = 0
prev_error = 0
error = 0


slow_speed = 50#Speed for corners, cornering lines and intersections
fast_speed =  100 #Speed for corners, cornering lines and intersections
state = "initial"
edge_detected = 0
start_time_state_5 = None

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
      
def finding_contour(lower,upper,frame,erode_iteration,dilate_iteration):
  #Create a mask to isolate the blue color
  mask = cv2.inRange(frame, lower, upper)


  #Peform morphological operations in order to remove small features findContours might pick up and smooth main features to blobs
  #So it is easier to find the contours
  kernel = np.ones((3,3), np.uint8) #We typically use 3x3 as the output will be smaller than the i
  mask= cv2.erode(mask, kernel, iterations=erode_iteration)
  mask = cv2.dilate(mask, kernel, iterations=dilate_iteration)
  contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  return contours




#Track blue line(state 3)
def finding_box(frame,state):
  roi = frame
  global edge_detected

  roi_height, roi_width = roi.shape[:2]
   #Get poistion of center of current frame
  roi_center = (roi_width // 2, roi_height // 2)  # Fix the order of width and height
   #Convert frame to HSV color space
  hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)




  #Define upper and lower value of line color for mask
  lower_blue = np.array([100, 190, 120])
  upper_blue = np.array([130, 255, 255])
 
  #lower_blue = np.array([90, 100, 100])
  #upper_blue = np.array([130, 255, 255])
   #Define upper and lower value of line color for mask
  #lower_blue = np.array([90, 122, 0])
  #upper_blue = np.array([130, 255, 255])


  # Define the HSV range for light brown color
  lower_brown = np.array([0, 0, 0])  
  upper_brown = np.array([80, 255, 255])


 
  #Initalize state 3 variables
  center_point, line_found, highest_point,angle,x_midpoint,y_midpoint,error = None,None,None,None,None,None,None
  largest_area,largest_brown_area = 1,1
  line_type = "","",""
  line_type, direction = "",""
  max_distance_threshold = 200.0
  contours_list,approx = [],[]
  highest_point = (0,0)
  highest_distance = 0


  if state == "initial":
      blue_contours = finding_contour(lower_blue,upper_blue,hsv,15,19)
      brown_contours = finding_contour(lower_brown,upper_brown,hsv,15,19)
          # Find the largest contour
      if brown_contours:
       largest_contour = max(brown_contours, key=cv2.contourArea)     
       largest_brown_area = cv2.contourArea(largest_contour)
       if largest_brown_area <12000.0:
           edge_detected += 1
       cv2.drawContours(roi, [brown_contours[0]], -1, (0, 75, 150), 2)
  else:
       blue_contours = finding_contour(lower_blue,upper_blue,hsv,4,8)
  #Filter out contours by area to reduce noise
  for cnt in blue_contours:
      area_c = cv2.contourArea(cnt)
      if area_c > 0.0 and area_c < 30000.0:
          contours_list.append(cnt)
 
  if contours_list:  # Check if contours list is not empty    
      # Find the largest contour
      largest_contour = max(contours_list, key=cv2.contourArea)     
      largest_area = cv2.contourArea(largest_contour)
     
      #Find minimum area rectangle of the largest contour
      blackbox = cv2.minAreaRect(largest_contour)
      box = cv2.boxPoints(blackbox)
      box = np.intp(box)


      #Find centorid of contour
      center_point = calculate_centerline(largest_contour)

      approx = cv2.approxPolyDP(largest_contour,0.01*cv2.arcLength(largest_contour,True),True)


      #Classify lines based on area
      if len(approx) <= 7:
          line_type = "Line"
      else:
          line_type = "Corner"


      #Create vector for straight and cornering lines
      
      if line_type == "Line":            
          for point in largest_contour:
                  distance = np.linalg.norm(point - center_point)
                  if distance > max_distance_threshold:
                      continue  # Skip points that are beyond the maximum distance
                  if distance > highest_distance:
                      highest_point = point[0] 
    
      #Create vector for corner
      if line_type == "Corner":            
          for point in box:
              # Check if the point is in front of the center point in the y-direction
              if point[1] < center_point[1]:
                  # Check if this point is further away than the previous highest point
                   distance = math.sqrt((point[0] - center_point[0])**2 + (point[1] - center_point[1])**2)
                   if distance > highest_distance:
                       highest_point = point
                       highest_distance = distance
      
      #Calculate the mid point of the line vector
      if highest_point is not None:  # Check if highest_point is not None before accessing its coordinates
           x_midpoint = int((center_point[0] + highest_point[0]) // 2)         
           y_midpoint = int((center_point[1] + highest_point[1]) // 2)
           angle = int(calculate_angle(center_point,highest_point))
     
      #Define direction of corner based on angle
      if angle > 90 and line_type == "Corner":
          line_type = "Left " + line_type
      if 90 > angle and line_type == "Corner":
          line_type = "Right " + line_type
         
      if center_point:
          line_found = True
          #CODE FOR DEBUGGING

          # Calculate the distance from the point to the closest point on the contour
          cv2.circle(roi, (roi_center[0],roi_center[1]), 10,(255, 255, 255),-1)
          cv2.circle(roi,(x_midpoint,y_midpoint), 10, (0, 0, 255), -1)
          cv2.drawContours(roi, [largest_contour], -1, (255, 255, 0), 10)  # You can change the color and thickness
          cv2.line(roi , center_point, highest_point, (255,0,0),3)
          cv2.putText(roi,f'Angle: {str(angle)}',(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
          
          error = x_midpoint - roi_center[0]
         
 
                 
  if not(contours_list) and state != "initial":
   line_type = "Finding Line"
   line_found = False
   error = -500
  #return roi,largest_area,line_type,error,highest_point, center_point
  return roi, largest_area, line_found,line_type,error,edge_detected, len(approx)




#FOR TESTING AND DEBUGING
cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,280)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)



while True:
   ret, frame = cap.read()
   if not ret:
       break
   frame = imutils.resize(frame,width=280,height = 300)
   line_track_return = finding_box(frame,state)
   processed_frame = line_track_return[0]
   area = line_track_return[1]
   line_found = line_track_return[2]
   line_type = line_track_return[3]
   distance = line_track_return[4]
   edge_detector = line_track_return[5]
   approx = line_track_return[6]
   right_speed,left_speed = None,None
   #global start_time_state_5
  
   if processed_frame is not None and processed_frame.shape[0] > 0 and processed_frame.shape[1] > 0:
       cv2.imshow("Blue Tape Detection", processed_frame)
       print(approx)
       if state == "initial" and not line_found:
           #drive(-70,70)
           if start_time_state_5 is None:
              start_time_state_5 = perf_counter()
           if edge_detector >= 1:
               #drive(slow_speed,-slow_speed)
               print("Finding Line: Turning Left")
           else:
               #drive(-slow_speed,slow_speed)
               print("Finding Line: Turning Right")
           
           elapsed_time = (perf_counter() - start_time_state_5)
           if elapsed_time > 3:
               state = "done"
           
       elif state == "initial" and line_found:
           state = "2"      
       elif state == "2" and line_type == "Line":
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
               #drive(right_speed, left_speed)
       elif state == "2" and line_type == "Right Corner":
           state = "3a"     
       elif state == "2" and line_type == "Left Corner":
           state = "3b" 
       elif state == "3a":          
           if line_found:
             #drive(-slow_speed,slow_speed)
             print("Turning Right")
           else:
               #drive(-slow_speed,slow_speed)
               sleep(0.3)
               state = "4"
           print("Turning Right")
       elif state == "3b":
           if line_found:
             #drive(slow_speed,-slow_speed)
             print("Turning Left")
           else:
               #drive(slow_speed,-slow_speed)
               sleep(0.3)
               state = "4"
           print("Turning Left")
       elif state == "4":
           #drive(slow_speed,slow_speed)
           sleep(0.4)
           state = "done"
       elif state == "done":
           print("In Box!")
           break
       else:
           #drive(-slow_speed,-slow_speed)
           print("Backing Up")

       
   if cv2.waitKey(1) & 0xFF == 27:
       break
  
cap.release()
cv2.destroyAllWindows()
