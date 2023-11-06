import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
from time import sleep
import os 


"""
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
fast_speed =  100 #Speed for corners, cornering lines and intersections
intersection_speed = 50 #Speed for intersections


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

   roi_height, roi_width = roi.shape[:2]
  
   #Get poistion of center of current frame
   roi_center = (roi_width // 2, roi_height // 2)  # Fix the order of width and height
  
   #Convert frame to HSV color space
   hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)


   #Define upper and lower value of line color for mask
   lower_blue = np.array([90, 70, 0])
   upper_blue = np.array([130, 255, 255])


   # Create a mask to isolate the blue color
   mask = cv2.inRange(hsv, lower_blue, upper_blue)
   
   #Peform morphological operations in order to remove small features findContours might pick up and smooth main features to blobs
   #So it is easier to find the contours
   kernel = np.ones((3,3), np.uint8) #We typically use 3x3 as the output will be smaller than the i
   mask = cv2.erode(mask, kernel, iterations=5)
   mask = cv2.dilate(mask, kernel, iterations=9)
   
   #Find contours
   contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   
   #Initalize state 3 variables
   center_point,error,angle,x_midpoint,y_midpoint,highest_point,box = None,None,None,None,None,None,None
   largest_area,right_corner_count,left_corner_count,intersection_count = 1,0,1,0
   line_type,direction = "",""
   max_distance_threshold = 200.0
   contours_list = []
   highest_point = (0,0)
   highest_distance = 0
   state_4_transition = False
   
   #Filter out contours by area to reduce noise 
   for cnt in contours:
       area_c = cv2.contourArea(cnt)
       if area_c > 2000.0 and area_c < 44000.0:
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

       #Classify lines based on area
       if largest_area < 20200.0 and largest_area >16000.0:
           line_type = "Straight Line"
       elif largest_area < 27500.0 and largest_area>20200.0:
           line_type = "Corner"
       elif largest_area > 27000.0:
           line_type = "Intersection"
       else:
           line_type = "Cornering Line"

       #Create vector for straight and cornering lines 
       if line_type == 'Straight Line' or line_type == 'Cornering Line':             
           for point in largest_contour:
                   distance = np.linalg.norm(point - center_point)
                   if distance > max_distance_threshold:
                       continue  # Skip points that are beyond the maximum distance
                   if distance > highest_distance:
                       highest_point = point[0]  
      
       #Create vector for corner
       if line_type == 'Corner':             
           for point in box:
               # Check if the point is in front of the center point in the y-direction
               if point[1] < center_point[1]:
                   # Check if this point is further away than the previous highest point
                    distance = math.sqrt((point[0] - center_point[0])**2 + (point[1] - center_point[1])**2)
                    if distance > highest_distance:
                        highest_point = point
                        highest_distance = distance
    
       #Create vector for intersections
       if line_type == 'Intersection':   
            highest_point = roi_center
            angle = 0
            center_point = roi_center
            intersection_count = intersection_count + 1
            if right_corner_count < left_corner_count:
                direction = "Right"
            else:
                direction = "Left"
        
        
       #Calculate the mid point of the line vector
       if highest_point is not None:  # Check if highest_point is not None before accessing its coordinates
            x_midpoint = int((center_point[0] + highest_point[0]) // 2)          
            y_midpoint = int((center_point[1] + highest_point[1]) // 2)
            angle = int(calculate_angle(center_point,highest_point))
       
       #Define direction of corner based on angle
       if angle > 90 and line_type == "Corner":
           line_type = "Left " + line_type
           left_corner_count += 1
       if 90 > angle and line_type == "Corner":
           line_type = "Right " + line_type
           right_corner_count += 1
           

       #If centorid of line can be calculated
       if center_point:
           """
           CODE FOR DEBUGGING
           cv2.circle(roi, (roi_center[0],roi_center[1]), 10,(255, 255, 255),-1)
           #cv2.circle(roi, (box[0][0],box[0][1]), 10,(255, 255, 255),-1)
           #cv2.circle(roi, (box[1][0],box[1][1]), 10,(50, 50, 50),-1)
           #cv2.circle(roi, (box[2][0],box[2][1]), 10,(255, 255, 0),-1)
           #cv2.circle(roi, (box[3][0],box[3][1]), 10,(0, 255, 255),-1)
           cv2.circle(roi, (center_point[0],center_point[1]), 4, (0, 0, 255),-1)
           cv2.circle(roi,(x_midpoint,y_midpoint), 10, (0, 0, 255), -1)
           #cv2.circle(roi,highest_point, 10, (255, 0, 255), -1)
           #cv2.circle(roi,reflected_point, 40, (255, 255, 0), -1)
           cv2.line(roi,(center_point[0],center_point[1]),highest_point,(255,60,50),10)
           cv2.drawContours(roi, [largest_contour], -1, (255, 255, 0), 10)  # You can change the color and thickness
           #cv2.drawContours(roi, [box], -1, (0, 255, 0), 10)  # You can changecv2.circle(roi, (roi_center[0],roi_center[1]), 10,(255, 255, 255),-1) the color and thickness
           #cv2.line(roi , p1, p2, (255,0,0),3)
           cv2.putText(roi,f'Angle: {str(angle)}',(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
           cv2.putText(roi,f'Line Type: {line_type}',(10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
           """
           
           if highest_point is not None:  # Check if highest_point is not None before accessing its coordinates
                #Horizontal shift is calculated for PID loop
                error = x_midpoint - roi_center[0]
           
   
                   
   if not(contours_list):
       error = -500 #This is returned if the line is not found
       corner_count = max(right_corner_count,left_corner_count)
       if intersection_count > 0 and corner_count > 4:
            state_4_transition = True
   #return roi,largest_area,line_type,error,highest_point, center_point
   return roi,error,line_type, largest_area,direction,angle,state_4_transition,intersection_count

"""
FOR TESTING AND DEBUGING

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,280 )
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)



while True:
    ret, frame = cap.read()
    if not ret:
        break
    # Check if the robot is upside down (replace this condition with your detection logic)
    robot_is_upside_down = False  # Replace with your actual condition

    if robot_is_upside_down:
        # Rotate the frame by 180 degrees
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    # Show the image with the blue tape centerline and the center of the image
    line_track_return = blue_line_tracking(frame)
    processed_frame = line_track_return[0]
    distance = line_track_return[1]
    line_type = line_track_return[2]
    area = line_track_return[3]
    direction = line_track_return[4]
    right_speed,left_speed,angle = None,None,line_track_return[5]
   
 

    if processed_frame is not None and processed_frame.shape[0] > 0 and processed_frame.shape[1] > 0:
        cv2.imshow("Blue Tape Detection", processed_frame)
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
                if line_type == "Right Corner" or line_type == "Cornering Line" or line_type == "Left Corner":
                    pid_output = pid(error,integral,derivative,kp_corner,ki_corner,kd_corner)
                    right_speed = (slow_speed) - pid_output
                    left_speed = (slow_speed) + pid_output

                    # Ensure motor speeds are within a valid range (0-100)
                    right_speed = max(0,min((slow_speed),right_speed))
                    left_speed = max(0, min((slow_speed), left_speed))
                    drive(right_speed, left_speed)
                
                elif line_type == "Straight Line":
                    pid_output = pid(error,integral,derivative,kp_straight,ki_straight,kd_straight)
                    right_speed = (fast_speed) - pid_output
                    left_speed = (fast_speed) + pid_output

                    # Ensure motor speeds are within a valid range (0-100)
                    right_speed = max(0,min((fast_speed),right_speed))
                    left_speed = max(0, min((fast_speed), left_speed))
                    drive(right_speed, left_speed)
                
                else:
                    if direction == "Left":
                        right_speed,left_speed = -1,intersection_speed
                        drive(right_speed,left_speed)
                    
                    right_speed,left_speed = intersection_speed,-1
                    drive(right_speed,left_speed)
                    print(direction)
                    
                
                prev_error = error
                print(right_speed,left_speed,angle,line_type,area)
               
               


            
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
"""