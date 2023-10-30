import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
from time import sleep
import os 



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

def calculate_centerline(contours):
    moments = cv2.moments(contours)
    if moments["m00"] != 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        return cx,cy
    return None
    
def calculate_angle(A, B):
    angle_radians = math.atan2(B[1]-A[1], B[0]-A[0])
    angle_degrees = math.degrees(angle_radians)

    return angle_degrees * -1
        
def order_box(box):
    srt = np.argsort(box[:, 1])
    btm1 = box[srt[0]]
    btm2 = box[srt[1]]

    top1 = box[srt[2]]
    top2 = box[srt[3]]

    bc = btm1[0] < btm2[0]
    btm_l = btm1 if bc else btm2
    btm_r = btm2 if bc else btm1

    tc = top1[0] < top2[0]
    top_l = top1 if tc else top2
    top_r = top2 if tc else top1

    return np.array([top_l, top_r, btm_r, btm_l])


def calc_line(x1, y1, x2, y2):
    a = float(y2 - y1) / (x2 - x1) if x2 != x1 else 0
    b = y1 - a * x1
    return a, b

def calc_line_length(p1, p2):
    dx = (p1[0] - p2[0])//2
    dy = (p1[1] - p2[1])//2
    return math.sqrt(dx * dx + dy * dy)

def get_vert_angle(p1, p2, w, h):
    px1 = p1[0] - w/2
    px2 = p2[0] - w/2
    
    py1 = h - p1[1]
    py2 = h - p2[1]

    angle = 90
    if px1 != px2:
        a, b = calc_line(px1, py1, px2, py2)
        angle = 0
        if a != 0:
            x0 = -b/a
            y1 = 1.0
            x1 = (y1 - b) / a
            dx = x1 - x0
            tg = y1 * y1 / dx / dx
            angle = 180 * np.arctan(tg) / np.pi
            if a < 0:
                angle = 180 - angle
    return angle

def calc_box_vector(box):
    v_side = calc_line_length(box[0], box[3])
    h_side = calc_line_length(box[0], box[1])
    idx = [0, 1, 2, 3]
    if v_side < h_side:
        idx = [0, 3, 1, 2]
    return ((box[idx[0]][0] + box[idx[1]][0]) / 2, (box[idx[0]][1] + box[idx[1]][1]) / 2), ((box[idx[2]][0] + box[idx[3]][0]) / 2, (box[idx[2]][1]  +box[idx[3]][1]) / 2)



def blue_line_tracking(frame):
    roi = frame
    """
    # Define the height of the ROI as a percentage of the image height
    roi_height_percentage =  100 # Adjust as needed

    # Calculate the ROI coordinates
    roi_top_left = (0, image_height - (image_height * roi_height_percentage // 100))
    roi_bottom_right = (image_width, image_height)

    # Create the ROI by slicing the image
    roi = frame[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]
    """
    roi_height, roi_width = roi.shape[:2]
    
    roi_center = (roi_width // 2, roi_height // 2)  # Fix the order of width and height
    
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)



    # Define the lower and upper blue color thresholds in HSV
    lower_blue = np.array([100, 130, 130]) #Value tuned for Senior Design Lab
    upper_blue = np.array([110, 255, 255]) #Value tuned for Senior Design Lab

    #lower_blue = np.array([90, 50, 50])
    #upper_blue = np.array([130, 255, 255])

    # Create a mask to isolate the blue color
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=5)
    mask = cv2.dilate(mask, kernel, iterations=9)	
   

    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centerline,error,angle,p1,p2,x_midpoint = None,None,None,None,None,None


    if contours:  # Check if contours list is not empty
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        # Calculate the centerline of the largest contour
        blackbox = cv2.minAreaRect(largest_contour) 
        box = cv2.boxPoints(blackbox)
        box = np.intp(box)
        box = order_box(box)
        centerline = calculate_centerline(largest_contour)
        p1 ,p2 = calc_box_vector(box)
        #angle = get_vert_angle(p1,p2,w,h)
        x_1,y_1,x_2,y_2 = int(p1[0]),int(p1[1]),int(p2[0]),int(p2[1])
        x_midpoint,y_midpoint = ((x_1 + x_2)//2),((y_1 +y_2)//2)
        # Filter the contours
        p1 = (int(p1[0]), int(p1[1]))
        p2 = (int(p2[0]), int(p2[1]))
        angle = calculate_angle(p1,p2)
        angle = int(angle)
        if centerline:
            cv2.circle(roi, (roi_center[0],centerline[1]), 4,(255, 255, 255),-1)
            cv2.circle(roi, (centerline[0],centerline[1]), 4, (0, 0, 255),-1)
            #cv2.circle(roi, (x_midpoint,y_midpoint), 4, (0, 0, 255),-1)
            cv2.drawContours(roi, [largest_contour], -1, (255, 255, 0), 10)  # You can change the color and thickness
            #cv2.drawContours(roi, [box], -1, (0, 255, 0), 10)  # You can change the color and thickness
            cv2.line(roi , p1, p2, (255,0,0),3)
            cv2.putText(roi,str(angle),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
             # Get the rotated rectangles for the largest contour
             
            error = (x_midpoint) - roi_center[0]
         
            # Display the distance as text on the image
            font = cv2.FONT_HERSHEY_SIMPLEX
           

           
    if not(contours):
        error = -500
    return roi, error,angle, x_midpoint

"""
# Initialize camera (or load an image)
cap = cv2.VideoCapture(0)


while True:
    ret, frame = cap.read()
    if not ret:
        break
    # Check if the robot is upside down (replace this condition with your detection logic)
    robot_is_upside_down = True  # Replace with your actual condition

    if robot_is_upside_down:
        # Rotate the frame by 180 degrees
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    # Show the image with the blue tape centerline and the center of the image
    line_track_return = blue_line_tracking(frame)
    processed_frame = line_track_return[0]
    distance = line_track_return[1]
    angle = line_track_return[2]
    right_speed,left_speed = None,None
    #p1,p2 =  line_track_return[3], line_track_return[4]
    x_midpoint = line_track_return[3]

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

cap.release()
cv2.destroyAllWindows()
"""
