import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
from time import sleep
import os 

os.environ['QT_QPA_PLATFORM']= 'xcb'


kp = 2.5 # Proportional gain
ki = 0.0  # Integral gain
kd = 1.1  # Derivative gain
integral = 0
prev_error = 0
error = 0 

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
    


        

def blue_line_tracking(frame):
    distance = None
    image_height, image_width, _ = frame.shape

    # Define the height of the ROI as a percentage of the image height
    roi_height_percentage =    75 # Adjust as needed

    # Calculate the ROI coordinates
    roi_top_left = (0, image_height - (image_height * roi_height_percentage // 100))
    roi_bottom_right = (image_width, image_height)

    # Create the ROI by slicing the image
    roi = frame[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]
    roi_height, roi_width = roi.shape[:2]
    roi_center = (roi_width // 2, roi_height // 2)  # Fix the order of width and height
    right_roi = (roi_width // 2+ 310, roi_height // 2 + 50)  # Fix the order of width and height
    left_roi = (roi_width // 50, roi_height // 2 + 50)  # Fix the order of width and height
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    right_hsv = hsv[right_roi[1],right_roi[0]]
    left_hsv = hsv[left_roi[1],left_roi[0]]


    # Define the lower and upper blue color thresholds in HSV
    #lower_blue = np.array([100, 130, 130]) #Value tuned for Senior Design Lab
    #upper_blue = np.array([110, 255, 255]) #Value tuned for Senior Design Lab

    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Create a mask to isolate the blue color
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
   

    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centerline = None  # Initialize centerline as None
    horizontal_distance =None
    right_detector,left_detector = None, None

    if contours:  # Check if contours list is not empty
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the centerline of the largest contour
        centerline = calculate_centerline(largest_contour)
        # Filter the contours
    
            
        if centerline:
            cv2.circle(roi, (roi_center[0],centerline[1]), 4,(255, 255, 255),-1)
            cv2.circle(roi, (centerline[0],centerline[1]), 4, (0, 0, 255),-1)
            cv2.drawContours(roi, largest_contour, -1, (0, 255, 0), 10)  # You can change the color and thickness
             # Get the rotated rectangles for the largest contour
   
            horizontal_distance = centerline[0] - roi_center[0]
            pixel_in_range = lambda x: (np.all(x >= np.min(lower_blue)) and np.all(x <= np.max(upper_blue)))
            right_detector = pixel_in_range(right_hsv)
            left_detector = pixel_in_range(left_hsv)
            # Calculate the Euclidean distance
            #e_calculation = lambda x1, y1, x2, y2: math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2


            # Display the distance as text on the image
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(roi, f"Distance: {horizontal_distance:.2f}", (roi_center[0]+20,roi_center[1]), font, 1, (255, 255, 255), 2)
            #cv2.putText(roi, f"Direction: {direction}",(roi_center[0]+20,roi_center[1]-50), font, 1, (255, 255, 255), 2)
           
    if not(contours):
        horizontal_distance = -500
    return roi, horizontal_distance, (centerline,roi_center), right_detector,left_detector

# Initialize camera (or load an image)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    # Show the image with the blue tape centerline and the center of the image
    line_track_return = blue_line_tracking(frame)
    processed_frame = line_track_return[0]
    distance = line_track_return[1]
    cordinates = line_track_return[2]
    right = line_track_return[3]
    left =line_track_return[4]

    if processed_frame is not None and processed_frame.shape[0] > 0 and processed_frame.shape[1] > 0:
        cv2.imshow("Blue Tape Detection", processed_frame)
        if distance == -500:
            drive(0,0)
            print("Stopped")
        else:    
            error = distance

            if error is None:
                integral = 0
                    # Rest of your PID controller logic
            else:


                integral += error
                derivative = error - prev_error

                # Calculate the control output
                control_output = kp * error + ki * integral + kd * derivative

                # Map the control output to motor speeds
                right_speed = (50) - control_output
                left_speed = (50) + control_output

                # Ensure motor speeds are within a valid range (0-100)
                right_speed = max(0,min((50),right_speed))
                left_speed = max(0, min((50), left_speed))

                drive(right_speed, left_speed)

                prev_error = error
                print(right_speed,left_speed,control_output)
               


            
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()

