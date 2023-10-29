
import cv2 
import numpy as np 
from matplotlib import pyplot as plt 

def calculate_centerline(contours):
    moments = cv2.moments(contours)
    if moments["m00"] != 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        return cx,cy
    return None

# reading image 
# Initialize camera (or load an image)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    image_height, image_width, _ = frame.shape

    # Define the height of the ROI as a percentage of the image height
    roi_height_percentage = 55  # Adjust as needed

    # Calculate the ROI coordinates
    roi_top_left = (0, image_height - (image_height * roi_height_percentage // 100))
    roi_bottom_right = (image_width, image_height)

    # Create the ROI by slicing the image
    roi = frame[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]
    roi_height, roi_width = roi.shape[:2]
    roi_center = (roi_width // 2, roi_height // 2)  # Fix the order of width and height
  
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Create a mask to isolate the blue color
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
   
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centerline = None  # Initialize centerline as None
    direction = ""
    distance = None
        
    if contours:  # Check if contours list is not empty
            
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the centerline of the largest contour
        centerline = calculate_centerline(largest_contour)
        print(largest_contour)
    
    cv2.imshow('file',frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
"""
# converting image into grayscale image 
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
  
# setting threshold of gray image 
_, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY) 
  
# using a findContours() function 
contours, _ = cv2.findContours( 
    threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
  
i = 0
  
# list for storing names of shapes 
for contour in contours: 
  
    # here we are ignoring first counter because  
    # findcontour function detects whole image as shape 
    if i == 0: 
        i = 1
        continue
  
    # cv2.approxPloyDP() function to approximate the shape 
    approx = cv2.approxPolyDP( 
        contour, 0.01 * cv2.arcLength(contour, True), True) 
      
    # using drawContours() function 
    cv2.drawContours(img, [contour], 0, (0, 0, 255), 5) 
  
    # finding center point of shape 
    M = cv2.moments(contour) 
    if M['m00'] != 0.0: 
        x = int(M['m10']/M['m00']) 
        y = int(M['m01']/M['m00']) 
  
    # putting shape name at center of each shape 
    if len(approx) == 3: 
        cv2.putText(img, 'Triangle', (x, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
  
    elif len(approx) == 4: 
        cv2.putText(img, 'Quadrilateral', (x, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
  
    elif len(approx) == 5: 
        cv2.putText(img, 'Pentagon', (x, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
  
    elif len(approx) == 6: 
        cv2.putText(img, 'Hexagon', (x, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
  
    else: 
        cv2.putText(img, 'circle', (x, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
  
# displaying the image after drawing contours 
cv2.imshow('shapes', img) 
  
cv2.waitKey(0) 
cv2.destroyAllWindows() 
"""