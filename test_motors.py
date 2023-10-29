import cv2
import numpy as np
import math
from gpiozero import Robot 
import RPi.GPIO as GPIO
from time import sleep

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
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif left_speed < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
    if right_speed > 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    elif right_speed < 0:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)

    pwm_a.ChangeDutyCycle(abs(right_speed))
    pwm_b.ChangeDutyCycle(abs(left_speed))

while(True):
                    drive(-1,50)
                    sleep(2)
                    drive(0,0)
                    sleep(0.5)
                    drive(50,-50)
                    sleep(2)
                    break
