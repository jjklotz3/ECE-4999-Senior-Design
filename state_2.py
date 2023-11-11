import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
import time
import os 
from state_3 import blue_line_tracking, calculate_angle, calculate_centerline
import smbus					#import SMBus module of I2C
from time import sleep          #import

calibration_angle_x = 90.0
calibration_angle_y = 90.0
calibration_angle_z = 90.0

current_angle_x = calibration_angle_x
current_angle_y = calibration_angle_y
current_angle_z = calibration_angle_z

time_step = 1.0


#Set the GPIO mode to BCM
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

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

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


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

def dist(a,b):
     return math.sqrt((a*a)+(b*b))

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")

while True:
	
    #Read Accelerometer raw value
    #acc_x = read_raw_data(ACCEL_XOUT_H)
    #acc_y = read_raw_data(ACCEL_YOUT_H)
    #acc_z = read_raw_data(ACCEL_ZOUT_H)

    #acc_x = acc_x/16384.0
    #acc_y = acc_y/16384.0
    #acc_z = acc_z/16384.0
        
    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    gyro_x = gyro_x/131.0
    gyro_y = gyro_y/131.0
    gyro_z = gyro_z/131.0

        
    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    angle_x = math.degrees(math.atan2(gyro_x,dist(gyro_y,gyro_z)))
    angle_y = math.degrees(math.atan2(gyro_y,dist(gyro_x,gyro_z)))
    angle_z = math.degrees(math.atan2(gyro_z,dist(gyro_y,gyro_x)))
            
    #Gx = gyro_x/131.0
    #Gy = gyro_y/131.0
    #Gz = gyro_z/131.0
    
    """
    angle_x -= calibration_angle_x
    angle_y -= calibration_angle_y
    angle_z -= calibration_angle_z


    if angle_x < 0:
        angle_x += 360
    elif angle_x >= 360:
        angle_x -= 360

    if angle_y < 0:
        angle_y += 360
    elif angle_y >= 360:
        angle_y -= 360

    if angle_z < 0:
        angle_z += 360
    elif angle_z >= 360:
        angle_z -= 360


    current_angle_x = angle_x
    current_angle_y = angle_y
    current_angle_z = angle_z

    """      
    print ("Angle X: {:.2f}".format(current_angle_x),"Angle Y: {:.2f}".format(current_angle_y),"Angle Z: {:.2f}".format(current_angle_z))
    sleep(time_step)

