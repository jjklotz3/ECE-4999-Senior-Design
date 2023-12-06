from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

#THIS SCRIPT IS MEANT TO CONTROL THE SERVOS FOR SPRING LAUNCH TEST

# Use the pigpio pin factory for better servo control
pi_gpio_factory = PiGPIOFactory(host='localhost', port=8888)

# Specify the pin and other parameters
launch_servo = AngularServo(12, pin_factory=pi_gpio_factory, min_angle=0, max_angle=270, min_pulse_width=0.0005, max_pulse_width=0.0025)
lifting_arm_l = AngularServo(16, pin_factory=pi_gpio_factory, min_angle=0, max_angle=270, min_pulse_width=0.0005, max_pulse_width=0.0025)
lifting_arm_r = AngularServo(25, pin_factory=pi_gpio_factory, min_angle=0, max_angle=270, min_pulse_width=0.0005, max_pulse_width=0.0025)

angle1 = 0
angle2 = 248

while True:
    launch_servo.angle = 110
    lifting_arm_l.angle = 0
    lifting_arm_r.angle = 248
    sleep(1)
    while(angle1 != 132 and angle2 != 128):
        angle1 += 20
        angle2 -= 20
        lifting_arm_l.angle = angle1
        lifting_arm_r.angle = angle2
        sleep(0.4)
    sleep(4)
    launch_servo.angle = 200
    sleep(0.4)
    lifting_arm_l.angle = 0
    lifting_arm_r.angle = 248
    break
