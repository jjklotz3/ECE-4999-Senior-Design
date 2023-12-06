from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import keyboard

#THE PURPOSE OF TIS SCRIPT IS TO PROPERLY CONTOL SPRING RELEASE SERVO WHEN GETTING THE SPRING INTO POSITION BEFORE START

# Use the pigpio pin factory for better servo control
pi_gpio_factory = PiGPIOFactory(host='localhost', port=8888)

# Specify the pin and other parameters
launch_servo = AngularServo(12, pin_factory=pi_gpio_factory, min_angle=0, max_angle=270, min_pulse_width=0.0005, max_pulse_width=0.0025)

while True:
    launch_servo.angle = 160
    user_input = input("Press 'o' to open, 'c' to close, and 'e' to exit:\n")
    print(f'Input: {user_input}')

    if user_input == 'c':
        launch_servo.angle = 110
        sleep(2)
        break
    elif user_input == 'e':
        break
    else:
        print("Invalid Input")
