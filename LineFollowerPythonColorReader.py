import time

start_time = time.time()   # record when program starts

#!/usr/bin/env python3
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B
from ev3dev2.motor import MoveDifferential


# Initialize the color sensor
left_color_sensor = ColorSensor(INPUT_1) # Connect to port 1 (1st port from the left)
right_color_sensor = ColorSensor(INPUT_2) # Connect to port 2 (2st port from the left)

# Initialize the motors
left_motor = LargeMotor(OUTPUT_A)  # Connect to port A (left motor)
right_motor = LargeMotor(OUTPUT_B) # Connect to port B (right motor)
robot = MoveDifferential(OUTPUT_A, OUTPUT_B, 56, 114) # 56mm wheel diameter, 114mm axle track
robot.stop() # Ensure the robot is stopped at the start

# Define the speed of the robot
turn_speed = 200  # Adjust this value based on your preference
speed = 200  # Adjust this value based on your preference

# Define the turn degrees
turn_degrees = 30  # Degrees to turn when a line is detected

#Define the time to turn and drive
turn_time = 0.5  # Time to turn when a line is detected
drive_time = 0.1  # Time to drive forward when no line is detected

# Define the black and white values
blacks = [ 10, 30 ] # Space for black detection
whites = [ 80, 90 ] # Space for white detection

# Set the color sensor to measure reflected light intensity
left_color_sensor.mode = 'COL-REFLECT'
# Calibrate the sensors (optional)
# left_color_sensor.calibrate_white()
# right_color_sensor.calibrate_white()
# left_color_sensor.calibrate_black()
# right_color_sensor.calibrate_black()

# Start the line following loop
try:
    while True:
        if any(abs(left_color_sensor.reflected_light_intensity - black) <= 10 for black in blacks):
            robot.turn_left(turn_speed, turn_degrees,brake=False,block=False)

        if any(abs(right_color_sensor.reflected_light_intensity - black) <= 10 for black in blacks):
            robot.turn_right(turn_speed, turn_degrees, brake=False,block=False)

        if (any(abs(right_color_sensor.reflected_light_intensity - white) <= 10 for white in whites)
            and any(abs(left_color_sensor.reflected_light_intensity - white) <= 10 for white in whites)):
            robot.on_for_seconds(speed, drive_time, brake=False, block=False)
        
        if time.time() - start_time % 500:
            brightness = left_color_sensor.reflected_light_intensity
            print("Reflected light intensity:", brightness)
except KeyboardInterrupt:
    print("Line following stopped by user.")
    robot.off()


