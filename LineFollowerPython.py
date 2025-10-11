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
speed_on_turn = 200  # Speed while turning
speed = 200  # Speed while driving forward

# Define the turn degrees
turn_speed_multiplier = 1  # Turn speed multiplier

#Define the time to turn and drive
turn_time = 0.5  # Time to turn when a line is detected
drive_time = 0.1  # Time to drive forward when no line is detected

# Define min and max turn degrees
MIN_TURN = 2
MAX_TURN = 60
# Define the black and white values
blacks = [10, 30] # Space for black detection
whites = [80, 90] # Space for white detection

# Start the line following loop
try:
    while True:

        left_color_value = left_color_sensor.reflected_light_intensity
        right_color_value = right_color_sensor.reflected_light_intensity

        if any(abs(left_color_value - black) <= 10 for black in blacks):
            darkness = 100 - left_color_value
            turn_speed = max(MIN_TURN, min(MAX_TURN, turn_speed_multiplier * darkness))
            robot.turn_left(speed_on_turn, turn_speed,brake=False,block=False)

        if any(abs(right_color_value - black) <= 10 for black in blacks):
            darkness = 100 - right_color_value
            turn_speed = max(MIN_TURN, min(MAX_TURN, turn_speed_multiplier * darkness))
            robot.turn_right(speed_on_turn, turn_speed, brake=False,block=False)

        if (any(abs(right_color_value - white) <= 10 for white in whites)
            and any(abs(left_color_value - white) <= 10 for white in whites)):
            robot.on_for_seconds(speed, drive_time, brake=False, block=False)
        
except KeyboardInterrupt:
    print("Line following stopped by user.")
    robot.off()


