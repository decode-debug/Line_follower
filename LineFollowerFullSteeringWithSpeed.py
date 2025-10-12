#!/usr/bin/env python3
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B
from ev3dev2.motor import MediumMotor, OUTPUT_C
from ev3dev2.motor import ServoMotor, MotorSet
from ev3dev2.motor import SpeedPercent
from simple_pid import PIDController
# Initialize the color sensor
left_color_sensor = ColorSensor(INPUT_1) # Connect to port 1 (1st port from the left)
right_color_sensor = ColorSensor(INPUT_2) # Connect to port 2 (2st port from the left)

# Initialize the motors
left_motor = LargeMotor(OUTPUT_A)  # Connect to port A (left motor)
right_motor = LargeMotor(OUTPUT_B) # Connect to port B (right motor)
steermotor = MediumMotor(OUTPUT_C) # Connect to port C (steering motor)
group = MotorSet((OUTPUT_A, OUTPUT_B))
robotsteering = ServoMotor(OUTPUT_C)

# Wheel and robot parameters
wheel_diameter = 56  # in mm
acceleraion_value = 100  # value to determine how fast should the robot accelerate
# Steering parameters
BASE = 40              # Base speed
KP   = 1.2             # Amplification of the proportional term
KI   = 0.5             # Amplification of the integral term
KD   = 0.3             # Amplification of the derivative term
SpeedCLAMP = 100       # speed clamp
ArcClamp = 90          # Arc clamp
DT = 0.02              # sampling time (50Hz)
robotsteering.reset()  # Reset the steering motor to 0 position

# TODO: fix speed changing while steering
def set_speed_percent(group, turn_degrees):
    """Set speed for a MotorSet with clamping."""
    initial_rotational_speed = left_motor.speed
    PIDController(kp=KP, 
                  ki=KI, 
                  kd=KD, 
                  dt=DT,
                  integral_limit=SpeedCLAMP)
    if turn_degrees == 0: # going straight
        speed = PIDController.compute(initial_rotational_speed, acceleraion_value)  # accelerate fast
    speed = PIDController.compute(initial_rotational_speed, -abs(turn_degrees))
    clamped_speed = max(-SpeedCLAMP, min(SpeedCLAMP, speed))
    group.on(SpeedPercent(clamped_speed))

def turn_degrees(steermotor, LightDifference):
    """
    Turn the steering motor to a specific angle in degrees.
    :param steermotor: ServoMotor instance
    :param degrees: Angle in degrees (-90..90)
    """
    pid = PIDController(kp=KP, ki=KI, kd=KD, dt=DT, integral_limit=ArcClamp, use_angle=True)

    d = pid.compute(0, LightDifference)  # error = 0 - LightDifference

    steermotor.on_to_position(SpeedPercent(20), d, brake=False, block=False)
    return d


# Start the line following loop
try:
    while True:
        Leftlight = left_color_sensor.reflected_light_intensity
        RightLight = right_color_sensor.reflected_light_intensity

        LightDifference = Leftlight - RightLight

        if abs(LightDifference) < 5:
            # To do: make it steer with derivative and integral component
            d = robotsteering.turn_degrees(0)  # No steering
            set_speed_percent(group, d)  # Move forward at speed given by steering

        else:
            d = robotsteering.turn_degrees(LightDifference)  # Turn left or right based on light difference
            set_speed_percent(group, d)  # Move forward at speed given by steering

except KeyboardInterrupt:
    print("Line following stopped by user.")


group.stop() # Ensure the robot is stopped at the start
steermotor.stop() # Ensure the steering motor is stopped at the start