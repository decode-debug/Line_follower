#!/usr/bin/env python3
import math
import time
from time import sleep
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent
from ev3dev2.motor import MoveDifferential
from ev3dev2.wheel import EV3Tire


class PIDController:
    def __init__(self, kp, ki, kd, dt,
                 integral_limit=None,
                 use_angle=False,
                 d_alpha=0.0,
                 derivative_on_measurement=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.previous_error = 0.0

        self.integral_limit = integral_limit
        self.use_angle = use_angle
        self.d_alpha = float(d_alpha)
        self.derivative_on_measurement = derivative_on_measurement

        self._previous_measured = None
        self._previous_derivative = 0.0

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def reset(self, integral=0.0, previous_error=0.0):
        self.integral = float(integral)
        self.previous_error = float(previous_error)
        self._previous_measured = None
        self._previous_derivative = 0.0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value

        # Całka z anty-windup
        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = self._clamp(self.integral, -self.integral_limit, self.integral_limit)

        # Pochodna
        if self.dt > 0:
            if self.derivative_on_measurement:
                if self._previous_measured is None:
                    raw_derivative = 0.0
                else:
                    dm = measured_value - self._previous_measured
                    raw_derivative = - dm / self.dt
                self._previous_measured = measured_value
            else:
                de = error - self.previous_error
                raw_derivative = de / self.dt
        else:
            raw_derivative = 0.0

        derivative = (1.0 - self.d_alpha) * raw_derivative + self.d_alpha * self._previous_derivative
        self._previous_derivative = derivative

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output


# --- Inicjalizacja EV3 ---
left_color_sensor  = ColorSensor(INPUT_1)
right_color_sensor = ColorSensor(INPUT_2)
left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_B)
robot = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, 114)
robot.stop()

# --- Parametry prędkości ---
BASE_SPEED = 40           # % – prędkość bazowa
DEADBAND = 5              # % – małe błędy ignorowane
MAX_CORRECTION = 100       # % – maksymalna korekta z PID
SLEW_LIMIT = 200           # maks. zmiana prędkości w jednej iteracji

def clamp_speed(pct):
    return max(0, min(100, int(round(pct))))


def limit_delta(delta, limit):
    if delta > limit:
        return limit
    if delta < -limit:
        return -limit
    return delta

# --- Prędkości ciągłe ---
leftcirclespeed = BASE_SPEED
ricghtcirclespeed = BASE_SPEED

# PID controller
pid = PIDController(kp=0.80, ki=0.0, kd=0.015, dt=0.01,
                    integral_limit=10, use_angle=False, d_alpha=0.2)

#kalibracja (opcjonalnie)
left_color_sensor.calibrate_white()
right_color_sensor.calibrate_white()

# --- Główna pętla ---
try:
    while True:
        L = left_color_sensor.reflected_light_intensity + 4
        R = right_color_sensor.reflected_light_intensity
        error = R - L
        correction = pid.compute(0.0, error)

        if abs(correction) < DEADBAND:
            correction = 0.0
        correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

        delta_left  = limit_delta(correction,  SLEW_LIMIT)
        delta_right = limit_delta(-correction, SLEW_LIMIT)

        # Aktualizacja płynna
        leftcirclespeed  = clamp_speed(leftcirclespeed  + delta_left)
        ricghtcirclespeed = clamp_speed(ricghtcirclespeed + delta_right)

        leftcirclespeed = min(2*BASE_SPEED, leftcirclespeed)
        ricghtcirclespeed = min(2*BASE_SPEED, ricghtcirclespeed)

        # Sterowanie prędkościami
        robot.on_for_seconds(
            SpeedPercent(leftcirclespeed),
            SpeedPercent(ricghtcirclespeed),
            0.01, brake=False, block=False
        )

        sleep(0.01)
        print("Lewy czujnik: {}, Prawy czujnik: {}".format(L, R))



except KeyboardInterrupt:
    robot.off()