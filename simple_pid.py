import math


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

        # Options / improvements
        self.integral_limit = integral_limit      # e.g., 100.0 (units of the integral)  # noqa: E501
        self.use_angle = use_angle                # True -> angle error (-pi..pi]        # noqa: E501
        self.d_alpha = float(d_alpha)             # 0..1, low-pass filter for D term     # noqa: E501
        self.derivative_on_measurement = derivative_on_measurement

        # Internal states for options
        self._previous_measured = None            # for D on measurement
        self._previous_derivative = 0.0           # for D filtering

    @staticmethod
    def _wrap_angle_rad(x):
        """Wrap angle to (-pi, pi]."""
        return math.atan2(math.sin(x), math.cos(x))

    @staticmethod
    def _clamp(x, lo, hi):
        """Clamp x to [lo, hi]."""
        return max(lo, min(hi, x))

    def reset(self, integral=0.0, previous_error=0.0):
        """Reset integral/derivative states."""
        self.integral = float(integral)
        self.previous_error = float(previous_error)
        self._previous_measured = None
        self._previous_derivative = 0.0

    def compute(self, setpoint, measured_value):
        """Compute PID output given setpoint and measured value."""
        if measured_value == 0:
            return 0
        # 1) Error (optionally wrapped for angular variables)
        if self.use_angle:
            error = self._wrap_angle_rad(setpoint - measured_value)
        else:
            error = setpoint - measured_value

        # 2) Integral with anti-windup (clamp)
        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = self._clamp(self.integral, -self.integral_limit, self.integral_limit)  # noqa: E501

        # 3) Derivative (backward difference) — optionally on measurement, then low-pass filtered  # noqa: E501
        if self.dt > 0:
            if self.derivative_on_measurement:
                if self._previous_measured is None:
                    raw_derivative = 0.0
                else:
                    dm = measured_value - self._previous_measured
                    if self.use_angle:
                        dm = self._wrap_angle_rad(dm)
                    raw_derivative = - dm / self.dt  # D on measurement => negative sign  # noqa: E501
                self._previous_measured = measured_value
            else:
                de = error - self.previous_error
                raw_derivative = de / self.dt
        else:
            raw_derivative = 0.0

        # Low-pass filter for D: d = (1 - a) * raw + a * prev
        derivative = (1.0 - self.d_alpha) * raw_derivative + self.d_alpha * self._previous_derivative  # noqa: E501
        self._previous_derivative = derivative

        # 4) PID output
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)  # noqa: E501

        # 5) Update state
        self.previous_error = error

        return output
