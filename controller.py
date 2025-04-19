# controller.py

import numpy as np

class PIDController:
    def __init__(self, Kp=150, Ki=10, Kd=50):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_error = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral_error = 0.0
        self.prev_error = 0.0

    def compute(self, current_heading, desired_heading, dt):
        error = (desired_heading - current_heading + np.pi) % (2 * np.pi) - np.pi
        self.integral_error += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative