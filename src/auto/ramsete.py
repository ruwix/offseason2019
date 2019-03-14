import numpy as np
from utils.geometry import RobotState, Twist, boundHalfRadians


class Ramsete:
    def __init__(self, kbeta: float, kzeta: float):
        self.kbeta = kbeta  # 0 < kbeta; larger kbeta makes convergence more aggresive
        self.kzeta = kzeta  # 0 < kzeta < 1; larger kbeta provides more damping

    def update(self, state: RobotState, state_d: RobotState) -> Twist:
        # Compute errors
        error = state_d - state
        error.heading = boundHalfRadians(error.heading)
        # Compute trig functions
        cos_theta = np.cos(state.heading)
        sin_theta = np.sin(state.heading)
        # Compute gains
        k1 = 2 * self.kzeta * np.sqrt(state_d.omega ** 2 + self.kbeta * state_d.v ** 2)
        k2 = self.kbeta
        k3 = k1
        # Find linear velocity
        v = state_d.v * np.cos(error.heading) + k1 * (
            error.x * cos_theta + error.y * sin_theta
        )
        # Find angular velocity
        omega = (
            state_d.omega
            + k2
            * state_d.v
            * np.sinc(error.heading)
            * (error.y * cos_theta - error.x * sin_theta)
            + k3 * error.heading
        )
        return Twist(v, omega)

