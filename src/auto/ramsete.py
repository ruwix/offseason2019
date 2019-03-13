import numpy as np
from utils.geometry import RobotState, Twist


class Ramsete:
    def __init__(self, kbeta: float, kzeta: float):
        self.kbeta = kbeta  # 0 < kbeta; larger kbeta makes convergence more aggresive
        self.kzeta = kzeta  # 0 < kzeta < 1; larger kbeta provides more damping

    def update(self, state: RobotState, state_d: RobotState) -> Twist:
        theta = np.deg2rad(state.heading)
        theta_d = np.deg2rad(state_d.heading)
        omega_d = np.deg2rad(state_d.omega)
        # Compute errors
        error = state_d - state
        error_theta = np.deg2rad(error.heading)
        # Compute trig functions
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        # Compute gains
        k1 = 2 * self.kzeta * np.sqrt(omega_d ** 2 + self.kbeta * state_d.v ** 2)
        k2 = self.kbeta
        k3 = k1
        # Find linear velocity
        v = state_d.v * np.cos(error_theta) + k1 * (
            error.x * cos_theta + error.y * sin_theta
        )
        # Find angular velocity
        omega = (
            omega_d
            + k2
            * state_d.v
            * np.sinc(error_theta)
            * (error.y * cos_theta - error.x * sin_theta)
            + k3 * error_theta
        )
        return Twist(v, np.rad2deg(omega))

