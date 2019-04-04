import numpy as np
from utils.geometry import RobotState, boundRadians
from utils.physicalstates import ChassisState


class Ramsete:
    def __init__(self, kbeta: float, kzeta: float):
        self.kbeta = kbeta  # 0 < kbeta; larger kbeta makes convergence more aggresive
        self.kzeta = kzeta  # 0 < kzeta < 1; larger kbeta provides more damping
        self.error = RobotState()

    def update(self, state: RobotState, state_d: RobotState) -> ChassisState:
        # Compute errors
        self.error = state_d - state
        self.error.heading = boundRadians(self.error.heading)
        # Compute trig functions
        cos_theta = np.cos(state.heading)
        sin_theta = np.sin(state.heading)
        # Compute gains
        k1 = 2 * self.kzeta * np.sqrt(state_d.omega ** 2 + self.kbeta * state_d.v ** 2)
        k2 = self.kbeta
        k3 = k1
        # Find linear velocity
        v = state_d.v * np.cos(self.error.heading) + k1 * (
            self.error.x * cos_theta + self.error.y * sin_theta
        )
        # Find angular velocity
        omega = (
            state_d.omega
            + k2
            * state_d.v
            * np.sinc(self.error.heading)
            * (self.error.y * cos_theta - self.error.x * sin_theta)
            + k3 * self.error.heading
        )
        return ChassisState(v, omega)

    def getError(self):
        return self.error
