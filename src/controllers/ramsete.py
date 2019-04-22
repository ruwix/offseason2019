import numpy as np

from trajectory.timedtrajectory import TimedState
from utils.geometry import Pose, boundRadians
from utils.physicalstates import ChassisState


class Ramsete:
    def __init__(self, kbeta: float, kzeta: float):
        self.kbeta = kbeta  # 0 < kbeta; larger kbeta makes convergence more aggresive
        self.kzeta = kzeta  # 0 < kzeta < 1; larger kbeta provides more damping
        self.error = Pose()

    def update(self, state: Pose, state_d: TimedState) -> ChassisState:
        # Compute errors
        self.error = state_d.state.pose.inFrameOfReferenceOf(state)
        self.error.theta = boundRadians(self.error.theta)
        # Compute desired velocities
        v_d = state_d.velocity
        omega_d = state_d.state.curvature * v_d
        # Compute gains
        k1 = 2 * self.kzeta * np.sqrt(omega_d ** 2 + self.kbeta * v_d ** 2)
        k2 = self.kbeta
        k3 = k1
        # Find linear velocity
        v = v_d * np.cos(self.error.theta) + k1 * self.error.x
        # Find angular velocity
        omega = (
            omega_d
            + k2 * v_d * np.sinc(self.error.theta) * self.error.y
            + k3 * self.error.theta
        )
        return ChassisState(v, omega)
