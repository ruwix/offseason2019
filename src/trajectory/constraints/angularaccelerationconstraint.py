from trajectory.constraints.timingconstraint import TimingConstraint
from utils.mathextension import MinMax
import numpy as np
from utils.geometry import PoseWithCurvature


class AngularAccelerationConstraint(TimingConstraint):
    def __init__(self, max_angular_acceleration: float):
        self.max_angular_acceleration = max_angular_acceleration

    def getMaxVelocity(self, state: PoseWithCurvature) -> float:
        # We don't want v^2 * dk/ds alone to go over the max angular acceleration.
        # v^2 * dk/ds = maxAngularAcceleration when linear acceleration = 0.
        # v = sqrt(maxAngularAcceleration / dk/ds)
        if state.dkds == 0:
            return np.inf
        else:
            return np.sqrt(self.max_angular_acceleration / abs(state.dkds))

    def getMinMaxAcceleration(
        self, state: PoseWithCurvature, velocity: float
    ) -> MinMax:
        # We want to limit the acceleration such that we never go above the specified angular acceleration.
        #
        # Angular acceleration = dw/dt     WHERE   w = omega = angular velocity
        # w = v * k                        WHERE   v = linear velocity, k = curvature
        #
        # dw/dt = d/dt (v * k)
        #
        # By chain rule,
        # dw/dt = dv/dt * k + v * dk/dt   [1]
        #
        # We don't have dk/dt, but we do have dk/ds and ds/dt
        # dk/ds * ds/dt = dk/dt     [2]
        #
        # Substituting [2] in [1], we get
        # dw/dt = acceleration * curvature + velocity * velocity * d_curvature
        # WHERE acceleration = dv/dt, velocity = ds/dt, d_curvature = dk/dt and curvature = k
        #
        # We now want to find the linear acceleration such that the angular acceleration (dw/dt) never goes above
        # the specified amount.
        #
        # acceleration * curvature = dw/dt - (velocity * velocity * d_curvature)
        # acceleration = (dw/dt - (velocity * velocity * d_curvature)) / curvature
        if state.curvature == 0:
            return MinMax()
        else:
            max_abs_acceleration = np.abs(
                (self.max_angular_acceleration - (velocity ** 2 * state.dkds))
                / state.curvature
            )
            return MinMax(-max_abs_acceleration, max_abs_acceleration)
