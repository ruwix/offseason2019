from trajectory.constraints.timingconstraint import TimingConstraint
from utils.mathextension import MinMax
import numpy as np
from utils.geometry import PoseWithCurvature


class CentripetalAccelerationConstraint(TimingConstraint):
    def __init__(self, max_centripetal_acceleration: float):
        self.max_centripetal_acceleration = max_centripetal_acceleration

    def getMaxVelocity(self, state: PoseWithCurvature) -> float:
        if state.curvature == 0:
            return np.inf
        else:
            return np.sqrt(abs(self.max_centripetal_acceleration / state.curvature))

    def getMinMaxAcceleration(
        self, state: PoseWithCurvature, velocity: float
    ) -> MinMax:
        return MinMax()
