from abc import ABC, abstractmethod

from utils.geometry import PoseWithCurvature
from utils.mathextension import MinMax


class TimingConstraint(ABC):
    def getMaxVelocity(self, state: PoseWithCurvature) -> float:
        raise NotImplementedError

    def getMinMaxAcceleration(
        self, state: PoseWithCurvature, velocity: float
    ) -> MinMax:
        raise NotImplementedError
