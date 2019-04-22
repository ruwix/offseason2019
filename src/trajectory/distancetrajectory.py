import numpy as np

from utils.geometry import PoseWithCurvature
from utils.mathextension import epsilonEquals


class DistanceTrajectory:
    def __init__(self, poses: np.array):
        self.poses = poses
        self.distances = np.empty(len(poses) - 1)
        self.distances[0] = 0
        for i in range(len(poses) - 1):
            p0 = poses[i]
            p1 = poses[i + 1]
            self.distances[i] = self.distances[i - 1] + p0.distance(p1)
        self.length = self.distances[-1]

    def getPoseWithCurvature(self, distance: float) -> PoseWithCurvature:
        if distance <= 0:
            return self.poses[0]
        elif distance >= self.length:
            return self.poses[-1]
        else:
            index = np.argmax(self.distances > distance)
            if epsilonEquals(self.distances[index], self.distances[index - 1]):
                return self.poses[index]
            else:
                return self.poses[index - 1].interpolate(
                    self.poses[index],
                    (distance - self.distances[index - 1])
                    / (self.distances[index] - self.distances[index - 1]),
                )
