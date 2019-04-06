from utils.epsilon import epsilonEquals
import numpy as np


class DistanceTrajectory:
    def __init__(self, poses):
        self.poses = poses
        self.distances = np.empty(len(poses) - 1)
        self.distances[0] = 0
        for i in range(len(poses) - 1):
            p0 = poses[i]
            p1 = poses[i + 1]
            self.distances[i] = self.distances[i - 1] + p0.point.getDistance(p1.point)
        self.length = self.distances[-1]

    def getPose(self, distance):
        if distance <= 0:
            return self.poses[0]
        elif distance >= self.distances[-1]:
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
