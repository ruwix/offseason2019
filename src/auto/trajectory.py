import numpy as np
from csv import reader
from auto.hermitespline import HermiteSpline


def loadPath(file):
    with open(file, "r") as path:
        _reader = reader(path)
        headings = next(_reader)
        assert headings == ["x", "y", "heading"]
        ret = np.array(list(_reader)).astype(float)
        path.close()
        return ret


class Trajectory:
    def __init__(self, poses, dt):
        self.path = HermiteSpline(poses)
        self.poses = np.empty((0, 3))
        self.velocities = np.empty((0, 3))
        self.dt = dt

    def build(self):
        np.set_printoptions(suppress=True)
        for i in range(0, self.path.length * 100):
            pose = self.path.interpolatePose(i / 100).reshape((1, 3))
            self.poses = np.append(self.poses, pose, axis=0)
        for i in range(0, len(self.poses) - 1):
            velocity = ((self.poses[i + 1] - self.poses[i]) / self.dt).reshape((1, 3))
            self.velocities = np.append(self.velocities, velocity, axis=0)
