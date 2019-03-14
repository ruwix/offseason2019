import numpy as np
from csv import reader, writer
from auto.hermitespline import HermiteSpline
from pyfrc.sim import get_user_renderer

from utils.geometry import RobotState, boundHalfRadians


class Trajectory:
    def __init__(self, poses: np.array, time: float, sample_size: float = 0.02):
        self.path = HermiteSpline(poses)
        self.poses = np.empty((0, 3))
        self.velocities = np.empty((0, 2))
        self.sample_size = sample_size
        self.time = time
        self.timestamp = 0

    @staticmethod
    def loadPath(file: str) -> np.array:
        with open(file, "r") as path:
            _reader = reader(path)
            headings = next(_reader)
            assert headings == ["x", "y", "heading"]
            ret = np.array(list(_reader)).astype(float)
            path.close()
            return ret

    def getAverageVelocity(self) -> float:
        return self.path.getArcLength() / self.time

    def update(self, t: float) -> None:
        self.timestamp = t

    def getState(self) -> RobotState:
        if not self.isFinished():
            pose = self.path.getPose(self.timestamp / (self.time / self.path.length))
            twist = self.path.getTwist(self.timestamp / (self.time / self.path.length))
            twist.omega = boundHalfRadians(twist.omega)
            twist /= self.time
            return RobotState(pose.x, pose.y, pose.theta, twist.v, twist.omega)
        else:
            return None

    def build(self) -> None:
        for i in range(0, int(self.path.length / self.sample_size)):
            pose = self.path.getPose(i * self.sample_size)
            twist = self.path.getTwist(i * self.sample_size) / self.time
            _pose = np.array([pose.x, pose.y, pose.theta]).reshape((1, 3))
            _twist = np.array([twist.v, twist.omega]).reshape((1, 2))
            self.poses = np.append(self.poses, _pose, axis=0)
            self.velocities = np.append(self.velocities, _twist, axis=0)

    def isFinished(self) -> float:
        return self.timestamp >= self.time

    def writeCSV(self, filename: str) -> None:
        with open(filename, mode="w") as output:
            writer_ = writer(output, delimiter=",", quotechar='"')
            writer_.writerow(["x", "y", "heading", "v", "omega"])
            data = np.concatenate((self.poses, self.velocities), axis=1)
            writer_.writerows(data)
            output.close()

    def drawSimulation(self) -> None:
        get_user_renderer().draw_line(self.poses[:, 0:2], scale=(1 / 12, 1 / 12))
