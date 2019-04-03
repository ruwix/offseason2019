import numpy as np
from csv import reader, writer
from autonomous.quintichermitespline import QuinticHermiteSpline

from autonomous.cubichermitespline import CubicHermiteSpline

from pyfrc.sim import get_user_renderer

from utils.geometry import RobotState, boundRadians
from utils import units


class Trajectory:
    def __init__(
        self, poses: np.array, time: float, reversed=False, sample_size: float = 0.02
    ):
        if reversed:
            for i in range(0, len(poses)):
                poses[i].theta += np.pi
        self.splines = np.empty(len(poses) - 1, dtype=QuinticHermiteSpline)
        for i in range(0, len(poses) - 1):
            self.splines[i] = QuinticHermiteSpline(poses[i], poses[i + 1])
        self.states = np.empty(0)
        self.time = time
        self.reversed = reversed
        self.sample_size = sample_size
        self.timestamp = 0

    def update(self, timestamp: float) -> None:
        self.timestamp = timestamp
        t = self.timestamp / (self.time / len(self.splines))
        self.index = int(t)
        self.t = t - self.index

    def getState(self) -> RobotState:
        if not self.isFinished():
            pose = self.splines[self.index].getPose(self.t)
            twist = self.splines[self.index].getTwist(self.t)
            twist /= self.time
            if self.reversed:
                pose.theta += np.pi
                twist *= -1
            return RobotState(pose.x, pose.y, pose.theta, twist.x, twist.omega)
        else:
            return None

    def build(self) -> None:
        for spline in self.splines:
            for t in range(0, int(1 / self.sample_size)):
                pose = spline.getPose(t * self.sample_size)
                twist = spline.getTwist(t * self.sample_size)
                twist /= self.time
                if self.reversed:
                    pose.theta += np.pi
                    twist *= -1
                state = RobotState(pose.x, pose.y, pose.theta, twist.x, twist.omega)
                self.states = np.append(self.states, state)

    def isFinished(self) -> float:
        return self.timestamp >= self.time

    def writeCSV(self, filename: str) -> None:
        with open(filename, mode="w") as output:
            writer_ = writer(output, delimiter=",", quotechar='"')
            writer_.writerow(["x", "y", "heading", "v", "omega"])
            data = np.empty((0, 5))
            for state in self.states:
                array = np.array(
                    [state.x, state.y, state.heading, state.v, state.omega]
                ).reshape((1, 5))
                array = np.round(array, 2)
                data = np.append(data, array, axis=0)
            writer_.writerows(data)
            output.close()

    def drawSimulation(self) -> None:
        points = np.empty((0, 2))
        for state in self.states:
            point = np.array([state.x, state.y + 4.1148]).reshape((1, 2))
            points = np.append(points, point, axis=0)
        get_user_renderer().draw_line(
            points,
            scale=(units.feet_per_meter, units.feet_per_meter),
            color="#FFFFFF",
            width=2,
        )
