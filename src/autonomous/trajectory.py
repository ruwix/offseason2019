import numpy as np
from csv import reader, writer
from autonomous.quintichermitespline import QuinticHermiteSpline
from autonomous.cubichermitespline import CubicHermiteSpline

from pyfrc.sim import get_user_renderer

from utils.geometry import RobotState, boundHalfRadians
from utils import units


class Trajectory:
    def __init__(
        self, poses: np.array, time: float, reversed=False, sample_size: float = 0.02
    ):
        if reversed:
            for i in range(0, len(poses)):
                poses[i].theta += np.pi
        # self.path = CubicHermiteSpline(poses)
        self.path = QuinticHermiteSpline(poses)
        QuinticHermiteSpline.optimizeSpline(self.path)
        self.states = np.empty(0)
        self.time = time
        self.reversed = reversed
        self.sample_size = sample_size
        self.timestamp = 0

    def getAverageVelocity(self) -> float:
        return self.path.getArcLength() / self.time

    def update(self, t: float) -> None:
        self.timestamp = t
        self.t = self.timestamp / (self.time / self.path.length)

    def getState(self) -> RobotState:
        if not self.isFinished():
            pose = self.path.getPose(self.t)
            twist = self.path.getTwist(self.t)
            twist /= self.time
            if self.reversed:
                pose.theta += np.pi
                twist *= -1
            return RobotState(pose.x, pose.y, pose.theta, twist.x, twist.omega)
        else:
            return None

    def build(self) -> None:
        for i in range(0, int(self.path.length / self.sample_size)):
            pose = self.path.getPose(i * self.sample_size)
            twist = self.path.getTwist(i * self.sample_size)
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
