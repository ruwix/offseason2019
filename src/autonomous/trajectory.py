import numpy as np
from autonomous.quintichermitespline import QuinticHermiteSpline

from autonomous.cubichermitespline import CubicHermiteSpline

from pyfrc.sim import get_user_renderer

from utils.geometry import RobotState, boundRadians, Vector, Pose
from utils import units
from wpilib import RobotBase
from utils.splinegenerator import parameterizeSplines
from utils.distancetrajectory import DistanceTrajectory


class TrajectoryContraints:
    def __init__(self, max_velocity):
        self.max_velocity = max_velocity


class TrajectoryGenerator:
    def __init__(
        self, max_dx=0.05, max_dy=0.00635, max_dtheta=5 * units.degrees_per_radian
    ):
        self.max_dx = max_dx
        self.max_dy = max_dy
        self.max_dtheta = max_dtheta

    def generateTrajectory(
        self, spline_poses: np.array, constrains: TrajectoryContraints, reversed=False
    ):
        if reversed:
            for i in range(0, len(spline_poses)):
                poses[i].theta += np.pi
        splines = np.empty(len(spline_poses) - 1, dtype=QuinticHermiteSpline)
        for i in range(0, len(spline_poses) - 1):
            splines[i] = QuinticHermiteSpline(spline_poses[i], spline_poses[i + 1])
        return self.getDistanceTrajectory(splines)

    def getTrajectoryPosesFromSplines(self, splines):
        return parameterizeSplines(splines, 0.05, 0.00635, 5 * units.degrees_per_radian)

    def getDistanceTrajectory(self, splines):
        return DistanceTrajectory(self.getTrajectoryPosesFromSplines(splines))
    
    # def update(self, timestamp: float) -> None:
    #     self.timestamp = timestamp
    #     t = self.timestamp / (self.time / len(self.splines))
    #     self.index = int(t)
    #     self.t = t - self.index

    # def getState(self) -> RobotState:
    #     if not self.isFinished():
    #         t = int(self.timestamp / self.sample_size)
    #         pose = self.splines[self.index].getPose(self.t)
    #         velocity = self.splines[self.index].getVelocities(self.t)
    #         acceleration = self.splines[self.index].getAcceleration(self.t)
    #         velocity.linear /= self.time
    #         velocity.angular /= self.time
    #         acceleration.linear /= self.time
    #         acceleration.angular /= self.time
    #         if self.reversed:
    #             pose.theta += np.pi
    #             velocity.linear *= -1
    #             velocity.angular *= -1
    #         return RobotState(
    #             pose.x,
    #             pose.y,
    #             pose.theta,
    #             velocity.linear,
    #             velocity.angular,
    #             acceleration.linear,
    #             acceleration.angular,
    #         )
    #     else:
    #         return None

    # def build(self) -> None:
    #     for _t in range(0, int(1 / self.sample_size)):
    #         arc_length = self.arc_length * _t * self.sample_size
    #         i, t = self.getParameterAtDistance(arc_length)
    #         pose = self.splines[i].getPose(t)
    #         velocity = self.splines[i].getVelocities(t)
    #         acceleration = self.splines[i].getAcceleration(t)
    #         velocity.linear /= self.time
    #         velocity.angular /= self.time
    #         acceleration.linear /= self.time
    #         acceleration.angular /= self.time
    #         if self.reversed:
    #             pose.theta += np.pi
    #             velocity.linear *= -1
    #             velocity.angular *= -1
    #         state = RobotState(
    #             pose.x,
    #             pose.y,
    #             pose.theta,
    #             velocity.linear,
    #             velocity.angular,
    #             acceleration.linear,
    #             acceleration.angular,
    #         )
    #         self.states = np.append(self.states, state)

    # def isFinished(self) -> float:
    #     return self.timestamp >= self.time

    # def writeCSV(self, filename: str) -> None:
    #     pass
    # with open(filename, mode="w") as output:
    #     writer_ = writer(output, delimiter=",", quotechar='"')
    #     writer_.writerow(["x", "y", "heading", "v", "omega", "a", "alpha"])
    #     data = np.empty((0, 7))
    #     for state in self.states:
    #         array = np.array(
    #             [
    #                 state.x,
    #                 state.y,
    #                 state.heading,
    #                 state.v,
    #                 state.omega,
    #                 state.a,
    #                 state.alpha,
    #             ]
    #         ).reshape((1, 7))
    #         array = np.round(array, 2)
    #         data = np.append(data, array, axis=0)
    #     writer_.writerows(data)
    #     output.close()

    # def drawSimulation(self) -> None:
    #     if RobotBase.isSimulation() and get_user_renderer() != None:
    #         points = np.empty((0, 2))
    #         for state in self.states:
    #             point = np.array([state.x, state.y + 4.1148]).reshape((1, 2))
    #             points = np.append(points, point, axis=0)
    #         get_user_renderer().draw_line(
    #             points,
    #             scale=(units.feet_per_meter, units.feet_per_meter),
    #             color="#FFFFFF",
    #             width=2,
    #         )

