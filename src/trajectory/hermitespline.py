import numpy as np
from utils.geometry import Vector, Pose, RobotState, boundRadians, PoseWithCurvature
from utils.physicalstates import ChassisState
from abc import ABC, abstractmethod


class HermiteSpline(ABC):
    def __init__(self, start: Pose, end: Pose):
        self.start = start
        self.end = end

    @abstractmethod
    def getPoint(self, t: float) -> Vector:
        """Interpolate a point along the spline."""
        raise NotImplementedError

    @abstractmethod
    def getD(self, t: float) -> Vector:
        """Interpolate the derivative along the spline."""
        raise NotImplementedError

    @abstractmethod
    def getDD(self, t: float) -> Vector:
        """Interpolate the 2nd derivative along the spline."""
        raise NotImplementedError

    @abstractmethod
    def getDDD(self, t: float) -> Vector:
        """Interpolate the 3rd derivative along the spline."""
        raise NotImplementedError

    def getCurvature(self, t: float) -> float:
        """Interpolate the curvature along the spline."""
        d = self.getD(t)
        dd = self.getDD(t)
        dx2dy2 = d.x ** 2 + d.y ** 2
        return (d.x * dd.y - d.y * dd.x) / (dx2dy2 ** 1.5)

    def getRadius(self, t: float) -> float:
        """Interpolate the radius along the spline."""
        c = self.getCurvature(t)
        if c == 0:
            return np.Inf
        else:
            return 1 / self.getCurvature(t)

    def getDCurvature(self, t: float) -> float:
        """Interpolate the derivative of the curvature along the spline."""
        d = self.getD(t)
        dd = self.getDD(t)
        ddd = self.getDDD(t)
        dx2dy2 = d.x ** 2 + d.y ** 2
        dc = (d.x * ddd.y - d.y * ddd.x) / (dx2dy2 ** 1.5) - 3 * (
            d.x * dd.y - d.y * dd.x
        ) * (d.x * dd.x + d.y * dd.y) / (dx2dy2 ** 2.5)
        return dc

    def getSumDCurvature2(self, dt: float = 0.01) -> float:
        sum_dc2 = 0
        for i in range(0, int(1 / dt) - 1):
            sum_dc2 += dt * (self.getDCurvature(i * dt) ** 2)
        return sum_dc2

    @staticmethod
    def getSumDCurvatures2(splines: np.array, dt: float = 0.01) -> float:
        sum_dc2 = 0
        for spline in splines:
            sum_dc2 += spline.getDCurvature(dt)
        return sum_dc2

    def getHeading(self, t: float) -> float:
        """Interpolate the angle of the tangent along the spline."""
        d = self.getD(t)
        return np.arctan2(d.y, d.x)

    def getPose(self, t: float) -> Pose:
        """Interpolate a pose along the spline."""
        point = self.getPoint(t)
        heading = self.getHeading(t)
        return Pose(point.x, point.y, heading)

    def getPoseWithCurvature(self, t: float) -> PoseWithCurvature:
        """Interpolate a pose with a curvature along the spline."""
        pose = self.getPose(t)
        curvature = self.getCurvature(t)
        dkds = self.getDCurvature(t) / self.getLinearVelocity(t)
        return PoseWithCurvature(pose.x, pose.y, pose.theta, curvature, dkds)

    def getLinearVelocity(self, t: float) -> float:
        """Interpolate the linear velocity of a particle traveling along the spline."""
        d = self.getD(t)
        return d.getMagnitude()

    def getAngularVelocity(self, t: float) -> float:
        """Interpolate the angular velocity of a particle traveling along the spline."""
        omega = self.getLinearVelocity(t) * self.getCurvature(t)
        return omega

    def getVelocities(self, t: float) -> ChassisState:
        """Interpolate the velocities of a particle traveling along the spline."""
        v = self.getLinearVelocity(t)
        omega = self.getAngularVelocity(t)
        return ChassisState(v, omega)

    def getLinearAcceleration(self, t: float) -> float:
        """Interpolate the linear acceleration of a particle traveling along the spline."""
        d = self.getD(t)
        dd = self.getDD(t)
        direction = np.sign(d.x * dd.x + d.y * dd.y)
        return direction * dd.getMagnitude()

    def getAngularAcceleration(self, t: float) -> float:
        """Interpolate the angular acceleration of a particle traveling along the spline."""
        alpha = self.getLinearAcceleration(t) * self.getCurvature(t)
        return alpha

    def getAcceleration(self, t: float) -> ChassisState:
        """Interpolate the acceleration of a particle traveling along the spline."""
        a = self.getLinearAcceleration(t)
        alpha = self.getAngularAcceleration(t)
        return ChassisState(a, alpha)

    def getArcLength(self, dt: float = 0.01) -> float:
        arc_length = 0
        for i in range(0, int(1 / dt)):
            p0 = self.getPoint(i * dt)
            p1 = self.getPoint((i + 1) * dt)
            arc_length += p0.getDistance(p1)
        return arc_length

    @abstractmethod
    def optimizeSpline(self):
        raise NotImplementedError

    @abstractmethod
    def computeCoefficients(self) -> None:
        raise NotImplementedError
