import numpy as np
from utils.geometry import Vector, Pose, Twist, RobotState
from abc import ABC, abstractmethod


class HermiteSpline:
    def __init__(self, poses: np.array):
        self.poses = poses
        self.length = len(self.poses) - 1

    @abstractmethod
    def getPoint(self, t: float) -> Vector:
        """Interpolate a point along the spline where 0 <= t <= self.length."""
        return Vector(0, 0)

    @abstractmethod
    def getD(self, t: float) -> Vector:
        return Vector(0, 0)
        """Interpolate the derivative along the spline where 0 <= t <= self.length."""

    @abstractmethod
    def getDD(self, t: float) -> Vector:
        return Vector(0, 0)
        """Interpolate the 2nd derivative along the spline where 0 <= t <= self.length."""

    @abstractmethod
    def getDDD(self, t: float) -> Vector:
        """Interpolate the 3rd derivative along the spline where 0 <= t <= self.length."""
        return Vector(0, 0)

    @abstractmethod
    def getCurvature(self, t: float) -> float:
        """Interpolate the curvature along the spline where 0 <= t <= self.length."""
        return Vector(0, 0)

    def getRadius(self, t: float) -> float:
        """Interpolate the curvature along the spline where 0 <= t <= self.length."""
        c = self.getCurvature(t)
        if c == 0:
            return float("inf")
        else:
            return 1 / self.getCurvature(t)

    def getDCurvature(self, t: float) -> float:
        """Interpolate the derivative of the curvature along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
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
        for i in range(0, self.length * int(1 / dt) - 1):
            sum_dc2 += dt * (self.getDCurvature(i * dt) ** 2)
        return sum_dc2

    def getHeading(self, t: float) -> float:
        """Interpolate the angle of the tangent along the spline where 0 <= t <= self.length."""
        d = self.getD(t)
        return np.arctan2(d.y, d.x)

    def getPose(self, t: float) -> Pose:
        """Interpolate the pose on the spline where 0 <= t <= self.length."""
        point = self.getPoint(t)
        heading = self.getHeading(t)
        return Pose(point.x, point.y, heading)

    def getLinearVelocity(self, t: float) -> float:
        """Interpolate the linear velocity of a particle traveling along the spline where 0 <= t <= self.length."""
        d = self.getD(t)
        return d.getMagnitude()

    def getAngularVelocity(self, t: float) -> float:
        """Interpolate the angular velocity of a particle traveling along the spline where 0 <= t <= self.length."""
        omega = self.getLinearVelocity(t) * self.getCurvature(t)
        return omega

    def getTwist(self, t: float) -> Twist:
        """Interpolate the twist of a particle traveling along the spline at timestamp t where 0 <= t <= self.length."""
        x = self.getLinearVelocity(t)
        omega = self.getAngularVelocity(t)
        return Twist(x, 0, omega)

    def getArcLength(self, dt: float = 0.01) -> float:
        arc_length = 0
        for i in range(0, self.length * int(1 / dt) - 1):
            p0 = self.getPoint(i * dt)
            p1 = self.getPoint((i + 1) * dt)
            arc_length += p0.getDistance(p1)
        return arc_length

    @abstractmethod
    def optimizeSpline(self):
        pass

    @abstractmethod
    def computeCoefficients(self) -> None:
        """Compute the coefficients of the spline. This must be called in order to make interpolations."""
        pass
