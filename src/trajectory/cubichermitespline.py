import numpy as np

from trajectory.hermitespline import HermiteSpline
from utils.geometry import Pose, RobotState, Vector


class CubicHermiteSpline(HermiteSpline):
    def __init__(self, start: Pose, end: Pose):
        super().__init__(start, end)
        self.computeCoefficients()

    def getPoint(self, t: float) -> Vector:
        """Interpolate a point along the spline."""
        x = (self.ax * t ** 3) + (self.bx * t ** 2) + (self.cx * t) + (self.dx)
        y = (self.ay * t ** 3) + (self.by * t ** 2) + (self.cy * t) + (self.dy)
        return Vector(x, y)

    def getD(self, t: float) -> Vector:
        """Interpolate the derivative along the spline."""
        dx = (3 * self.ax * t ** 2) + (2 * self.bx * t) + (self.cx)
        dy = (3 * self.ay * t ** 2) + (2 * self.by * t) + (self.cy)
        return Vector(dx, dy)

    def getDD(self, t: float) -> Vector:
        """Interpolate the 2nd derivative along the spline."""
        ddx = (6 * self.ax * t) + (2 * self.bx)
        ddy = (6 * self.ay * t) + (2 * self.by)
        return Vector(ddx, ddy)

    def getDDD(self, t: float) -> Vector:
        """Interpolate the 3rd derivative along the spline."""
        dddx = 6 * self.ax
        dddy = 6 * self.ay
        return Vector(dddx, dddy)

    def optimizeSpline(self):
        pass

    def computeCoefficients(self) -> None:
        """Compute the coefficients of the spline. This must be called in order to make interpolations."""
        scale = 2 * np.hypot(self.end.x - self.start.x, self.end.y - self.start.y)
        self.dx0 = scale * np.cos(self.start.theta)
        self.dx1 = scale * np.cos(self.end.theta)
        self.ax = self.dx0 + self.dx1 + 2 * self.start.x - 2 * self.end.x
        self.bx = -2 * self.dx0 - self.dx1 - 3 * self.start.x + 3 * self.end.x
        self.cx = self.dx0
        self.dx = self.start.x
        self.dy0 = scale * np.sin(self.start.theta)
        self.dy1 = scale * np.sin(self.end.theta)
        self.ay = self.dy0 + self.dy1 + 2 * self.start.y - 2 * self.end.y
        self.by = -2 * self.dy0 - self.dy1 - 3 * self.start.y + 3 * self.end.y
        self.cy = self.dy0
        self.dy = self.start.y
