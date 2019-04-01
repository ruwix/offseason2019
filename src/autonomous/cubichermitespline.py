import numpy as np
from utils.geometry import Vector, Pose, Twist, RobotState
from autonomous.hermitespline import HermiteSpline


class CubicHermiteSpline(HermiteSpline):
    def __init__(self, poses: np.array):
        super().__init__(poses)
        self.dx0 = np.empty(self.length)
        self.dx1 = np.empty(self.length)
        self.ax = np.empty(self.length)
        self.bx = np.empty(self.length)
        self.cx = np.empty(self.length)
        self.dx = np.empty(self.length)
        self.dy0 = np.empty(self.length)
        self.dy1 = np.empty(self.length)
        self.ay = np.empty(self.length)
        self.by = np.empty(self.length)
        self.cy = np.empty(self.length)
        self.dy = np.empty(self.length)
        self.computeCoefficients()

    def getPoint(self, t: float) -> Vector:
        """Interpolate a point along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        x = (
            (self.ax[index] * t ** 3)
            + (self.bx[index] * t ** 2)
            + (self.cx[index] * t)
            + (self.dx[index])
        )
        y = (
            (self.ay[index] * t ** 3)
            + (self.by[index] * t ** 2)
            + (self.cy[index] * t)
            + (self.dy[index])
        )
        return Vector(x, y)

    def getD(self, t: float) -> Vector:
        """Interpolate the derivative along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        dx = (3 * self.ax[index] * t ** 2) + (2 * self.bx[index] * t) + (self.cx[index])
        dy = (3 * self.ay[index] * t ** 2) + (2 * self.by[index] * t) + (self.cy[index])
        return Vector(dx, dy)

    def getDD(self, t: float) -> Vector:
        """Interpolate the 2nd derivative along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        ddx = (6 * self.ax[index] * t) + (2 * self.bx[index])
        ddy = (6 * self.ay[index] * t) + (2 * self.by[index])
        return Vector(ddx, ddy)

    def getDDD(self, t: float) -> Vector:
        """Interpolate the 3rd derivative along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        dddx = 6 * self.ax[index]
        dddy = 6 * self.ay[index]
        return Vector(dddx, dddy)

    def optimizeSpline(self):
        pass

    def computeCoefficients(self) -> None:
        """Compute the coefficients of the spline. This must be called in order to make interpolations."""
        for i in range(0, self.length):
            scale = 2 * np.hypot(
                self.poses[i + 1].x - self.poses[i].x,
                self.poses[i + 1].y - self.poses[i].y,
            )
            self.dx0[i] = scale * np.cos(self.poses[i].theta)
            self.dx1[i] = scale * np.cos(self.poses[i + 1].theta)
            self.ax[i] = (
                self.dx0[i]
                + self.dx1[i]
                + 2 * self.poses[i].x
                - 2 * self.poses[i + 1].x
            )
            self.bx[i] = (
                -2 * self.dx0[i]
                - self.dx1[i]
                - 3 * self.poses[i].x
                + 3 * self.poses[i + 1].x
            )
            self.cx[i] = self.dx0[i]
            self.dx[i] = self.poses[i].x
            self.dy0[i] = scale * np.sin(self.poses[i].theta)
            self.dy1[i] = scale * np.sin(self.poses[i + 1].theta)
            self.ay[i] = (
                self.dy0[i]
                + self.dy1[i]
                + 2 * self.poses[i].y
                - 2 * self.poses[i + 1].y
            )
            self.by[i] = (
                -2 * self.dy0[i]
                - self.dy1[i]
                - 3 * self.poses[i].y
                + 3 * self.poses[i + 1].y
            )
            self.cy[i] = self.dy0[i]
            self.dy[i] = self.poses[i].y
