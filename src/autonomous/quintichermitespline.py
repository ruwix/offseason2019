import numpy as np
from utils.geometry import Vector, Pose, Twist, RobotState
from autonomous.hermitespline import HermiteSpline
from copy import copy
import time


class QuinticHermiteSpline(HermiteSpline):
    MAX_ITERATIONS = 100
    EPSILON = 1e-5
    MIN_DELTA = 1e-3

    def __init__(self, poses: np.array):
        super().__init__(poses)
        self.dx0 = np.empty(self.length)
        self.dx1 = np.empty(self.length)
        self.ddx0 = np.empty(self.length)
        self.ddx1 = np.empty(self.length)
        self.ax = np.empty(self.length)
        self.bx = np.empty(self.length)
        self.cx = np.empty(self.length)
        self.dx = np.empty(self.length)
        self.ex = np.empty(self.length)
        self.fx = np.empty(self.length)
        self.dy0 = np.empty(self.length)
        self.ddy0 = np.empty(self.length)
        self.ddy1 = np.empty(self.length)
        self.dy1 = np.empty(self.length)
        self.ay = np.empty(self.length)
        self.by = np.empty(self.length)
        self.cy = np.empty(self.length)
        self.dy = np.empty(self.length)
        self.ey = np.empty(self.length)
        self.fy = np.empty(self.length)
        self.computeCoefficients()

    def getPoint(self, t: float) -> Vector:
        """Interpolate a point along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        x = (
            (self.ax[index] * t ** 5)
            + (self.bx[index] * t ** 4)
            + (self.cx[index] * t ** 3)
            + (self.dx[index] * t ** 2)
            + (self.ex[index] * t)
            + (self.fx[index])
        )
        y = (
            (self.ay[index] * t ** 5)
            + (self.by[index] * t ** 4)
            + (self.cy[index] * t ** 3)
            + (self.dy[index] * t ** 2)
            + (self.ey[index] * t)
            + (self.fy[index])
        )
        return Vector(x, y)

    def getD(self, t: float) -> Vector:
        """Interpolate the derivative along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        dx = (
            (5 * self.ax[index] * t ** 4)
            + (4 * self.bx[index] * t ** 3)
            + (3 * self.cx[index] * t ** 2)
            + (2 * self.dx[index] * t)
            + (self.ex[index])
        )
        dy = (
            (5 * self.ay[index] * t ** 4)
            + (4 * self.by[index] * t ** 3)
            + (3 * self.cy[index] * t ** 2)
            + (2 * self.dy[index] * t)
            + (self.ey[index])
        )
        return Vector(dx, dy)

    def getDD(self, t: float) -> Vector:
        """Interpolate the 2nd derivative along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        ddx = (
            (20 * self.ax[index] * t ** 3)
            + (12 * self.bx[index] * t ** 2)
            + (6 * self.cx[index] * t)
            + (2 * self.dx[index])
        )
        ddy = (
            (20 * self.ay[index] * t ** 3)
            + (12 * self.by[index] * t ** 2)
            + (6 * self.cy[index] * t)
            + (2 * self.dy[index])
        )
        return Vector(ddx, ddy)

    def getDDD(self, t: float) -> Vector:
        """Interpolate the 3rd derivative along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        dddx = (
            (60 * self.ax[index] * t ** 2)
            + (24 * self.bx[index] * t)
            + (6 * self.dx[index])
        )
        dddy = (
            (60 * self.ay[index] * t ** 2)
            + (24 * self.by[index] * t)
            + (6 * self.dy[index])
        )
        return Vector(dddx, dddy)

    @staticmethod
    def optimizeSpline(spline):
        if spline.length <= 1:
            return
        prev = spline.getSumDCurvature2()
        for _ in range(0, QuinticHermiteSpline.MAX_ITERATIONS):
            QuinticHermiteSpline.optimizationIteration(spline)
            current = spline.getSumDCurvature2()
            if (prev - current) < QuinticHermiteSpline.MIN_DELTA:
                return current
            prev = current
        return prev

    @staticmethod
    def optimizationIteration(spline):
        if spline.length <= 1:
            return
        control_points = np.empty(spline.length, dtype=Vector)
        temp = copy(spline)
        original_sum = spline.getSumDCurvature2()
        magnitude = 0
        for i in range(0, spline.length - 1):
            control_points[i] = Vector()
            temp.ddx1[i] += QuinticHermiteSpline.EPSILON
            temp.ddx0[i + 1] += QuinticHermiteSpline.EPSILON
            temp.computeCoefficients()
            ddx = (
                temp.getSumDCurvature2() - original_sum
            ) / QuinticHermiteSpline.EPSILON
            temp = copy(spline)
            temp.ddy1[i] += QuinticHermiteSpline.EPSILON
            temp.ddy0[i + 1] += QuinticHermiteSpline.EPSILON
            temp.computeCoefficients()
            ddy = (
                temp.getSumDCurvature2() - original_sum
            ) / QuinticHermiteSpline.EPSILON
            control_points[i] = Vector(ddx, ddy)
            temp = copy(spline)
            magnitude += control_points[i].x ** 2 + control_points[i].y ** 2
        magnitude = np.sqrt(magnitude)

    def computeCoefficients(self) -> None:
        """Compute the coefficients of the spline. This must be called in order to make interpolations."""
        for i in range(0, self.length):
            scale = 1.2 * np.hypot(
                self.poses[i + 1].x - self.poses[i].x,
                self.poses[i + 1].y - self.poses[i].y,
            )
            self.dx0[i] = scale * np.cos(self.poses[i].theta)
            self.dx1[i] = scale * np.cos(self.poses[i + 1].theta)
            self.ddx0[i] = 0
            self.ddx1[i] = 0
            self.ax[i] = (
                -6 * self.poses[i].x
                - 3 * self.dx0[i]
                - 0.5 * self.ddx0[i]
                + 0.5 * self.ddx1[i]
                - 3 * self.dx1[i]
                + 6 * self.poses[i + 1].x
            )
            self.bx[i] = (
                15 * self.poses[i].x
                + 8 * self.dx0[i]
                + 1.5 * self.ddx0[i]
                - self.ddx1[i]
                + 7 * self.dx1[i]
                - 15 * self.poses[i + 1].x
            )
            self.cx[i] = (
                -10 * self.poses[i].x
                - 6 * self.dx0[i]
                - 1.5 * self.ddx0[i]
                + 0.5 * self.ddx1[i]
                - 4 * self.dx1[i]
                + 10 * self.poses[i + 1].x
            )
            self.dx[i] = 0.5 * self.ddx0[i]
            self.ex[i] = self.dx0[i]
            self.fx[i] = self.poses[i].x
            self.dy0[i] = scale * np.sin(self.poses[i].theta)
            self.dy1[i] = scale * np.sin(self.poses[i + 1].theta)
            self.ddy0[i] = 0
            self.ddy1[i] = 0
            self.ay[i] = (
                -6 * self.poses[i].y
                - 3 * self.dy0[i]
                - 0.5 * self.ddy0[i]
                + 0.5 * self.ddy1[i]
                - 3 * self.dy1[i]
                + 6 * self.poses[i + 1].y
            )
            self.by[i] = (
                15 * self.poses[i].y
                + 8 * self.dy0[i]
                + 1.5 * self.ddy0[i]
                - self.ddy1[i]
                + 7 * self.dy1[i]
                - 15 * self.poses[i + 1].y
            )
            self.cy[i] = (
                -10 * self.poses[i].y
                - 6 * self.dy0[i]
                - 1.5 * self.ddy0[i]
                + 0.5 * self.ddy1[i]
                - 4 * self.dy1[i]
                + 10 * self.poses[i + 1].y
            )
            self.dy[i] = 0.5 * self.ddy0[i]
            self.ey[i] = self.dy0[i]
            self.fy[i] = self.poses[i].y

