import numpy as np
from utils.geometry import Vector, Pose, RobotState
from autonomous.hermitespline import HermiteSpline
from copy import copy
import time


class QuinticHermiteSpline(HermiteSpline):
    MAX_ITERATIONS = 100
    EPSILON = 1e-5
    MIN_DELTA = 1e-3

    def __init__(self, start: Pose, end: Pose):
        super().__init__(start, end)
        self.computeCoefficients()

    def getPoint(self, t: float) -> Vector:
        """Interpolate a point along the spline."""
        x = (
            (self.ax * t ** 5)
            + (self.bx * t ** 4)
            + (self.cx * t ** 3)
            + (self.dx * t ** 2)
            + (self.ex * t)
            + (self.fx)
        )
        y = (
            (self.ay * t ** 5)
            + (self.by * t ** 4)
            + (self.cy * t ** 3)
            + (self.dy * t ** 2)
            + (self.ey * t)
            + (self.fy)
        )
        return Vector(x, y)

    def getD(self, t: float) -> Vector:
        """Interpolate the derivative along the spline."""
        dx = (
            (5 * self.ax * t ** 4)
            + (4 * self.bx * t ** 3)
            + (3 * self.cx * t ** 2)
            + (2 * self.dx * t)
            + (self.ex)
        )
        dy = (
            (5 * self.ay * t ** 4)
            + (4 * self.by * t ** 3)
            + (3 * self.cy * t ** 2)
            + (2 * self.dy * t)
            + (self.ey)
        )
        return Vector(dx, dy)

    def getDD(self, t: float) -> Vector:
        """Interpolate the 2nd derivative along the spline."""
        ddx = (
            (20 * self.ax * t ** 3)
            + (12 * self.bx * t ** 2)
            + (6 * self.cx * t)
            + (2 * self.dx)
        )
        ddy = (
            (20 * self.ay * t ** 3)
            + (12 * self.by * t ** 2)
            + (6 * self.cy * t)
            + (2 * self.dy)
        )
        return Vector(ddx, ddy)

    def getDDD(self, t: float) -> Vector:
        """Interpolate the 3rd derivative along the spline."""
        dddx = (60 * self.ax * t ** 2) + (24 * self.bx * t) + (6 * self.dx)
        dddy = (60 * self.ay * t ** 2) + (24 * self.by * t) + (6 * self.dy)
        return Vector(dddx, dddy)

    @staticmethod
    def optimizeSpline(spline):
        pass
        # if spline.length <= 1:
        #     return
        # prev = spline.getSumDCurvature2()
        # for _ in range(0, QuinticHermiteSpline.MAX_ITERATIONS):
        #     QuinticHermiteSpline.optimizationIteration(spline)
        #     current = spline.getSumDCurvature2()
        #     if (prev - current) < QuinticHermiteSpline.MIN_DELTA:
        #         return current
        #     prev = current
        # return prev

    @staticmethod
    def optimizationIteration(spline):
        pass
        # if spline.length <= 1:
        #     return
        # control_points = np.empty(spline.length, dtype=Vector)
        # temp = copy(spline)
        # original_sum = spline.getSumDCurvature2()
        # magnitude = 0
        # for i in range(0, spline.length - 1):
        #     control_points = Vector()
        #     temp.ddx1 += QuinticHermiteSpline.EPSILON
        #     temp.ddx0 += QuinticHermiteSpline.EPSILON
        #     temp.computeCoefficients()
        #     ddx = (
        #         temp.getSumDCurvature2() - original_sum
        #     ) / QuinticHermiteSpline.EPSILON
        #     temp = copy(spline)
        #     temp.ddy1 += QuinticHermiteSpline.EPSILON
        #     temp.ddy0 += QuinticHermiteSpline.EPSILON
        #     temp.computeCoefficients()
        #     ddy = (
        #         temp.getSumDCurvature2() - original_sum
        #     ) / QuinticHermiteSpline.EPSILON
        #     control_points = Vector(ddx, ddy)
        #     temp = copy(spline)
        #     magnitude += control_points.x ** 2 + control_points.y ** 2
        # magnitude = np.sqrt(magnitude)

    def computeCoefficients(self) -> None:
        """Compute the coefficients of the spline. This must be called in order to make interpolations."""
        scale = 1.2 * np.hypot(self.end.x - self.start.x, self.end.y - self.start.y)
        self.dx0 = scale * np.cos(self.start.theta)
        self.dx1 = scale * np.cos(self.end.theta)
        self.ddx0 = 0
        self.ddx1 = 0
        self.ax = (
            -6 * self.start.x
            - 3 * self.dx0
            - 0.5 * self.ddx0
            + 0.5 * self.ddx1
            - 3 * self.dx1
            + 6 * self.end.x
        )
        self.bx = (
            15 * self.start.x
            + 8 * self.dx0
            + 1.5 * self.ddx0
            - self.ddx1
            + 7 * self.dx1
            - 15 * self.end.x
        )
        self.cx = (
            -10 * self.start.x
            - 6 * self.dx0
            - 1.5 * self.ddx0
            + 0.5 * self.ddx1
            - 4 * self.dx1
            + 10 * self.end.x
        )
        self.dx = 0.5 * self.ddx0
        self.ex = self.dx0
        self.fx = self.start.x
        self.dy0 = scale * np.sin(self.start.theta)
        self.dy1 = scale * np.sin(self.end.theta)
        self.ddy0 = 0
        self.ddy1 = 0
        self.ay = (
            -6 * self.start.y
            - 3 * self.dy0
            - 0.5 * self.ddy0
            + 0.5 * self.ddy1
            - 3 * self.dy1
            + 6 * self.end.y
        )
        self.by = (
            15 * self.start.y
            + 8 * self.dy0
            + 1.5 * self.ddy0
            - self.ddy1
            + 7 * self.dy1
            - 15 * self.end.y
        )
        self.cy = (
            -10 * self.start.y
            - 6 * self.dy0
            - 1.5 * self.ddy0
            + 0.5 * self.ddy1
            - 4 * self.dy1
            + 10 * self.end.y
        )
        self.dy = 0.5 * self.ddy0
        self.ey = self.dy0
        self.fy = self.start.y

