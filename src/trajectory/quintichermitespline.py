from copy import copy

import numpy as np

from trajectory.hermitespline import HermiteSpline
from trajectory.quadraticspline import QuadraticSpline
from utils.geometry import Pose, Vector


class QuinticHermiteSpline(HermiteSpline):
    MAX_ITERATIONS = 100
    EPSILON = 1e-5
    MIN_DELTA = 1e-3
    STEP_SIZE = 1

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
        dddx = (60 * self.ax * t ** 2) + (24 * self.bx * t) + (6 * self.cx)
        dddy = (60 * self.ay * t ** 2) + (24 * self.by * t) + (6 * self.cy)
        return Vector(dddx, dddy)

    @staticmethod
    def optimizeSpline(splines: np.array) -> float:
        prev = HermiteSpline.getSumDCurvatures2(splines)
        for i in range(0, QuinticHermiteSpline.MAX_ITERATIONS):
            QuinticHermiteSpline.optimizationIteration(splines)
            current = HermiteSpline.getSumDCurvatures2(splines)
            if (prev - current) < QuinticHermiteSpline.MIN_DELTA:
                return current
            prev = current
        return prev

    @staticmethod
    def optimizationIteration(splines: np.array):
        if len(splines) <= 1:
            return
        control_points = np.empty(len(splines) - 1, dtype=Vector)
        for i in range(len(control_points)):
            control_points[i] = Vector()
        magnitude = 0
        for i in range(0, len(splines) - 1):
            if splines[i].start.isCollinear(splines[i + 1].start) or splines[
                i
            ].end.isCollinear(splines[i + 1].end):
                continue
            original = HermiteSpline.getSumDCurvatures2(splines)
            temp = copy(splines[i])
            temp1 = copy(splines[i + 1])
            splines[i].start.x = temp.start.x
            splines[i].end.x = temp.end.x
            splines[i].dx0 = temp.dx0
            splines[i].dx1 = temp.dx1
            splines[i].ddx0 = temp.ddx0
            splines[i].ddx1 = temp.ddx1 + QuinticHermiteSpline.EPSILON
            splines[i].start.y = temp.start.y
            splines[i].end.y = temp.end.y
            splines[i].dy0 = temp.dy0
            splines[i].dy1 = temp.dy1
            splines[i].ddy0 = temp.ddy0
            splines[i].ddy1 = temp.ddy1
            splines[i + 1].start.x = temp1.start.x
            splines[i + 1].end.x = temp1.end.x
            splines[i + 1].dx0 = temp1.dx0
            splines[i + 1].dx1 = temp1.dx1
            splines[i + 1].ddx0 = temp1.ddx0 + QuinticHermiteSpline.EPSILON
            splines[i + 1].ddx1 = temp1.ddx1
            splines[i + 1].start.y = temp1.start.y
            splines[i + 1].end.y = temp1.end.y
            splines[i + 1].dy0 = temp1.dy0
            splines[i + 1].dy1 = temp1.dy1
            splines[i + 1].ddy0 = temp1.ddy0
            splines[i + 1].ddy1 = temp1.ddy1
            control_points[i].x = (
                HermiteSpline.getSumDCurvatures2(splines) - original
            ) / QuinticHermiteSpline.EPSILON
            splines[i].start.x = temp.start.x
            splines[i].end.x = temp.end.x
            splines[i].dx0 = temp.dx0
            splines[i].dx1 = temp.dx1
            splines[i].ddx0 = temp.ddx0
            splines[i].ddx1 = temp.ddx1 + QuinticHermiteSpline.EPSILON
            splines[i].start.y = temp.start.y
            splines[i].end.y = temp.end.y
            splines[i].dy0 = temp.dy0
            splines[i].dy1 = temp.dy1
            splines[i].ddy0 = temp.ddy0
            splines[i].ddy1 = temp.ddy1 + QuinticHermiteSpline.EPSILON
            splines[i + 1].start.x = temp1.start.x
            splines[i + 1].end.x = temp1.end.x
            splines[i + 1].dx0 = temp1.dx0
            splines[i + 1].dx1 = temp1.dx1
            splines[i + 1].ddx0 = temp1.ddx0
            splines[i + 1].ddx1 = temp1.ddx1
            splines[i + 1].start.y = temp1.start.y
            splines[i + 1].end.y = temp1.end.y
            splines[i + 1].dy0 = temp1.dy0
            splines[i + 1].dy1 = temp1.dy1
            splines[i + 1].ddy0 = temp1.ddy0 + QuinticHermiteSpline.EPSILON
            splines[i + 1].ddy1 = temp1.ddy1
            control_points[i].y = (
                HermiteSpline.getSumDCurvatures2(splines) - original
            ) / QuinticHermiteSpline.EPSILON

            splines[i] = copy(temp)
            splines[i + 1] = copy(temp1)
            magnitude += (
                control_points[i].x * control_points[i].x
                + control_points[i].y * control_points[i].y
            )

        magnitude = np.sqrt(magnitude)

        p2 = Vector(0.0, HermiteSpline.getSumDCurvatures2(splines))

        for i in range(len(splines) - 1):
            if splines[i].start.isCollinear(splines[i + 1].start) or splines[
                i
            ].end.isCollinear(splines[i + 1].end):
                continue
            control_points[i].x *= QuinticHermiteSpline.STEP_SIZE / magnitude
            control_points[i].y *= QuinticHermiteSpline.STEP_SIZE / magnitude

            splines[i].ddx1 -= control_points[i].x
            splines[i].ddy1 -= control_points[i].y
            splines[i + 1].ddx0 -= control_points[i].x
            splines[i + 1].ddy0 -= control_points[i].y

            splines[i].computeCoefficients()
            splines[i + 1].computeCoefficients()

        p1 = Vector(
            -QuinticHermiteSpline.STEP_SIZE, HermiteSpline.getSumDCurvatures2(splines)
        )

        for i in range(len(splines) - 1):
            if splines[i].start.isCollinear(splines[i + 1].start) or splines[
                i
            ].end.isCollinear(splines[i + 1].end):
                continue

            splines[i].ddx1 += 2 * control_points[i].x
            splines[i].ddy1 += 2 * control_points[i].y
            splines[i + 1].ddx0 += 2 * control_points[i].x
            splines[i + 1].ddy0 += 2 * control_points[i].y

            splines[i].computeCoefficients()
            splines[i + 1].computeCoefficients()

        p3 = Vector(
            QuinticHermiteSpline.STEP_SIZE, HermiteSpline.getSumDCurvatures2(splines)
        )
        step_size = QuadraticSpline(p1, p2, p3).getVertexXCoordinate()
        for i in range(len(splines) - 1):
            if splines[i].start.isCollinear(splines[i + 1].start) or splines[
                i
            ].end.isCollinear(splines[i + 1].end):
                continue
            control_points[i].x *= 1 + step_size / QuinticHermiteSpline.STEP_SIZE
            control_points[i].y *= 1 + step_size / QuinticHermiteSpline.STEP_SIZE

            splines[i].ddx1 += control_points[i].x
            splines[i].ddy1 += control_points[i].y
            splines[i + 1].ddx0 += control_points[i].x
            splines[i + 1].ddy0 += control_points[i].y

            splines[i].computeCoefficients()
            splines[i + 1].computeCoefficients()

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
