import numpy as np
from utils.geometry import Vector, Pose, Twist, RobotState


class HermiteSpline:
    def __init__(self, poses: np.array):
        self.poses = poses
        self.length = len(self.poses) - 1
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

    def getCurvature(self, t: float) -> float:
        """Interpolate the curvature along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        d = self.getD(t)
        dd = self.getDD(t)
        dx2dy2 = d.x ** 2 + d.y ** 2
        return (d.x * dd.y - d.y * dd.x) / (dx2dy2 ** 1.5)

    def getRadius(self, t: float) -> float:
        """Interpolate the curvature along the spline where 0 <= t <= self.length."""
        c = self.getCurvature(t)
        if c == 0:
            return float("inf")
        else:
            return 1 / self.getCurvature(t)

    def getDCurvature(self, t: float) -> float:
        assert 0 <= t <= self.length
        """Interpolate the derivative of the curvature along the spline where 0 <= t <= self.length."""
        d = self.getD(t)
        dd = self.getDD(t)
        ddd = self.getDDD(t)
        dx2dy2 = d.x ** 2 + d.y ** 2
        dc = (d.x * ddd.y - d.y * ddd.x) / (dx2dy2 ** 1.5) - 3 * (
            d.x * dd.y - d.y * dd.x
        ) * (d.x * dd.x + d.y * dd.y) / (dx2dy2 ** 2.5)
        return dc

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
        omega = self.getLinearVelocity(t) / self.getRadius(t)
        return omega

    def getTwist(self, t: float) -> Twist:
        """Interpolate the twist of a particle traveling along the spline at timestamp t where 0 <= t <= self.length."""
        v = self.getLinearVelocity(t)
        omega = self.getAngularVelocity(t)
        return Twist(v, omega)

    def getArcLength(self, sample_size: float = 0.01) -> float:
        arc_length = 0
        for i in range(0, self.length * int(1 / sample_size) - 1):
            p0 = self.getPoint(i * sample_size)
            p1 = self.getPoint((i + 1) * sample_size)
            arc_length += p0.getDistance(p1)
        return arc_length

    def computeCoefficients(self) -> None:
        """Compute the coefficients of the spline. This must be called in order to make interpolations."""
        for i in range(0, self.length):
            scale = 2 * np.hypot(
                self.poses[i + 1][0] - self.poses[i][0],
                self.poses[i + 1][1] - self.poses[i][1],
            )
            self.dx0[i] = scale * np.cos(self.poses[i][2])
            self.dx1[i] = scale * np.cos(self.poses[i + 1][2])
            self.ax[i] = (
                self.dx0[i]
                + self.dx1[i]
                + 2 * self.poses[i][0]
                - 2 * self.poses[i + 1][0]
            )
            self.bx[i] = (
                -2 * self.dx0[i]
                - self.dx1[i]
                - 3 * self.poses[i][0]
                + 3 * self.poses[i + 1][0]
            )
            self.cx[i] = self.dx0[i]
            self.dx[i] = self.poses[i][0]
            self.dy0[i] = scale * np.sin(self.poses[i][2])
            self.dy1[i] = scale * np.sin(self.poses[i + 1][2])
            self.ay[i] = (
                self.dy0[i]
                + self.dy1[i]
                + 2 * self.poses[i][1]
                - 2 * self.poses[i + 1][1]
            )
            self.by[i] = (
                -2 * self.dy0[i]
                - self.dy1[i]
                - 3 * self.poses[i][1]
                + 3 * self.poses[i + 1][1]
            )
            self.cy[i] = self.dy0[i]
            self.dy[i] = self.poses[i][1]

