import numpy as np


class HermiteSpline:
    def __init__(self, poses):
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

    def getPoint(self, t):
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
        return np.array([x, y])

    def getD(self, t):
        """Interpolate the derivative along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        dx = (3 * self.ax[index] * t ** 2) + (2 * self.bx[index] * t) + (self.cx[index])
        dy = (3 * self.ay[index] * t ** 2) + (2 * self.by[index] * t) + (self.cy[index])
        return np.array([dx, dy])

    def getDD(self, t):
        """Interpolate the 2nd derivative along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        ddx = (6 * self.ax[index] * t) + (2 * self.bx[index])
        ddy = (6 * self.ay[index] * t) + (2 * self.by[index])
        return np.array([ddx, ddy])

    def getDDD(self, t):
        """Interpolate the 3rd derivative along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        dddx = 6 * self.ax[index]
        dddy = 6 * self.ay[index]
        return np.array([dddx, dddy])

    def getCurvature(self, t):
        """Interpolate the curvature along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        dx, dy = self.getD(t)
        ddx, ddy = self.getDD(t)
        dx2dy2 = dx ** 2 + dy ** 2
        return (dx * ddy - dy * ddx) / (dx2dy2 ** 1.5)

    def getRadius(self, t):
        """Interpolate the curvature along the spline where 0 <= t <= self.length."""
        c = self.getCurvature(t)
        if c == 0:
            return float("inf")
        else:
            return 1 / self.getCurvature(t)

    def getDCurvature(self, t):
        assert 0 <= t <= self.length
        """Interpolate the derivative of the curvature along the spline where 0 <= t <= self.length."""
        dx, dy = self.getD(t)
        ddx, ddy = self.getDD(t)
        dddx, dddy = self.getDDD(t)
        dx2dy2 = dx ** 2 + dy ** 2
        dc = (dx * dddy - dy * dddx) / (dx2dy2 ** 1.5) - 3 * (dx * ddy - dy * ddx) * (
            dx * ddx + dy * ddy
        ) / (dx2dy2 ** 2.5)
        return dc

    def getHeading(self, t):
        """Interpolate the angle of the tangent along the spline where 0 <= t <= self.length."""
        dx, dy = self.getD(t)
        return np.rad2deg(np.arctan2(dy, dx))

    def getPose(self, t):
        """Interpolate the pose on the spline where 0 <= t <= self.length."""
        points = self.getPoint(t)
        heading = self.getHeading(t)
        return np.append(points, heading)

    def getLinearVelocity(self, t):
        """Interpolate the linear velocity of a particle traveling along the spline where 0 <= t <= self.length."""
        dx, dy = self.getD(t)
        return np.hypot(dx, dy)

    def getAngularVelocity(self, t):
        """Interpolate the angular velocity of a particle traveling along the spline where 0 <= t <= self.length."""
        omega = self.getLinearVelocity(t) / self.getRadius(t)
        return -np.rad2deg(omega)

    def getTwist(self, t):
        """Interpolate the twist of a particle traveling along the spline at timestamp t where 0 <= t <= self.length."""
        v = self.getLinearVelocity(t)
        omega = self.getAngularVelocity(t)
        return np.append(v, omega)

    def getArcLength(self, sample_size=0.01):
        arc_length = 0
        for i in range(0, self.length * int(1 / sample_size) - 1):
            x0, y0 = self.getPoint(i * sample_size)
            x1, y1 = self.getPoint((i + 1) * sample_size)
            arc_length += np.hypot(x1 - x0, y1 - y0)
        return arc_length

    def computeCoefficients(self):
        """Compute the coefficients of the spline. This must be called in order to make interpolations."""
        for i in range(0, self.length):
            scale = 2 * np.hypot(
                self.poses[i + 1][0] - self.poses[i][0],
                self.poses[i + 1][1] - self.poses[i][1],
            )
            self.dx0[i] = scale * np.cos(np.deg2rad(-self.poses[i][2]))
            self.dx1[i] = scale * np.cos(np.deg2rad(-self.poses[i + 1][2]))
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
            self.dy0[i] = scale * np.sin(np.deg2rad(-self.poses[i][2]))
            self.dy1[i] = scale * np.sin(np.deg2rad(-self.poses[i + 1][2]))
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

