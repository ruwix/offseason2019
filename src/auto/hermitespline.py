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

    def interpolatePoint(self, t):
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

    def interpolateDerivative(self, t):
        """Interpolate the derivative along the spline where 0 <= t <= self.length."""
        assert 0 <= t <= self.length
        index = int(np.floor(t))
        t -= index
        dx = (3 * self.ax[index] * t ** 2) + (2 * self.bx[index] * t) + (self.cx[index])
        dy = (3 * self.ay[index] * t ** 2) + (2 * self.by[index] * t) + (self.cy[index])
        return np.array([dx, dy])

    def interpolate2ndDerivative(self, t):
        assert 0 <= t <= self.length
        """Interpolate the 2nd derivative along the spline where 0 <= t <= self.length."""
        index = int(np.floor(t))
        t -= index
        ddx = (6 * self.ax[index] * t) + (2 * self.bx[index])
        ddy = (6 * self.ay[index] * t) + (2 * self.by[index])
        return np.array([ddx, ddy])

    def interpolateCurvature(self, t):
        assert 0 <= t <= self.length
        """Interpolate the curvature along the spline where 0 <= t <= self.length."""
        dx, dy = self.interpolateDerivative(t)
        ddx, ddy = self.interpolate2ndDerivative(t)
        return (dx * ddy - dy * ddx) / ((dx * dx + dy * dy) * np.hypot(dx, dy))

    def interpolateHeading(self, t):
        """Interpolate a the angle of the tangent along the spline where 0 <= t <= self.length."""
        dx, dy = self.interpolateDerivative(t)
        return np.rad2deg(np.arctan2(dy, dx))

    def interpolatePose(self, t):
        """Interpolate a pose on the spline where 0 <= t <= self.length."""
        points = self.interpolatePoint(t)
        heading = self.interpolateHeading(t)
        return np.append(points, heading)

    def computeCoefficients(self):
        """Compute the coefficients of the spline equations. This must be called inorder to make interpolations."""
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
                + 2 * self.poses[i][0]
                - 2 * self.poses[i + 1][1]
            )
            self.by[i] = (
                -2 * self.dy0[i]
                - self.dy1[i]
                - 3 * self.poses[i][0]
                + 3 * self.poses[i + 1][1]
            )
            self.cy[i] = self.dy0[i]
            self.dy[i] = self.poses[i][0]

