import numpy as np


class Ramsete:
    def __init__(self, kbeta, kzeta):
        self.kbeta = kbeta  # 0 < kbeta; larger kbeta makes convergence more aggresive
        self.kzeta = kzeta  # 0 < kzeta < 1; larger kbeta provides more damping

    def update(self, pose, pose_d, vd):
        # Compute errors
        error = pose_d - pose
        # Compute trig functions
        cos_theta = np.cos(pose[2])
        sin_theta = np.sin(pose[2])
        # Compute gains
        k1 = 2 * self.kzeta * np.sqrt(vd[1] ** 2 + self.kbeta * vd[0] ** 2)
        k2 = self.kbeta
        k3 = k1
        # Find linear velocity
        v = vd[0] * np.cos(error[2]) + k1 * (
            error[0] * cos_theta + error[1] * sin_theta
        )
        # Find angular velocity
        w = (
            vd[1]
            + k2
            * vd[0]
            * np.sinc(error[2])
            * (error[1] * cos_theta - error[0] * sin_theta)
            + k3 * error[2]
        )
        return (v, w)
