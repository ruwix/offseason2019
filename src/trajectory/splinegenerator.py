"""
Implementation from:
NASA Ames Robotics "The Cheesy Poofs"
Team 254
"""
import itertools

import numpy as np

from utils.geometry import Pose, PoseWithCurvature, boundRadians

MIN_SAMPLE_SIZE = 1


def parameterizeSpline(
    spline,
    max_dx: float,
    max_dy: float,
    max_dtheta: float,
    t0: float = 0,
    t1: float = 1,
) -> np.array:
    dt = (t1 - t0) / MIN_SAMPLE_SIZE
    ret = np.empty(0, dtype=PoseWithCurvature)
    t = 0
    while t < t1:
        next_time = t + dt
        ret = np.append(
            ret, getSegmentArc(spline, t, next_time, max_dx, max_dy, max_dtheta)
        )
        t = next_time
    return ret


def parameterizeSplines(
    splines: np.array, max_dx: float, max_dy: float, max_dtheta: float
) -> np.array:
    ret = np.empty(1, dtype=PoseWithCurvature)
    ret[0] = splines[0].getPoseWithCurvature(0)
    for i, spline in enumerate(splines):
        samples = parameterizeSpline(spline, max_dx, max_dy, max_dtheta)
        ret = np.append(ret, samples)
    return ret


def getSegmentArc(
    spline: np.array,
    t0: float,
    t1: float,
    max_dx: float,
    max_dy: float,
    max_dtheta: float,
) -> np.array:
    p0 = spline.getPoint(t0)
    p1 = spline.getPoint(t1)
    r0 = spline.getHeading(t0)
    r1 = spline.getHeading(t1)
    transform_point = (p1 - p0).rotateBy(-r0)
    transformation = Pose(transform_point.x, transform_point.y, boundRadians(r1 - r0))
    twist = transformation.getTwist()
    if twist.dx > max_dx or twist.dy > max_dy or twist.dtheta > max_dtheta:
        return np.concatenate(
            (
                getSegmentArc(spline, t0, (t0 + t1) / 2, max_dx, max_dy, max_dtheta),
                getSegmentArc(spline, (t0 + t1) / 2, t1, max_dx, max_dy, max_dtheta),
            )
        )
    else:
        return np.array([spline.getPoseWithCurvature(t1)])
