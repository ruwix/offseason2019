import numpy as np
from utils.geometry import Pose, PoseWithCurvature
import itertools

MIN_SAMPLE_SIZE = 100


def zipPairs(iterable):
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)


def parameterizeSpline(spline, max_dx, max_dy, max_dtheta, t0=0, t1=1):
    dt = (t1 - t0) / MIN_SAMPLE_SIZE
    ret = np.empty(0, dtype=PoseWithCurvature)
    t = 0
    i = 0
    while t < t1:
        next_time = t + dt
        ret = np.append(
            ret, getSegmentArc(spline, t, next_time, max_dx, max_dy, max_dtheta)
        )
        t = next_time
        i += 1
    return ret


def parameterizeSplines(splines, max_dx, max_dy, max_dtheta):
    ret = np.empty(1, dtype=PoseWithCurvature)
    ret[0] = splines[0].getPoseWithCurvature(0)
    for i, spline in enumerate(splines):
        samples = parameterizeSpline(spline, max_dx, max_dy, max_dtheta)
        ret = np.append(ret, samples)
    return ret


def getSegmentArc(spline, t0, t1, max_dx, max_dy, max_dtheta):
    p0 = spline.getPoint(t0)
    p1 = spline.getPoint(t1)
    r0 = spline.getHeading(t0)
    r1 = spline.getHeading(t1)
    transform_point = (p1 - p0) * -r0
    transformation = Pose(transform_point.x, transform_point.y, r1 - r0)
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