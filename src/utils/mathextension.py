import numpy as np


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * np.clip(t, 0, 1)


class MinMax:
    def __init__(self, _min=np.NINF, _max=np.inf):
        self.min = _min
        self.max = _max

    def isValid(self) -> None:
        return self.min <= self.max


def getAngleDiff(a, b):
    diff = np.fmod(a - b, 2 * np.pi)
    if diff >= np.pi:
        diff -= 2 * np.pi
    elif diff < -np.pi:
        diff += 2 * np.pi
    return diff
