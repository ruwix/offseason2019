import numpy as np


def lerp(a, b, t):
    return a + (b - a) * np.clip(t, 0, 1)


class MinMax:
    def __init__(self, _min=np.NINF, _max=np.inf):
        self.min = _min
        self.max = _max

    def isValid(self):
        return self.min <= self.max