import numpy as np


def lerp(a, b, t):
    return a + (b - a) * np.clip(t, 0, 1)

