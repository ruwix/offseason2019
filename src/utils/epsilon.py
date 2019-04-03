EPSILON = 1e-9


def epsilonEquals(a, b, epsilon=EPSILON):
    return abs(a - b) < epsilon
