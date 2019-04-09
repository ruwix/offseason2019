EPSILON = 1e-9


def epsilonEquals(a: float, b: float, epsilon: float = EPSILON) -> float:
    return abs(a - b) < epsilon
