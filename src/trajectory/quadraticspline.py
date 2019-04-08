from utils.geometry import Vector


class QuadraticSpline:
    def __init__(self, p1: Vector, p2: Vector, p3: Vector):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3

    def getA(self) -> float:
        return (
            self.p3.x * (self.p2.y - self.p1.y)
            + self.p2.x * (self.p1.y - self.p3.y)
            + self.p1.x * (self.p3.y - self.p2.y)
        )

    def getB(self) -> float:
        return (
            self.p3.x * self.p3.x * (self.p1.y - self.p2.y)
            + self.p2.x * self.p2.x * (self.p3.y - self.p1.y)
            + self.p1.x * self.p1.x * (self.p2.y - self.p3.y)
        )

    def getVertexXCoordinate(self) -> float:
        return -self.getB() / (2 * self.getA())

