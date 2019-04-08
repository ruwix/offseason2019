class QuadraticSpline:
    def __init__(self, p1, p2, p3):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3

    def getA(self):
        return (
            self.p3.x * (self.p2.y - self.p1.y)
            + self.p2.x * (self.p1.y - self.p3.y)
            + self.p1.x * (self.p3.y - self.p2.y)
        )

    def getB(self):
        return (
            self.p3.x * self.p3.x * (self.p1.y - self.p2.y)
            + self.p2.x * self.p2.x * (self.p3.y - self.p1.y)
            + self.p1.x * self.p1.x * (self.p2.y - self.p3.y)
        )

    def getVertexXCoordinate(self):
        return -self.getB() / (2 * self.getA())

