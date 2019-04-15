import ctre

from utils import units


class LazyPigeonIMU(ctre.PigeonIMU):
    def __init__(self, id):
        super().__init__(id)

    def getYaw(self):
        return self.getYawPitchRoll()[0] * units.radians_per_degree
