from enum import Enum


class AutoSide(Enum):
    LEFT = 0
    MIDDLE = 1
    RIGHT = 2


class AutoMode(Enum):
    CROSS_LINE = 0
    FRONT_ROCKET = 1
    SIDE_ROCKET = 2
    BACK_ROCKET = 3

    BAY = 4
    NOTHING = 5


class AutoSelector:
    def __init__(self):
        self.mode = AutoMode.CROSS_LINE
        self.side = AutoSide.LEFT

    def getMode(self):
        return self.mode

    def getSide(self):
        return self.side

    def getSelection(self):
        return (self.side, self.mode)

    def execute(self):
        self.mode = AutoMode.CROSS_LINE
        self.side = AutoSide.LEFT
