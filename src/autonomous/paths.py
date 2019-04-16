import os
from csv import reader
from enum import Enum

import numpy as np

import paths
from utils import units
from utils.geometry import Pose


def loadPath(file: str) -> np.array:
    file = f"{os.path.dirname(paths.__file__)}/{file}"
    with open(file, "r") as path:
        _reader = reader(path)
        headings = next(_reader)
        assert headings == ["x", "y", "heading"]
        data = np.array(list(_reader)).astype(float)
        ret = np.empty(len(data), dtype=Pose)
        for i, pose in enumerate(data):
            ret[i] = Pose(pose[0], pose[1], pose[2] * units.radians_per_degree)
        path.close()
        return ret


class Path(Enum):
    START_2_BACK_ROCKET = "start2backrocket.csv"
    BACK_ROCKET_2_LOADING_STATION = "backrocket2loadingstation.csv"
    LOADING_STATION_2_BACK_ROCKET = "loadingstation2backrocket.csv"
    START_2_FRONT_ROCKET = "start2frontrocket.csv"
    FRONT_ROCKET_2_LOADING_STATION = "frontrocket2loadingstation.csv"
    LOADING_STATION_2_FRONT_ROCKET = "loadingstation2frontrocket.csv"

    def getPoses(self, mirrored: bool = False):
        ret = loadPath(self.value)
        if mirrored:
            for i in range(len(ret)):
                ret[i].y *= -1
                ret[i].theta *= -1
        return ret
