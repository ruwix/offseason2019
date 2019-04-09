import os
from csv import reader
from enum import Enum

import numpy as np

import paths
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
            ret[i] = Pose(pose[0], pose[1], np.deg2rad(-pose[2]))
        path.close()
        return ret


class Path(Enum):
    START_2_LEFT_ROCKET = "start2leftrocket.csv"
    LEFT_ROCKET_BACKUP = "leftrocketbackup.csv"
    LEFT_ROCKET_2_LOADING_STATION = "leftrocket2loadingstation.csv"
    LOADING_STATION_BACKUP = "loadingstationbackup.csv"
    LOADING_STATION_2_LEFT_ROCKET = "loadingstation2leftrocket.csv"
    CIRCLE = "circle.csv"

    def getPoses(self):
        return loadPath(self.value)
