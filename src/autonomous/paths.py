import numpy as np
from csv import reader, writer
from enum import Enum
import paths
import os


def loadPath(file: str) -> np.array:
    file = f"{os.path.dirname(paths.__file__)}/{file}"
    with open(file, "r") as path:
        _reader = reader(path)
        headings = next(_reader)
        assert headings == ["x", "y", "heading"]
        ret = np.array(list(_reader)).astype(float)
        for i in range(0, len(ret)):
            ret[i][2] = np.deg2rad(-ret[i][2])
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
