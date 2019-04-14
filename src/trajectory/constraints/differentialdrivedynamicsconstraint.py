import numpy as np

from models.differentialdrive import DifferentialDrive
from trajectory.constraints.timingconstraint import TimingConstraint
from utils.geometry import PoseWithCurvature
from utils.mathextension import MinMax
from utils.physicalstates import ChassisState


class DifferentialDriveDynamicsConstraint(TimingConstraint):
    def __init__(self, drive: DifferentialDrive, max_voltage: float):
        self.drive = drive
        self.max_voltage = max_voltage

    def getMaxVelocity(self, state: PoseWithCurvature) -> float:
        return self.drive.getMaxAbsVelocity(state.curvature, self.max_voltage)

    def getMinMaxAcceleration(
        self, state: PoseWithCurvature, velocity: float
    ) -> MinMax:
        return self.drive.getMinMaxAcceleration(
            ChassisState(velocity, velocity * state.curvature),
            state.curvature,
            self.max_voltage,
        )
