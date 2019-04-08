from trajectory.constraints.timingconstraint import TimingConstraint
from utils.mathextension import MinMax
import numpy as np
from utils.physicalstates import ChassisState
from models.differentialdrive import DifferentialDrive
from utils.geometry import PoseWithCurvature


class DifferentialDriveDynamicsConstraint(TimingConstraint):
    def __init__(self, drive: DifferentialDrive, max_voltage: float):
        self.drive = drive
        self.max_voltage = max_voltage

    def getMaxVelocity(self, state: PoseWithCurvature) -> float:
        return drive.getAbsMaxVelocity(state.curvature, self.max_voltage)

    def getMinMaxAcceleration(
        self, state: PoseWithCurvature, velocity: float
    ) -> MinMax:
        acceleration = drive.getMinMaxAcceleration(
            ChassisState(velocity, velocity * state.curvature),
            state.curvature,
            self.max_voltage,
        )

