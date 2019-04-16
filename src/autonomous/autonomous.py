import numpy as np
import wpilib
from magicbot import AutonomousStateMachine, state, timed_state

from autonomous.paths import Path
from components.autoselector import AutoMode, AutoSelector, AutoSide
from components.chassis import Chassis
from components.localization import Localization
from controllers.ramsete import Ramsete
from models.differentialdrive import DifferentialDrive
from trajectory.constraints.angularaccelerationconstraint import (
    AngularAccelerationConstraint,
)
from trajectory.constraints.centripetalaccelerationconstraint import (
    CentripetalAccelerationConstraint,
)
from trajectory.constraints.differentialdrivedynamicsconstraint import (
    DifferentialDriveDynamicsConstraint,
)
from trajectory.constraints.timingconstraint import TimingConstraint
from trajectory.trajectorygenerator import TrajectoryGenerator
from utils import units
from utils.physicalstates import ChassisState


class Autonomous(AutonomousStateMachine):

    MODE_NAME = "Autonomous"
    DEFAULT = True

    chassis: Chassis
    autoselector: AutoSelector
    diff_drive: DifferentialDrive
    localization: Localization

    KBETA = 2.0
    KZETA = 0.7

    MAX_CENTRIPETAL_ACCELERATION: float = 2.4
    MAX_ANGULAR_ACCELERATION: float = 6.0
    MAX_VOLTAGE: float = 10

    START_VELOCITY: float = 0
    END_VELOCITY: float = 0
    MAX_VELOCITY: float = 3.0
    MAX_ACCELERATION: float = 1.2

    def __init__(self):
        self.timer = wpilib.Timer()
        self.trajectory = None
        self.tg = TrajectoryGenerator()
        self.ramsete = Ramsete(self.KBETA, self.KZETA)
        self.heading = 0

    def getTrajectory(self, poses, _reversed=False):
        constraints = np.array(
            [
                CentripetalAccelerationConstraint(self.MAX_CENTRIPETAL_ACCELERATION),
                AngularAccelerationConstraint(self.MAX_ANGULAR_ACCELERATION),
                DifferentialDriveDynamicsConstraint(self.diff_drive, self.MAX_VOLTAGE),
            ],
            dtype=TimingConstraint,
        )
        return self.tg.generateTrajectory(
            spline_poses=poses,
            constraints=constraints,
            start_velocity=self.START_VELOCITY,
            end_velocity=self.END_VELOCITY,
            max_velocity=self.MAX_VELOCITY,
            max_acceleration=self.MAX_ACCELERATION,
            _reversed=_reversed,
        )

    @state(first=True)
    def initMode(self, initial_call):
        side, mode = (
            AutoSide.LEFT,
            AutoMode.FRONT_ROCKET,
        )  # self.autoselector.getSelection()

        if side == AutoSide.LEFT:
            self.mirrored = False
            self.localization.setState(1.70, 1.09, 0)
        elif side == AutoSide.RIGHT:
            self.mirrored = True
            self.localization.setState(1.70, -1.09, 0)
        if mode == AutoMode.CROSS_LINE:
            self.next_state("crossLine")
        elif mode == AutoMode.BACK_ROCKET:
            self.next_state("startToBackRocket")
        elif mode == AutoMode.FRONT_ROCKET:
            self.next_state("startToFrontRocket")
        else:
            self.next_state("stop")

    def followTrajectory(self):
        if self.trajectory == None:
            return False
        if self.timer.get() < self.trajectory.length:
            state = self.localization.state
            state_d = self.trajectory.getState(self.timer.get())
            velocity = self.ramsete.update(state, state_d)
            print(f"{round(self.timer.get(),3)}\t{round(self.ramsete.getError(),3) }")
            self.chassis.setChassisState(velocity)
            return True
        else:
            return False

    @timed_state(duration=2.0, next_state="stop")
    def crossLine(self, initial_call):
        self.chassis.setWheelOutput(0.4, 0.4)

    @state
    def startToBackRocket(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.START_2_BACK_ROCKET.getPoses(mirrored=self.mirrored),
                _reversed=False,
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.setHeading(
                240 * units.radians_per_degree, "backRocketToLoadingStation"
            )

    @state
    def backRocketToLoadingStation(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.BACK_ROCKET_2_LOADING_STATION.getPoses(mirrored=self.mirrored),
                _reversed=False,
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.setHeading(0, "loadingStationToBackRocket")

    @state
    def loadingStationToBackRocket(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.LOADING_STATION_2_BACK_ROCKET.getPoses(mirrored=self.mirrored),
                _reversed=False,
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("stop")

    @state
    def startToFrontRocket(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.START_2_FRONT_ROCKET.getPoses(mirrored=self.mirrored),
                _reversed=False,
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.setHeading(
                180 * units.radians_per_degree, "frontRocketToLoadingStation"
            )

    @state
    def frontRocketToLoadingStation(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.FRONT_ROCKET_2_LOADING_STATION.getPoses(mirrored=self.mirrored),
                _reversed=False,
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.setHeading(0 * units.radians_per_degree, "loadingStationToFrontRocket")

    @state
    def loadingStationToFrontRocket(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.LOADING_STATION_2_FRONT_ROCKET.getPoses(mirrored=self.mirrored),
                _reversed=False,
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("stop")

    def setHeading(self, heading, next_state):
        self.heading = heading
        self._next_state = next_state
        self.next_state("turnToHeading")

    @state
    def turnToHeading(self, initial_call):
        if initial_call:
            self.chassis.setHeading(self.heading)
        if self.chassis.isAtHeading():
            self.next_state(self._next_state)

    @state
    def stop(self):
        self.chassis.setWheelOutput(0, 0)
