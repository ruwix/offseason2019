import numpy as np
import wpilib
from magicbot import AutonomousStateMachine, state, timed_state

from autonomous.paths import Path
from components.autoselector import AutoMode, AutoSelector, AutoSide
from components.chassis import Chassis
from controllers.ramsete import Ramsete
from trajectory.constraints.angularaccelerationconstraint import (
    AngularAccelerationConstraint,
)
from trajectory.constraints.centripetalaccelerationconstraint import (
    CentripetalAccelerationConstraint,
)
from trajectory.constraints.timingconstraint import TimingConstraint
from trajectory.trajectorygenerator import TrajectoryGenerator
from utils.geometry import RobotState
from utils.physicalstates import ChassisState


class Autonomous(AutonomousStateMachine):

    MODE_NAME = "Autonomous"
    DEFAULT = True

    chassis: Chassis
    ramsete: Ramsete
    autoselector: AutoSelector

    def __init__(self):
        self.timer = wpilib.Timer()
        self.trajectory = None
        self.tg = TrajectoryGenerator()

    def getTrajectory(self, poses, _reversed=False):
        constraints = np.array(
            [
                # CentripetalAccelerationConstraint(5.0),
                # AngularAccelerationConstraint(4.0),
            ],
            dtype=TimingConstraint,
        )
        return self.tg.generateTrajectory(
            spline_poses=poses,
            constraints=constraints,
            start_velocity=0,
            end_velocity=0,
            max_velocity=3,
            max_acceleration=1.2,
            _reversed=_reversed,
        )

    @state(first=True)
    def initMode(self, initial_call):
        side, mode = AutoSide.LEFT, AutoMode.ROCKET  # self.autoselector.getSelection()

        self.next_state("leftStartToRocket")
        # if side == AutoSide.LEFT:
        #     self.chassis.setState(1.70, -1.09, 0)
        # if mode == 0:
        #     self.next_state("crossLine")
        # elif side == AutoSide.LEFT and mode == AutoMode.ROCKET:
        #     self.next_state("leftStartToRocket")
        # else:
        #     self.next_state("stop")

    def followTrajectory(self):
        if self.trajectory == None:
            return False
        if self.timer.get() < self.trajectory.length:
            state = self.chassis.state
            state_d_t = self.trajectory.getState(self.timer.get())
            state_d = RobotState(
                x=state_d_t.state.x,
                y=state_d_t.state.y,
                heading=state_d_t.state.theta,
                v=state_d_t.velocity,
                omega=state_d_t.velocity * state_d_t.state.curvature,
            )
            velocity = self.ramsete.update(state, state_d)
            print(
                f"{round(self.timer.get(),3)}\t{round(state_d.v,3)}\t{round(self.ramsete.getError(),3)}"
            )

            self.chassis.setChassisState(velocity)
            return True
        else:
            return False

    @timed_state(duration=2, next_state="stop")
    def crossLine(self, initial_call):
        self.chassis.setChassisVelocity(36, 0)

    @state
    def leftStartToRocket(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.START_2_LEFT_ROCKET.getPoses(), _reversed=False
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("leftRocketBackup")

    @state
    def leftRocketBackup(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.LEFT_ROCKET_BACKUP.getPoses(), _reversed=True
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("leftRocketToLoadingStation")

    @state
    def leftRocketToLoadingStation(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.LEFT_ROCKET_2_LOADING_STATION.getPoses(), _reversed=False
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("loadingStationBackup")

    @state
    def loadingStationBackup(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.LOADING_STATION_BACKUP.getPoses(), _reversed=True
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("loadingStationToLeftRocket")

    @state
    def loadingStationToLeftRocket(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.LOADING_STATION_2_LEFT_ROCKET.getPoses(), _reversed=False
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("stop")

    @state
    def stop(self):
        self.chassis.setWheelOutput(0, 0)
