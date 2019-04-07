from magicbot import AutonomousStateMachine, timed_state, state
import wpilib

from components.chassis import Chassis
from components.autoselector import AutoSelector, AutoSide, AutoMode

from controllers.ramsete import Ramsete
from trajectory.trajectorygenerator import TrajectoryGenerator, TrajectoryContraints
from autonomous.paths import Path
import numpy as np
from utils.geometry import RobotState


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
        return self.tg.generateTrajectory(
            poses,
            TrajectoryContraints(max_velocity=2.0),
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
                state_d_t.state.x,
                state_d_t.state.y,
                state_d_t.state.theta,
                state_d_t.velocity,
                state_d_t.velocity * state_d_t.state.curvature,
            )
            velocity = self.ramsete.update(state, state_d)
            print(f"{round(self.timer.getMsClock()/1000,3)}\t{round(state_d,3)}")
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
            self.next_state("stop")
    @state
    def test(self, initial_call):
        if initial_call:
            self.trajectory = self.getTrajectory(
                Path.TEST.getPoses(), _reversed=False
            )
            self.tg.drawSimulation(self.trajectory)
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("stop")
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
