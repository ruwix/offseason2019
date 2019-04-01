from magicbot import AutonomousStateMachine, timed_state, state
import wpilib

from components.chassis import Chassis
from components.autoselector import AutoSelector, AutoSide, AutoMode

from autonomous.ramsete import Ramsete
from autonomous.trajectory import Trajectory
from autonomous.paths import Path
import numpy as np


class Autonomous(AutonomousStateMachine):

    MODE_NAME = "Autonomous"
    DEFAULT = True

    chassis: Chassis
    ramsete: Ramsete
    autoselector: AutoSelector

    def __init__(self):
        self.timer = wpilib.Timer()
        self.trajectory = None

    @state(first=True)
    def initMode(self, initial_call):
        side, mode = AutoSide.LEFT, AutoMode.ROCKET  # self.autoselector.getSelection()
        if side == AutoSide.LEFT:
            self.chassis.setState(1.70, -1.09, 0)
        if mode == 0:
            self.next_state("crossLine")
        elif side == AutoSide.LEFT and mode == AutoMode.ROCKET:
            self.next_state("leftStartToRocket")
        else:
            self.next_state("stop")

    def followTrajectory(self):
        if self.trajectory == None:
            return False
        self.trajectory.update(self.timer.get())
        if not self.trajectory.isFinished():
            state = self.chassis.state
            state_d = self.trajectory.getState()
            twist = self.ramsete.update(state, state_d)
            print(
                f"{round(self.timer.getMsClock()/1000,3)}\t{round(self.ramsete.getError(),3)}"
            )
            self.chassis.setChassisTwist(twist)
            return True
        else:
            return False

    @timed_state(duration=2, next_state="stop")
    def crossLine(self, initial_call):
        self.chassis.setChassisVelocity(36, 0)

    @state
    def leftStartToRocket(self, initial_call):
        if initial_call:
            self.trajectory = Trajectory(
                Path.START_2_LEFT_ROCKET.getPoses(), 3, reversed=False
            )
            self.trajectory.build()
            self.trajectory.writeCSV("logs/output.csv")
            self.trajectory.drawSimulation()
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("leftRocketBackup")

    @state
    def leftRocketBackup(self, initial_call):
        if initial_call:
            self.trajectory = Trajectory(
                Path.LEFT_ROCKET_BACKUP.getPoses(), 0.5, reversed=True
            )
            self.trajectory.build()
            self.trajectory.writeCSV("logs/output.csv")
            self.trajectory.drawSimulation()
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("leftRocketToLoadingStation")

    @state
    def leftRocketToLoadingStation(self, initial_call):
        if initial_call:
            self.trajectory = Trajectory(
                Path.LEFT_ROCKET_2_LOADING_STATION.getPoses(), 3, reversed=False
            )
            self.trajectory.build()
            self.trajectory.writeCSV("logs/output.csv")
            self.trajectory.drawSimulation()
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("loadingStationBackup")

    @state
    def loadingStationBackup(self, initial_call):
        if initial_call:
            self.trajectory = Trajectory(
                Path.LOADING_STATION_BACKUP.getPoses(), 3, reversed=True
            )
            self.trajectory.build()
            self.trajectory.writeCSV("logs/output.csv")
            self.trajectory.drawSimulation()
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("loadingStationToLeftRocket")

    @state
    def loadingStationToLeftRocket(self, initial_call):
        if initial_call:
            self.trajectory = Trajectory(
                Path.LOADING_STATION_2_LEFT_ROCKET.getPoses(), 3, reversed=False
            )
            self.trajectory.build()
            self.trajectory.writeCSV("logs/output.csv")
            self.trajectory.drawSimulation()
            self.timer.reset()
            self.timer.start()
        if not self.followTrajectory():
            self.next_state("stop")

    @state
    def stop(self):
        self.chassis.setWheelOutput(0, 0)
