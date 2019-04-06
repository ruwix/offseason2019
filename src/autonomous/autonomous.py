from magicbot import AutonomousStateMachine, timed_state, state
import wpilib

from components.chassis import Chassis
from components.autoselector import AutoSelector, AutoSide, AutoMode

from autonomous.ramsete import Ramsete
from autonomous.trajectory import TrajectoryGenerator, TrajectoryContraints
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
        tg = TrajectoryGenerator()
        d = tg.generateTrajectory(
            Path.START_2_LEFT_ROCKET.getPoses(),
            TrajectoryContraints(max_velocity=2.0),
            reversed=False,
        )
        t = 0
        while t < d.length:
            print(round(d.getPose(t), 3))
            t += 0.01
        self.next_state_now("stop")
        # if side == AutoSide.LEFT:
        #     self.chassis.setState(1.70, -1.09, 0)
        # if mode == 0:
        #     self.next_state("crossLine")
        # elif side == AutoSide.LEFT and mode == AutoMode.ROCKET:
        #     self.next_state("leftStartToRocket")
        # else:
        #     self.next_state("stop")

    # def followTrajectory(self):
    #     if self.trajectory == None:
    #         return False
    #     self.trajectory.update(self.timer.get())
    #     if not self.trajectory.isFinished():
    #         state = self.chassis.state
    #         state_d = self.trajectory.getState()
    #         velocity = self.ramsete.update(state, state_d)
    #         # print(
    #         #     f"{round(self.timer.getMsClock()/1000,3)}\t{round(self.ramsete.getError(),3)}"
    #         # )
    #         self.chassis.setChassisState(velocity)
    #         return True
    #     else:
    #         return False

    # @timed_state(duration=2, next_state="stop")
    # def crossLine(self, initial_call):
    #     self.chassis.setChassisVelocity(36, 0)

    # @state
    # def leftStartToRocket(self, initial_call):
    #     if initial_call:
    #         self.trajectory = self.createTrajectory(
    #             Path.START_2_LEFT_ROCKET.getPoses(), reversed=False
    #         )
    #         self.trajectory.build()
    #         self.trajectory.writeCSV("logs/output.csv")
    #         self.trajectory.drawSimulation()
    #         self.timer.reset()
    #         self.timer.start()
    #     if not self.followTrajectory():
    #         self.next_state("stop")

    # @state
    # def leftRocketBackup(self, initial_call):
    #     if initial_call:
    #         self.trajectory = self.createTrajectory(
    #             Path.LEFT_ROCKET_BACKUP.getPoses(), reversed=True
    #         )
    #         self.trajectory.build()
    #         self.trajectory.writeCSV("logs/output.csv")
    #         self.trajectory.drawSimulation()
    #         self.timer.reset()
    #         self.timer.start()
    #     if not self.followTrajectory():
    #         self.next_state("leftRocketToLoadingStation")

    # @state
    # def leftRocketToLoadingStation(self, initial_call):
    #     if initial_call:
    #         self.trajectory = self.createTrajectory(
    #             Path.LEFT_ROCKET_2_LOADING_STATION.getPoses(), reversed=False
    #         )
    #         self.trajectory.build()
    #         self.trajectory.writeCSV("logs/output.csv")
    #         self.trajectory.drawSimulation()
    #         self.timer.reset()
    #         self.timer.start()
    #     if not self.followTrajectory():
    #         self.next_state("loadingStationBackup")

    # @state
    # def loadingStationBackup(self, initial_call):
    #     if initial_call:
    #         self.trajectory = self.createTrajectory(
    #             Path.LOADING_STATION_BACKUP.getPoses(), reversed=True
    #         )
    #         self.trajectory.build()
    #         self.trajectory.writeCSV("logs/output.csv")
    #         self.trajectory.drawSimulation()
    #         self.timer.reset()
    #         self.timer.start()
    #     if not self.followTrajectory():
    #         self.next_state("loadingStationToLeftRocket")

    # @state
    # def loadingStationToLeftRocket(self, initial_call):
    #     if initial_call:
    #         self.trajectory = self.createTrajectory(
    #             Path.LOADING_STATION_2_LEFT_ROCKET.getPoses(), reversed=False
    #         )
    #         self.trajectory.build()
    #         self.trajectory.writeCSV("logs/output.csv")
    #         self.trajectory.drawSimulation()
    #         self.timer.reset()
    #         self.timer.start()
    #     if not self.followTrajectory():
    #         self.next_state("stop")

    @state
    def stop(self):
        self.chassis.setWheelOutput(0, 0)
