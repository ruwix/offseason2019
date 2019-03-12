from magicbot import AutonomousStateMachine, timed_state, state
import wpilib

# this is one of your components
from components.chassis import Chassis


class Autonomous(AutonomousStateMachine):

    MODE_NAME = "Drive Forward"
    DEFAULT = True

    # Injected from the definition in robot.py
    chassis: Chassis

    @timed_state(duration=1, first=True, next_state="stop")
    def drive_forward(self):
        self.chassis.setVelocity(120, 120)

    @state()
    def stop(self):
        self.chassis.setOutput(0, 0)

