"""
Implementation from:
NASA Ames Robotics "The Cheesy Poofs"
Team 254
"""
import numpy as np

from utils.mathextension import EPSILON


class DCMotorTransmission:
    """
    Model of a DC motor rotating a shaft. All parameters refer to the output (e.g. should already consider gearing
    and efficiency losses). The motor is assumed to be symmetric forward/reverse.
    """

    def __init__(self, speed_per_volt, torque_per_volt, friction_voltage):
        self.speed_per_volt = speed_per_volt  # rad/s per V (no load)
        self.torque_per_volt = torque_per_volt  # N m per V (stall)
        self.friction_voltage = friction_voltage  # V

    def getFreeSpeedAtVoltage(self, voltage: float) -> float:
        """Returns the free speed of the motor at the specified voltage."""
        if voltage > EPSILON:
            return np.max((0, voltage - self.friction_voltage)) * self.speed_per_volt
        elif voltage < -EPSILON:
            return np.min((0, voltage - self.friction_voltage)) * self.speed_per_volt
        else:
            return 0

    def getTorqueFromVoltage(self, output_speed: float, voltage: float) -> float:
        """Returns the torque produced by the motor."""
        effective_voltage = voltage
        if output_speed > EPSILON:  # Forward motion, rolling friction.
            effective_voltage -= self.friction_voltage
        elif output_speed < -EPSILON:  # Reverse motion, rolling friction.
            effective_voltage += self.friction_voltage
        elif voltage > EPSILON:  # System is static, forward torque.
            effective_voltage = np.max((0, voltage - self.friction_voltage))
        elif voltage < -EPSILON:  # System is static, reverse torque.
            effective_voltage = np.min((0, voltage + self.friction_voltage))
        else:
            return 0
        return self.torque_per_volt * (
            -output_speed / self.speed_per_volt + effective_voltage
        )

    def getVoltageFromTorque(self, output_speed: float, torque: float) -> float:
        """Returns the voltage going through the motor."""
        if output_speed > EPSILON:  # Forward motion, rolling friction.
            fv = self.friction_voltage
        elif output_speed < -EPSILON:  # Reverse motion, rolling friction.
            fv = -self.friction_voltage
        elif torque > EPSILON:  # System is static, forward torque.
            fv = self.friction_voltage
        elif torque < -EPSILON:  # System is static, reverse torque.
            fv = -self.friction_voltage
        else:
            return 0
        return torque / self.torque_per_volt + output_speed / self.speed_per_volt + fv
