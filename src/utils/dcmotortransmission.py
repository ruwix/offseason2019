"""
Implementation from:
NASA Ames Robotics "The Cheesy Poofs"
Team 254
"""
from numpy import np
from utils.epsilon import EPSILON


class DCMotorTransmission:
    def __init__(self, speed_per_volt, torque_per_volt, friction_voltage):
        self.speed_per_volt = speed_per_volt
        self.torque_per_volt = torque_per_volt
        self.friction_voltage = friction_voltage

    def getFreeSpeedAtVoltage(self, voltage):
        if voltage > EPSILON:
            return np.max(0, voltage - self.friction_voltage) * self.speed_per_volt
        elif voltage < -EPSILON:
            return np.min(0, voltage - self.friction_voltage) * self.speed_per_volt
        else:
            return 0

    def getTorqueFromVoltage(self, output_speed, voltage):
        effective_voltage = voltage
        if output_speed > EPSILON:
            effective_voltage -= self.friction_voltage
        elif output_speed < -EPSILON:
            effective_voltage += self.friction_voltage
        elif voltage > EPSILON:
            effective_voltage = np.max(0, voltage - self.friction_voltage)
        elif voltage < -EPSILON:
            effective_voltage = np.min(0, voltage + self.friction_voltage)
        else:
            return 0
        return self.torque_per_volt * (
            -output_speed / self.speed_per_volt + effective_voltage
        )

    def getVoltageFromTorque(self, output_speed, torque):
        if output_speed > EPSILON:
            fv = self.friction_voltage
        elif output_speed < -EPSILON:
            fv = -self.friction_voltage
        elif torque > EPSILON:
            fv = self.friction_voltage
        elif torque < -EPSILON:
            fv = -self.friction_voltage
        else:
            return 0
        return torque / self.torque_per_volt + output_speed / self.speed_per_volt + fv
