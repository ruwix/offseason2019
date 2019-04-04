import numpy as np


class RealDCMotor:
    def __init__(
        self, nominal_voltage, stall_torque, stall_current, free_current, free_speed
    ):
        self.nominal_voltage = nominal_voltage
        self.stall_torque = stall_torque
        self.stall_current = stall_current
        self.free_current = free_current

        self.free_speed = free_speed / 60 * (2 * np.pi)

        self.resistance = self.nominal_voltage / self.stall_current

        self.kv = self.free_speed / (
            self.nominal_voltage - self.resistance * self.free_current
        )

        self.kt = self.stall_torque / self.stall_current

    @staticmethod
    def gearbox(motor, num_motors):
        return RealDCMotor(
            motor.nominal_voltage,
            motor.stall_torque * num_motors,
            motor.stall_current,
            motor.free_current,
            motor.free_speed / (2.0 * np.pi) * 60,
        )
