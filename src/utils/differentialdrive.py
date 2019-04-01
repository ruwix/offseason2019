"""
Implementation from:
NASA Ames Robotics "The Cheesy Poofs"
Team 254
"""
from utils.dcmotortransmission import DCMotorTransmission
from utils.epsilon import EPSILON, epsilonEquals

import numpy as np


class WheelState:
    def __init__(self, left: float, right: float):
        self.left = left
        self.right = right


class DriveDynamics:
    def __init__(
        self,
        curvature: float,
        dcurvature: float,
        chassis_velocity: float,
        chassis_acceleration: float,
        wheel_velocity: float,
        wheel_acceleration: float,
        voltage: float,
        wheel_torque: float,
    ):
        self.curvature = curvature
        self.dcurvature = dcurvature
        self.chassis_velocity = chassis_velocity
        self.chassis_acceleration = chassis_acceleration
        self.wheel_velocity = wheel_velocity
        self.wheel_acceleration = wheel_acceleration
        self.voltage = voltage
        self.wheel_torque = wheel_torque


class ChassisState:
    def __init__(self, linear: float, angular: float):
        self.linear = linear
        self.angular = angular


class MinMax:
    def __init__(self, min, max):
        self.min = min
        self.max = max


class DifferentialDrive:
    def __init__(
        self,
        mass: float,
        moi: float,
        angular_drag: float,
        wheel_radius: float,
        effective_wheel_base_radius: float,
        left_transmission: DCMotorTransmission,
        right_transmission: DCMotorTransmission,
    ):
        self.mass = mass
        self.moi = moi
        self.angular_drag = angular_drag
        self.wheel_radius = wheel_radius
        self.effective_wheel_base_radius = effective_wheel_base_radius
        self.left_transmission = left_transmission
        self.right_transmission = right_transmission

    def solveForwardKinematics(self, wheel_motion: WheelState) -> ChassisState:
        linear = self.wheel_radius * (wheel_motion.right + wheel_motion.left) / 2.0
        angular = (
            self.wheel_radius
            * (wheel_motion.right - wheel_motion.left)
            / (2.0 * self.effective_wheel_base_radius)
        )
        return ChassisState(linear, angular)

    def solveInverseKinematics(self, chassis_motion: ChassisState) -> WheelState:
        left = (
            chassis_motion.linear
            - self.effective_wheel_base_radius * chassis_motion.angular
        ) / self.wheel_radius
        right = (
            chassis_motion.linear
            + self.effective_wheel_base_radius * chassis_motion.angular
        ) / self.wheel_radius
        return WheelState(left, right)

    def solveForwardDynamics(
        self, wheel_velocity: WheelState, voltage: WheelState
    ) -> DriveDynamics:
        chassis_velocity = self.solveForwardKinematics(wheel_velocity)
        return self.solveForwardDynamicsFull(
            wheel_velocity,
            chassis_velocity,
            (chassis_velocity.angular / chassis_velocity.linear),
        )

    def solveForwardDynamicsFull(
        self, wheel_velocity, chassis_velocity, curvature, voltage
    ) -> DriveDynamics:
        left_stationary = (
            epsilonEquals(wheel_velocity.left, 0)
            and abs(voltage.left) < self.left_transmission.friction_voltage
        )
        right_stationary = (
            epsilonEquals(wheel_velocity.right, 0)
            and abs(voltage.right) < self.right_transmission.friction_voltage
        )
        if left_stationary and right_stationary:
            wheel_torque = WheelState(0, 0)
            chassis_acceleration = ChassisState(0, 0)
            wheel_acceleration = WheelState(0, 0)
            dcurvature = 0
        else:
            wheel_torque = WheelState(
                self.left_transmission.getTorqueFromVoltage(
                    wheel_velocity.left, voltage.left
                ),
                self.right_transmission.getTorqueFromVoltage(
                    wheel_velocity.right, voltage.right
                ),
            )
            chassis_acceleration = ChassisState(
                (wheel_torque.right + wheel_torque.left)
                / (self.wheel_radius * self.mass),
                self.effective_wheel_base_radius
                * (wheel_torque.right - wheel_torque.left)
                / (self.wheel_radius * self.moi)
                - chassis_velocity.angular * self.angular_drag / self.moi,
            )
            dcurvature = (
                chassis_acceleration.angular - chassis_acceleration.linear * curvature
            ) / (chassis_velocity.linear * chassis_velocity.linear)

            wheel_acceleration = WheelState(
                chassis_acceleration.linear
                - chassis_acceleration.angular * self.effective_wheel_base_radius,
                chassis_acceleration.linear
                + chassis_acceleration.angular * self.effective_wheel_base_radius,
            )
        return DriveDynamics(
            curvature,
            dcurvature,
            chassis_velocity,
            chassis_acceleration,
            wheel_velocity,
            wheel_acceleration,
            voltage,
            wheel_torque,
        )

    def solveInverseDynamics(
        self, chassis_velocity: ChassisState, chassis_acceleration: ChassisState
    ) -> DriveDynamics:
        curvature = chassis_velocity.angular / chassis_velocity.linear
        dcurvature = (
            chassis_acceleration.angular - chassis_acceleration.linear * curvature
        ) / (chassis_velocity.linear ** 2)
        return solveInverseDynamicsFull(
            solveInverseKinematics(chassis_velocity),
            chassis_velocity,
            solveInverseKinematics(chassis_acceleration),
            chassis_acceleration,
            curvature,
            dcurvature,
        )

    def solveInverseDynamicsFull(
        self,
        wheel_velocity: WheelState,
        chassis_velocity: ChassisState,
        wheel_acceleration: WheelState,
        chassis_acceleration: ChassisState,
        curvature: float,
        dcurvature: float,
    ) -> DriveDynamics:
        wheel_torque = WheelState(
            self.wheel_radius
            / 2.0
            * (
                chassis_acceleration.linear * self.mass
                - chassis_acceleration.angular
                * self.moi
                / self.effective_wheel_base_radius
                - chassis_velocity.angular
                * self.angular_drag
                / self.effective_wheel_base_radius
            ),
            self.wheel_radius
            / 2.0
            * (
                chassis_acceleration.linear * self.mass
                + chassis_acceleration.angular
                * self.moi
                / self.effective_wheel_base_radius
                + chassis_velocity.angular
                * self.angular_drag
                / self.effective_wheel_base_radius
            ),
        )
        voltage = WheelState(
            self.left_transmission.getVoltageFromTorque(
                wheel_velocity.left, wheel_torque.left
            ),
            self.right_transmission.getVoltageFromTorque(
                wheel_velocity.right, wheel_torque.right
            ),
        )
        return DriveDynamics(
            curvature,
            dcurvature,
            chassis_velocity,
            chassis_acceleration,
            wheel_velocity,
            wheel_acceleration,
            voltage,
            wheel_torque,
        )

    def getMaxAbsVelocity(self, curvature: float, max_abs_voltage: float) -> float:
        left_speed_at_max_moltage = self.left_transmission.getFreeSpeedAtVoltage(
            max_abs_voltage
        )
        right_speed_at_max_moltage = self.right_transmission.getFreeSpeedAtVoltage(
            max_abs_voltage
        )

        if epsilonEquals(curvature, 0):
            return self.wheel_radius * np.min(
                left_speed_at_max_moltage, right_speed_at_max_moltage
            )

        if curvature == float("inf"):
            # Turn in place.  Return value meaning becomes angular velocity.
            wheel_speed = np.min(left_speed_at_max_moltage, right_speed_at_max_moltage)
            return (
                np.sign(curvature)
                * self.wheel_radius
                * wheel_speed
                / self.effective_wheel_base_radius
            )

        right_speed_if_left_max = (
            left_speed_at_max_moltage
            * (self.effective_wheel_base_radius * curvature + 1.0)
            / (1.0 - self.effective_wheel_base_radius * curvature)
        )

        if np.abs(right_speed_if_left_max) <= right_speed_at_max_moltage + EPSILON:
            # Left max is active constraint.
            return (
                self.wheel_radius
                * (left_speed_at_max_moltage + right_speed_if_left_max)
                / 2.0
            )
        left_speed_if_left_max = (
            right_speed_at_max_moltage
            * (1.0 - self.effective_wheel_base_radius * curvature)
            / (1.0 + self.effective_wheel_base_radius * curvature)
        )

        # Right at max is active constraint.
        return (
            self.wheel_radius
            * (right_speed_at_max_moltage + left_speed_if_left_max)
            / 2.0
        )

    def getMinMaxAcceleration(
        self, chassis_velocity: ChassisState, curvature: float, max_abs_voltage: float
    ):
        wheel_velocities = solveInverseKinematics()
        min = np.Inf
        max = np.NINF
        # Math:
        # (Tl + Tr) / r_w = m*a
        # (Tr - Tl) / r_w * r_wb - drag*w = i*(a * k + v^2 * dk)

        # 2 equations, 2 unknowns.
        # Solve for a and (Tl|Tr)
        if curvature == np.Inf or curvature == np.NINF:
            linear_term = 0.0
            angular_term = self.moi
        else:
            linear_term = mass * self.effective_wheel_base_radius
            angular_term = self.moi * curvature
        drag_torque = chassis_velocity.angular * angular_drag
        for left in (False, True):
            for sign in (1, -1):
                fixed_wheel_velocity = (
                    wheel_velocities.left if left else wheel_velocities.right
                )
                variable_wheel_velocity = (
                    wheel_velocities.right if left else wheel_velocities.left
                )

                fixed_transmission = (
                    self.left_transmission if left else self.right_transmission
                )
                variable_transmission = (
                    self.right_transmission if left else self.left_transmission
                )
                fixed_torque = fixedTransmission.getTorqueForVoltage(
                    fixed_wheel_velocity, sign * max_abs_voltage
                )
                if left:
                    variable_torque = (
                        -drag_torque * self.mass * self.wheel_radius
                        + fixed_torque * (linear_term + angular_term)
                    ) / (linear_term - angular_term)
                else:
                    variable_torque = (
                        drag_torque * self.mass * self.wheel_radius
                        + fixed_torque * (linear_term - angular_term)
                    ) / (linear_term + angular_term)
                variable_voltage = variable_transmission.getVoltageFromTorque(
                    variable_wheel_velocity, variable_torque
                )
                if np.abs(variable_voltage) <= max_abs_voltage + EPSILON:
                    if curvature == np.Inf or curvature == np.NINF:
                        sign = -1 if left else 1
                        accel = (
                            sign
                            * (fixed_torque - variable_torque)
                            * self.effective_wheel_base_radius
                            / (self.moi * self.wheel_radius)
                            - drag_torque / self.moi
                        )
                    else:
                        accel = (fixed_torque + variable_torque) / (
                            self.mass / self.moi
                        )
                    min = np.min(min, accel)
                    max = np.max(max, accel)
        return MinMax(min, max)

