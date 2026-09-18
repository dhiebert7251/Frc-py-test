"""Simulation physics model for `python -m robotpy sim`.

Only the drivetrain is modeled -- same scope decision as the competition
bot's physics.py. Shooter, Trigger, Elevator, and Gripper dynamics aren't
simulated; their telemetry in sim reflects commanded setpoints, not a
physical response.

The REV NEO 2.0 has no built-in DCMotor factory in wpimath as of the 2026
season, so it is hand-built below from REV's published spec sheet
(https://www.revrobotics.com/rev-21-1653/): free speed 5676 RPM (same as the
original NEO), stall torque 3.75 N*m, stall current 150 A, free current
1.8 A. The raw DCMotor(nominalVoltage, stallTorque, stallCurrent,
freeCurrent, freeSpeed, numMotors) constructor was verified directly against
the installed 2026 wpimath package before use here.
"""
from __future__ import annotations

import math

import rev
import wpilib.simulation
from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveKinematics, DifferentialDriveWheelSpeeds
from wpimath.system.plant import DCMotor

from constants import DriveTrainConstants

NEO_2_MOTOR = DCMotor(
    12.0,  # nominal voltage
    3.75,  # stall torque, N*m
    150.0,  # stall current, A
    1.8,  # free current, A
    DCMotor.NEO(1).freeSpeed,  # free speed, rad/s -- same as the original NEO
    1,  # number of motors modeled
)


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot) -> None:
        self.physics_controller = physics_controller

        drivetrain = robot.robot_container.drivetrain
        self._left_sim = rev.SparkMaxSim(drivetrain._left_lead, NEO_2_MOTOR)
        self._right_sim = rev.SparkMaxSim(drivetrain._right_lead, NEO_2_MOTOR)

        self._kinematics = DifferentialDriveKinematics(DriveTrainConstants.TRACK_WIDTH_METERS)

        self._max_wheel_speed_mps = (
            NEO_2_MOTOR.freeSpeed
            / DriveTrainConstants.GEAR_RATIO
            * (DriveTrainConstants.WHEEL_DIAMETER_METERS / 2.0)
        )

        navx_sim = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        navx_sim.getBoolean("Connected").set(True)
        self._navx_yaw_sim = navx_sim.getDouble("Yaw")
        self._yaw_degrees = 0.0

    def update_sim(self, now: float, tm_diff: float) -> None:
        # DriveTrain._configure_motors() inverts the right lead motor so positive
        # output drives forward on both sides; undo that here so the kinematics
        # below sees true left/right wheel speeds.
        left_speed = self._left_sim.getAppliedOutput() * self._max_wheel_speed_mps
        right_speed = -self._right_sim.getAppliedOutput() * self._max_wheel_speed_mps

        chassis_speeds: ChassisSpeeds = self._kinematics.toChassisSpeeds(
            DifferentialDriveWheelSpeeds(left_speed, right_speed)
        )

        self.physics_controller.drive(chassis_speeds, tm_diff)

        # DriveTrain.get_heading_degrees() negates the raw navX angle, so drive
        # the sim yaw with the negated chassis rotation to match.
        self._yaw_degrees -= math.degrees(chassis_speeds.omega) * tm_diff
        self._navx_yaw_sim.set(self._yaw_degrees)

        self._left_sim.iterate(left_speed, 12.0, tm_diff)
        self._right_sim.iterate(-right_speed, 12.0, tm_diff)
