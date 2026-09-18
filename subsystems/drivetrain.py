"""DriveTrain subsystem -- 6-wheel drop-center differential (tank) drive.

Teaching-bot proof of concept. Ported conceptually from the competition
bot's DriveTrain.py, simplified: no vision, no PathPlanner, no pose
estimator/odometry (left as a natural next lesson -- see README). Just
enough to drive in teleop and support two simple autonomous routines
(drive-to-distance from encoders, turn-to-heading from the navX gyro).
"""
from __future__ import annotations

from typing import Callable

import navx
import rev
import wpilib
import wpilib.drive
import wpimath
from commands2 import Command, Subsystem, cmd
from wpimath.controller import PIDController

from constants import Auto, DriveTrainConstants


class DriveTrain(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._left_lead = rev.SparkMax(DriveTrainConstants.LEFT_LEAD_CAN_ID, rev.SparkMax.MotorType.kBrushless)
        self._left_follow = rev.SparkMax(DriveTrainConstants.LEFT_FOLLOW_CAN_ID, rev.SparkMax.MotorType.kBrushless)
        self._right_lead = rev.SparkMax(DriveTrainConstants.RIGHT_LEAD_CAN_ID, rev.SparkMax.MotorType.kBrushless)
        self._right_follow = rev.SparkMax(DriveTrainConstants.RIGHT_FOLLOW_CAN_ID, rev.SparkMax.MotorType.kBrushless)

        self._left_encoder = self._left_lead.getEncoder()
        self._right_encoder = self._right_lead.getEncoder()

        self._gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        self._configure_motors()

        self._driver = wpilib.drive.DifferentialDrive(self._left_lead, self._right_lead)

        self._telemetry_loop_counter = 0

    def _configure_motors(self) -> None:
        conversion_factor = DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS / DriveTrainConstants.GEAR_RATIO

        right_lead_config = rev.SparkMaxConfig()
        right_lead_config.inverted(True)
        right_lead_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        right_lead_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        right_lead_config.encoder.positionConversionFactor(conversion_factor)
        right_lead_config.encoder.velocityConversionFactor(conversion_factor / 60.0)
        self._right_lead.configure(
            right_lead_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )

        left_lead_config = rev.SparkMaxConfig()
        left_lead_config.inverted(False)
        left_lead_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        left_lead_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        left_lead_config.encoder.positionConversionFactor(conversion_factor)
        left_lead_config.encoder.velocityConversionFactor(conversion_factor / 60.0)
        self._left_lead.configure(
            left_lead_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )

        left_follow_config = rev.SparkMaxConfig()
        left_follow_config.follow(self._left_lead)
        left_follow_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        left_follow_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        self._left_follow.configure(
            left_follow_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )

        right_follow_config = rev.SparkMaxConfig()
        right_follow_config.follow(self._right_lead)
        right_follow_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        right_follow_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        self._right_follow.configure(
            right_follow_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )

        self._left_encoder.setPosition(0)
        self._right_encoder.setPosition(0)

    # ---- Teleop drive ----

    def teleop_drive_command(self, left_y: Callable[[], float], right_y: Callable[[], float]) -> Command:
        def _run():
            self.drive(
                DriveTrainConstants.SPEED_SCALE * left_y(),
                DriveTrainConstants.SPEED_SCALE * right_y(),
            )

        return cmd.run(_run, self)

    def drive(self, left: float, right: float) -> None:
        self._driver.tankDrive(
            wpimath.applyDeadband(left, DriveTrainConstants.JOYSTICK_DEADBAND),
            wpimath.applyDeadband(right, DriveTrainConstants.JOYSTICK_DEADBAND),
        )

    def stop(self) -> None:
        self._driver.stopMotor()

    # ---- Sensors ----

    def get_left_distance_meters(self) -> float:
        return self._left_encoder.getPosition()

    def get_right_distance_meters(self) -> float:
        return self._right_encoder.getPosition()

    def get_average_distance_meters(self) -> float:
        return (self.get_left_distance_meters() + self.get_right_distance_meters()) / 2.0

    def reset_encoders(self) -> None:
        self._left_encoder.setPosition(0)
        self._right_encoder.setPosition(0)

    def get_heading_degrees(self) -> float:
        # Inverted for CCW-positive, matching WPILib's convention (same as
        # the competition bot's DriveTrain.get_heading()).
        return -self._gyro.getAngle()

    def reset_gyro(self) -> None:
        self._gyro.reset()

    # ---- Autonomous commands ----

    def drive_distance_command(self, distance_meters: float, speed: float = Auto.DRIVE_SPEED) -> Command:
        """Drives straight until the average of the two encoders reaches
        distance_meters, then stops. Simple encoder-based teaching example --
        no PID, just "keep going until far enough" (a constant-speed bang-bang
        style controller)."""

        def _finished() -> bool:
            return self.get_average_distance_meters() >= distance_meters

        return cmd.sequence(
            cmd.runOnce(self.reset_encoders, self),
            cmd.run(lambda: self.drive(speed, speed), self).until(_finished),
        ).finallyDo(lambda interrupted: self.stop())

    def turn_to_angle_command(self, target_degrees: float) -> Command:
        """PID-turns to an absolute heading (degrees, CCW-positive, 0 = the
        heading the gyro was last reset to). Positive = turn left."""
        pid = PIDController(DriveTrainConstants.TURN_KP, DriveTrainConstants.TURN_KI, DriveTrainConstants.TURN_KD)
        pid.enableContinuousInput(-180, 180)
        pid.setTolerance(DriveTrainConstants.TURN_TOLERANCE_DEGREES)
        pid.setSetpoint(target_degrees)

        def _run() -> None:
            output = pid.calculate(self.get_heading_degrees())
            self.drive(-output, output)

        return cmd.run(_run, self).until(pid.atSetpoint).finallyDo(lambda interrupted: self.stop())

    def periodic(self) -> None:
        self._telemetry_loop_counter += 1
        if self._telemetry_loop_counter >= DriveTrainConstants.TELEMETRY_PERIOD_LOOPS:
            self._telemetry_loop_counter = 0
            wpilib.SmartDashboard.putNumber("DriveTrain/LeftDistMeters", self.get_left_distance_meters())
            wpilib.SmartDashboard.putNumber("DriveTrain/RightDistMeters", self.get_right_distance_meters())
            wpilib.SmartDashboard.putNumber("DriveTrain/HeadingDeg", self.get_heading_degrees())
