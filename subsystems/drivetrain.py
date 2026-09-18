"""DriveTrain subsystem -- differential (tank) drive with vision-fused odometry
and PathPlanner autonomous.

Ported from DriveTrain.java.

Notes on API differences from the Java version:
  * com.studica.frc.AHRS -> navx.AHRS (robotpy-navx). Same NavXComType enum.
  * AutoBuilder.configure()'s `output` callback takes
    (ChassisSpeeds, DriveFeedforwards) in the 2026 Python bindings, vs. a
    single ChassisSpeeds argument in Java. drive_robot_relative() accepts an
    optional, currently-unused feedforwards argument to satisfy that.
  * PathPlannerAuto has no public getStartingPose() in pathplannerlib-python
    2026.1.2 (it is computed internally into a private attribute). It is
    reconstructed here from PathPlannerAuto.getPathGroupFromAutoFile(name)
    + PathPlannerPath.getStartingDifferentialPose(), which is what that
    private attribute is built from for a non-holonomic (differential) drive.
"""

from __future__ import annotations

from typing import Callable, Optional

import navx
import rev
import wpilib
import wpilib.drive
import wpimath
from commands2 import Command, Subsystem, cmd
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPLTVController
from pathplannerlib.logging import PathPlannerLogging
from pathplannerlib.util import DriveFeedforwards
from wpimath.controller import PIDController
from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    DifferentialDriveKinematics,
    DifferentialDriveWheelSpeeds,
)

from constants import Auto, DriveTrainConstants
from vision_measurement import VisionMeasurement

PPLTV_PREFIX = "DriveTrain/PPLTV/"
FIELD_LENGTH_METERS = 16.541


class DriveTrain(Subsystem):
    def __init__(self, vision_subsystem) -> None:
        super().__init__()

        self._vision_subsystem = vision_subsystem

        # ---- Hardware ----
        self._left_motor_lead = rev.SparkMax(
            DriveTrainConstants.LEFT_LEAD_CAN_ID, rev.SparkMax.MotorType.kBrushless
        )
        self._right_motor_lead = rev.SparkMax(
            DriveTrainConstants.RIGHT_LEAD_CAN_ID, rev.SparkMax.MotorType.kBrushless
        )
        self._left_motor_follow = rev.SparkMax(
            DriveTrainConstants.LEFT_FOLLOW_CAN_ID, rev.SparkMax.MotorType.kBrushless
        )
        self._right_motor_follow = rev.SparkMax(
            DriveTrainConstants.RIGHT_FOLLOW_CAN_ID, rev.SparkMax.MotorType.kBrushless
        )

        self._left_encoder = self._left_motor_lead.getEncoder()
        self._right_encoder = self._right_motor_lead.getEncoder()

        self._gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        self._driver = wpilib.drive.DifferentialDrive(self._left_motor_lead, self._right_motor_lead)

        # ---- Kinematics ----
        self._kinematics = DifferentialDriveKinematics(DriveTrainConstants.TRACK_WIDTH_METERS)

        self._configure_motors()

        # Pose estimator -- fuses encoder + gyro odometry with vision measurements.
        self._pose_estimator = DifferentialDrivePoseEstimator(
            self._kinematics,
            self.get_heading(),
            self.get_left_distance_meters(),
            self.get_right_distance_meters(),
            Pose2d(),
        )

        self._field = wpilib.Field2d()

        # ---- PathPlanner ----
        self._robot_config: Optional[RobotConfig] = None
        self._ltv_controller: Optional[PPLTVController] = None
        self._last_qx = Auto.PPLTV_Q_X
        self._last_qy = Auto.PPLTV_Q_Y
        self._last_qtheta = Auto.PPLTV_Q_THETA
        self._last_rvel = Auto.PPLTV_R_VEL
        self._last_romega = Auto.PPLTV_R_OMEGA
        self._last_dt = Auto.PPLTV_DT
        self._last_max_velocity = Auto.PPLTV_MAX_VELOCITY

        # Reverse driving -- swaps front/back so the robot's rear acts as the front.
        self._reverse_driving = False

        self._telemetry_loop_counter = 0

        self._init_ltv_tuning()
        self._configure_pathplanner()
        wpilib.SmartDashboard.putData("Field", self._field)  # register once, not every loop

    def periodic(self) -> None:
        self._pose_estimator.update(
            self.get_heading(), self.get_left_distance_meters(), self.get_right_distance_meters()
        )

        if self._vision_subsystem is not None and self._vision_subsystem.is_any_vision_available():
            self._update_vision_measurements()

        self._refresh_ltv_controller_from_dashboard()
        self._telemetry_loop_counter += 1
        if self._telemetry_loop_counter >= DriveTrainConstants.TELEMETRY_PERIOD_LOOPS:
            self._telemetry_loop_counter = 0
            self._update_telemetry()

    # ---- Motor Configuration ----

    def _configure_motors(self) -> None:
        # Right lead -- inverted so positive output = forward on both sides.
        # Encoder conversion factors are set on the primary (built-in hall) encoder so that
        # getPosition() returns meters and getVelocity() returns m/s directly.
        right_lead_config = rev.SparkMaxConfig()
        right_lead_config.inverted(True)
        right_lead_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        right_lead_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        right_lead_config.openLoopRampRate(0.15)
        right_lead_config.encoder.positionConversionFactor(
            DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS / DriveTrainConstants.GEAR_RATIO
        )
        right_lead_config.encoder.velocityConversionFactor(
            DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS / DriveTrainConstants.GEAR_RATIO / 60.0
        )
        self._right_motor_lead.configure(
            right_lead_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )

        # Left lead
        left_lead_config = rev.SparkMaxConfig()
        left_lead_config.inverted(False)
        left_lead_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        left_lead_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        left_lead_config.openLoopRampRate(0.15)
        left_lead_config.encoder.positionConversionFactor(
            DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS / DriveTrainConstants.GEAR_RATIO
        )
        left_lead_config.encoder.velocityConversionFactor(
            DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS / DriveTrainConstants.GEAR_RATIO / 60.0
        )
        self._left_motor_lead.configure(
            left_lead_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )

        # Follow motors mirror their respective leads. Status frame periods are slowed
        # significantly since no data is ever read from followers, reducing unnecessary
        # CAN bus traffic.
        left_follow_config = rev.SparkMaxConfig()
        left_follow_config.follow(self._left_motor_lead)
        left_follow_config.inverted(False)
        left_follow_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        left_follow_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        left_follow_config.openLoopRampRate(0.15)
        left_follow_config.signals.appliedOutputPeriodMs(500)
        left_follow_config.signals.primaryEncoderPositionPeriodMs(500)
        left_follow_config.signals.primaryEncoderVelocityPeriodMs(500)
        left_follow_config.signals.outputCurrentPeriodMs(500)
        self._left_motor_follow.configure(
            left_follow_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )

        right_follow_config = rev.SparkMaxConfig()
        right_follow_config.follow(self._right_motor_lead)
        right_follow_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        right_follow_config.inverted(True)
        right_follow_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        right_follow_config.openLoopRampRate(0.15)
        right_follow_config.signals.appliedOutputPeriodMs(500)
        right_follow_config.signals.primaryEncoderPositionPeriodMs(500)
        right_follow_config.signals.primaryEncoderVelocityPeriodMs(500)
        right_follow_config.signals.outputCurrentPeriodMs(500)
        self._right_motor_follow.configure(
            right_follow_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )

        # Reset encoder positions to zero
        self._left_encoder.setPosition(0)
        self._right_encoder.setPosition(0)

    # ---- PathPlanner Configuration ----

    def _configure_pathplanner(self) -> None:
        PathPlannerLogging.setLogActivePathCallback(
            lambda poses: self._field.getObject("path").setPoses(poses)
        )

        try:
            self._robot_config = RobotConfig.fromGUISettings()
            AutoBuilder.configure(
                self.get_pose,
                self.reset_pose,
                self.get_robot_relative_speeds,
                self.drive_robot_relative,
                self._ltv_controller,
                self._robot_config,
                self._is_red_alliance,
                self,
            )
        except Exception as e:  # noqa: BLE001 -- mirrors the Java catch-all around AutoBuilder setup
            wpilib.reportError(f"Failed to configure PathPlanner: {e}")

    @staticmethod
    def _is_red_alliance() -> bool:
        alliance = wpilib.DriverStation.getAlliance()
        return alliance == wpilib.DriverStation.Alliance.kRed

    def _init_ltv_tuning(self) -> None:
        wpilib.SmartDashboard.putNumber(PPLTV_PREFIX + "Qx", Auto.PPLTV_Q_X)
        wpilib.SmartDashboard.putNumber(PPLTV_PREFIX + "Qy", Auto.PPLTV_Q_Y)
        wpilib.SmartDashboard.putNumber(PPLTV_PREFIX + "Qtheta", Auto.PPLTV_Q_THETA)
        wpilib.SmartDashboard.putNumber(PPLTV_PREFIX + "Rvel", Auto.PPLTV_R_VEL)
        wpilib.SmartDashboard.putNumber(PPLTV_PREFIX + "Romega", Auto.PPLTV_R_OMEGA)
        wpilib.SmartDashboard.putNumber(PPLTV_PREFIX + "Dt", Auto.PPLTV_DT)
        wpilib.SmartDashboard.putNumber(PPLTV_PREFIX + "MaxVelocity", Auto.PPLTV_MAX_VELOCITY)
        self._rebuild_ltv_controller_from_dashboard()

    def _refresh_ltv_controller_from_dashboard(self) -> None:
        if not wpilib.DriverStation.isTest():
            return

        qx = wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Qx", Auto.PPLTV_Q_X)
        qy = wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Qy", Auto.PPLTV_Q_Y)
        qtheta = wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Qtheta", Auto.PPLTV_Q_THETA)
        rvel = wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Rvel", Auto.PPLTV_R_VEL)
        romega = wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Romega", Auto.PPLTV_R_OMEGA)
        dt = wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Dt", Auto.PPLTV_DT)
        max_vel = wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "MaxVelocity", Auto.PPLTV_MAX_VELOCITY)

        if (
            qx != self._last_qx
            or qy != self._last_qy
            or qtheta != self._last_qtheta
            or rvel != self._last_rvel
            or romega != self._last_romega
            or dt != self._last_dt
            or max_vel != self._last_max_velocity
        ):
            # Only rebuild the controller object. AutoBuilder.configure() was called
            # once in __init__ and must not be called again -- repeated calls register
            # duplicate command factories. PathPlanner reads self._ltv_controller by
            # reference, so reassigning it here is sufficient for new gains to take
            # effect on the next auto run.
            self._rebuild_ltv_controller(qx, qy, qtheta, rvel, romega, dt, max_vel)

    def _rebuild_ltv_controller_from_dashboard(self) -> None:
        self._rebuild_ltv_controller(
            wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Qx", Auto.PPLTV_Q_X),
            wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Qy", Auto.PPLTV_Q_Y),
            wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Qtheta", Auto.PPLTV_Q_THETA),
            wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Rvel", Auto.PPLTV_R_VEL),
            wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Romega", Auto.PPLTV_R_OMEGA),
            wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "Dt", Auto.PPLTV_DT),
            wpilib.SmartDashboard.getNumber(PPLTV_PREFIX + "MaxVelocity", Auto.PPLTV_MAX_VELOCITY),
        )

    def _rebuild_ltv_controller(
        self, qx: float, qy: float, qtheta: float, rvel: float, romega: float, dt: float, max_velocity: float
    ) -> None:
        if dt <= 0 or max_velocity <= 0:
            wpilib.reportError("Invalid PPLTV tuning values (dt/maxVelocity)")
            return
        self._ltv_controller = PPLTVController((qx, qy, qtheta), (rvel, romega), dt, max_velocity)
        self._last_qx = qx
        self._last_qy = qy
        self._last_qtheta = qtheta
        self._last_rvel = rvel
        self._last_romega = romega
        self._last_dt = dt
        self._last_max_velocity = max_velocity

    # ---- Pose Initialization ----

    def initialize_pose(self, auto_command: Optional[Command]) -> None:
        """Seeds the pose estimator with the best available pose.
        Priority: (1) vision AprilTag fix, (2) PathPlanner auto starting pose, (3) field origin.

        Call this at auto init (pass the selected auto command) and at teleop init (pass None).
        """
        initial_pose: Optional[Pose2d] = None

        # Priority 1: vision -- accept measurements up to POSE_INIT_MAX_VISION_AGE_SECONDS old.
        if self._vision_subsystem is not None:
            vision_measurement = self._vision_subsystem.get_best_vision_measurement()
            if vision_measurement is not None:
                age_sec = wpilib.Timer.getFPGATimestamp() - vision_measurement.timestamp_seconds
                if age_sec <= DriveTrainConstants.POSE_INIT_MAX_VISION_AGE_SECONDS:
                    initial_pose = vision_measurement.estimated_pose

        # Priority 2: PathPlanner auto starting pose (carries position + heading).
        # Flip X and heading for red alliance -- the starting pose is always blue-side coords.
        if initial_pose is None and isinstance(auto_command, PathPlannerAuto):
            starting_pose = self._get_auto_starting_pose(auto_command)
            if starting_pose is not None:
                if self._is_red_alliance():
                    starting_pose = self._flip_for_red_alliance(starting_pose)
                initial_pose = starting_pose

        # Priority 3: field origin fallback -- warn the drive team loudly.
        if initial_pose is None:
            initial_pose = Pose2d()
            wpilib.reportWarning(
                "DriveTrain: pose could not be initialized from vision or auto starting pose. "
                "Defaulting to field origin -- heading will be incorrect if robot is not "
                "facing the field-forward direction."
            )
            wpilib.SmartDashboard.putBoolean("DriveTrain/HeadingInitialized", False)
        else:
            wpilib.SmartDashboard.putBoolean("DriveTrain/HeadingInitialized", True)

        self.reset_pose(initial_pose)

    @staticmethod
    def _get_auto_starting_pose(auto_command: PathPlannerAuto) -> Optional[Pose2d]:
        """pathplannerlib-python has no public PathPlannerAuto.getStartingPose(); it is
        computed internally into a private attribute. Reconstruct it the same way, from
        the auto's path group file (differential-drive starting pose)."""
        paths = PathPlannerAuto.getPathGroupFromAutoFile(auto_command.getName())
        if not paths:
            return None
        return paths[0].getStartingDifferentialPose()

    @staticmethod
    def _flip_for_red_alliance(pose: Pose2d) -> Pose2d:
        """Mirrors a blue-alliance pose to its red-alliance equivalent.
        Flips X across the field center; Y is unchanged; heading is reflected
        across the vertical axis (180 deg - angle)."""
        return Pose2d(
            FIELD_LENGTH_METERS - pose.X(),
            pose.Y(),
            Rotation2d.fromDegrees(180.0) - pose.rotation(),
        )

    # ---- Teleop Drive ----

    def teleop_drive_command(
        self,
        left_y_supplier: Callable[[], float],
        right_y_supplier: Callable[[], float],
        boost_supplier: Callable[[], float],
    ) -> Command:
        """Teleop drive command. leftY = left wheel speed, rightY = right wheel speed.
        boost (0.0-1.0) interpolates the speed scale from SPEED_SCALE to 1.0.
        When reverse_driving is active, left axis drives right wheels (negated) and
        right axis drives left wheels (negated), so the robot's rear acts as the front."""

        def _run() -> None:
            scale = DriveTrainConstants.SPEED_SCALE + (1.0 - DriveTrainConstants.SPEED_SCALE) * boost_supplier()
            left_y = left_y_supplier()
            right_y = right_y_supplier()
            if self._reverse_driving:
                self.drive(scale * -right_y, scale * -left_y)
            else:
                self.drive(scale * left_y, scale * right_y)

        return cmd.run(_run, self)

    def toggle_reverse_driving(self) -> None:
        """Toggles reverse driving mode. Intended for the driver Start button."""
        self._reverse_driving = not self._reverse_driving

    def drive(self, left_y: float, right_y: float) -> None:
        """Applies deadband and drives in robot-relative tank mode."""
        self._driver.tankDrive(
            wpimath.applyDeadband(left_y, DriveTrainConstants.JOYSTICK_DEADBAND),
            wpimath.applyDeadband(right_y, DriveTrainConstants.JOYSTICK_DEADBAND),
        )

    def stop(self) -> None:
        self._driver.stopMotor()

    # ---- PathPlanner Driving ----

    def drive_robot_relative(
        self, robot_relative_speeds: ChassisSpeeds, feedforwards: Optional[DriveFeedforwards] = None
    ) -> None:
        """Called by PathPlanner every loop tick during autonomous. Converts desired
        ChassisSpeeds to left/right wheel percent outputs.

        feedforwards is accepted (and ignored) because AutoBuilder.configure()'s output
        callback is invoked with (ChassisSpeeds, DriveFeedforwards) in this binding --
        the Java version only ever used the ChassisSpeeds argument."""
        target_speeds = ChassisSpeeds.discretize(robot_relative_speeds, 0.02)
        wheel_speeds = self._kinematics.toWheelSpeeds(target_speeds)
        wheel_speeds.desaturate(Auto.MAX_MODULE_SPEED)
        self._driver.tankDrive(
            wheel_speeds.left / Auto.MAX_MODULE_SPEED,
            wheel_speeds.right / Auto.MAX_MODULE_SPEED,
            False,
        )

    def drive_field_relative(self, field_relative_speeds: ChassisSpeeds) -> None:
        self.drive_robot_relative(
            ChassisSpeeds.fromFieldRelativeSpeeds(field_relative_speeds, self.get_pose().rotation())
        )

    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        return self._kinematics.toChassisSpeeds(self.get_wheel_speeds())

    def turn_to_angle(self, target_degrees: float) -> Command:
        pid = PIDController(0.04, 0.0, 0.005)
        pid.enableContinuousInput(-180, 180)
        pid.setTolerance(2.0)
        pid.setSetpoint(target_degrees)

        def _run() -> None:
            output = pid.calculate(self.get_heading().degrees())
            self._driver.tankDrive(-output, output, False)

        return (
            cmd.run(_run, self)
            .until(pid.atSetpoint)
            .finallyDo(lambda interrupted: self._driver.stopMotor())
        )

    # ---- Odometry and Pose ----

    def get_pose(self) -> Pose2d:
        return self._pose_estimator.getEstimatedPosition()

    def reset_pose(self, pose: Pose2d) -> None:
        self._pose_estimator.resetPosition(
            self.get_heading(), self.get_left_distance_meters(), self.get_right_distance_meters(), pose
        )

    def get_heading(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self._gyro.getAngle())  # Inverted for CCW positive

    def reset_gyro(self) -> None:
        self._gyro.reset()

    def get_left_distance_meters(self) -> float:
        """Left wheel distance in meters. Conversion factor is set in motor config."""
        return self._left_encoder.getPosition()

    def get_right_distance_meters(self) -> float:
        """Right wheel distance in meters. Conversion factor is set in motor config."""
        return self._right_encoder.getPosition()

    def get_wheel_speeds(self) -> DifferentialDriveWheelSpeeds:
        """Wheel speeds in m/s. Conversion factor is set in motor config."""
        return DifferentialDriveWheelSpeeds(self._left_encoder.getVelocity(), self._right_encoder.getVelocity())

    # ---- Vision ----

    def _update_vision_measurements(self) -> None:
        if self._vision_subsystem is None:
            return
        measurement: Optional[VisionMeasurement] = self._vision_subsystem.get_best_vision_measurement_if_fresh()
        if measurement is not None:
            self._pose_estimator.addVisionMeasurement(
                measurement.estimated_pose, measurement.timestamp_seconds, measurement.standard_deviations
            )

    def get_vision_seeded_pose(self) -> Optional[Pose2d]:
        if self._vision_subsystem is None:
            return None
        measurement = self._vision_subsystem.get_best_vision_measurement_if_fresh()
        return measurement.estimated_pose if measurement is not None else None

    # ---- PathPlanner Auto ----

    def get_auto_command(self, auto_name: str) -> Command:
        return PathPlannerAuto(auto_name)

    # ---- Telemetry ----

    def _update_telemetry(self) -> None:
        self._field.setRobotPose(self.get_pose())
        wpilib.SmartDashboard.putNumber("DriveTrain/LeftDistMeters", self.get_left_distance_meters())
        wpilib.SmartDashboard.putNumber("DriveTrain/RightDistMeters", self.get_right_distance_meters())
        wpilib.SmartDashboard.putNumber("DriveTrain/HeadingDeg", self.get_heading().degrees())
        wpilib.SmartDashboard.putBoolean("DriveTrain/ReverseDriving", self._reverse_driving)
