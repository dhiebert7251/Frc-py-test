"""DriveTrain subsystem -- 6-wheel drop-center differential (tank) drive.

Teaching-bot proof of concept.

A **subsystem** in commands2 represents one physical mechanism and owns all
the hardware objects for it (motor controllers, sensors). Its job is
narrow on purpose: know how to DO things right now (spin the motors at a
given power, report what a sensor currently reads) and nothing about WHEN
or for HOW LONG to do them. That "when/how long" logic -- including
anything with real state, like a PID loop -- lives in commands/, as
explicit Command classes that call the plain methods defined below. See
the README's "Where do commands live?" section for the full reasoning
behind that split, and commands/drivetrain_commands.py for this
subsystem's four commands (teleop drive, drive-to-distance, turn-to-angle,
reset gyro).

This subsystem deliberately stops at raw encoder distances and a raw gyro
heading -- no PathPlanner, no vision, no pose estimator/odometry fusing
them together. Pose estimation is a natural *next* lesson once encoders,
gyro, and PID are all comfortable on their own, not a starting one.
"""
from __future__ import annotations

import navx
import wpilib
import wpilib.drive
import wpimath
from commands2 import Subsystem
from rev import ResetMode, PersistMode, SparkBaseConfig, SparkMax, SparkMaxConfig

from constants import DriveTrainConstants, METERS_PER_FOOT


class DriveTrain(Subsystem):
    def __init__(self) -> None:
        # Every Subsystem must call its parent's __init__() first. This is
        # what registers the subsystem with the CommandScheduler, which is
        # how the scheduler later knows "these two commands both want
        # DriveTrain, so they can't run at the same time."
        super().__init__()

        # ---- Motor groupings: 2 physical motors per side, "lead" + "follower" ----
        #
        # Each side of the drivetrain has 2 NEO 2.0 motors, but we only want
        # to give the software ONE number per side ("drive the left side at
        # 50% power"), not have to command two motors separately and keep
        # them in sync by hand. REV's SparkMax solves this with a
        # lead/follower relationship, configured below in _configure_motors():
        # the "follow" motor is told, once, "always match whatever the lead
        # motor is doing" -- after that, our code only ever talks to the two
        # LEAD motors (self._left_lead, self._right_lead). The two FOLLOW
        # motors (self._left_follow, self._right_follow) exist as Python
        # objects here only so we can configure them once at startup; nothing
        # in this file calls .set() or reads a sensor from a follow motor
        # again after __init__.
        #
        # The encoders are also read from the lead motors only (see below) --
        # a follower's own encoder still spins with its motor, but since it's
        # mechanically forced to match the leader, reading the leader's
        # encoder tells us everything we need to know about that whole side.
        self._left_lead = SparkMax(DriveTrainConstants.LEFT_LEAD_CAN_ID, SparkMax.MotorType.kBrushless)
        self._left_follow = SparkMax(DriveTrainConstants.LEFT_FOLLOW_CAN_ID, SparkMax.MotorType.kBrushless)
        self._right_lead = SparkMax(DriveTrainConstants.RIGHT_LEAD_CAN_ID, SparkMax.MotorType.kBrushless)
        self._right_follow = SparkMax(DriveTrainConstants.RIGHT_FOLLOW_CAN_ID, SparkMax.MotorType.kBrushless)

        # RelativeEncoder objects for the two lead motors. NEOs and NEO 2.0s
        # both have a built-in encoder inside the motor -- no separate sensor
        # to wire up, unlike Elevator's brushed Redline motor (see
        # subsystems/elevator.py for that contrast). getEncoder() with no
        # arguments returns the motor's built-in one.
        self._left_encoder = self._left_lead.getEncoder()
        self._right_encoder = self._right_lead.getEncoder()

        # The navX2 is a gyroscope (and more -- accelerometer, magnetometer)
        # that plugs into the roboRIO's MXP port and reports over SPI. This
        # is the only sensor on the robot that isn't a motor's built-in
        # encoder or a simple digital switch, which is why it needs its own
        # vendor library (robotpy-navx, imported above) instead of coming
        # from `rev` or core `wpilib`.
        self._gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        self._configure_motors()

        # DifferentialDrive is a small WPILib helper that takes "how fast
        # should the left/right side go" and turns that into calls on the
        # two motors it wraps -- it does NOT know about the follower motors
        # at all, because it doesn't need to (that's the whole point of
        # having configured them to follow, above).
        self._driver = wpilib.drive.DifferentialDrive(self._left_lead, self._right_lead)

        # Used by periodic() below to only publish telemetry every Nth loop
        # instead of every ~20ms -- SmartDashboard/NetworkTables traffic adds
        # up, and nothing reads these values fast enough to need them every
        # single loop.
        self._telemetry_loop_counter = 0

    def _configure_motors(self) -> None:
        """One-time SparkMax setup for all 4 drive motors, called once from
        __init__(). Nothing in here runs again after startup."""

        # A SparkMaxConfig object describes a full desired configuration; it
        # doesn't take effect until passed to .configure(). This "build a
        # config object, then apply it" two-step (rather than one call per
        # setting) is REVLib's pattern for every SparkMax on this robot.
        #
        # positionConversionFactor/velocityConversionFactor rescale the raw
        # "motor shaft rotations" the encoder actually measures into
        # "meters the robot has driven" -- multiplying by wheel circumference
        # accounts for one wheel rotation = one circumference of travel, and
        # dividing by the gear ratio accounts for the motor spinning
        # GEAR_RATIO times for every one wheel rotation. Only the LEAD
        # motors' encoders are configured this way, since those are the only
        # encoders this code ever reads (see the comment in __init__ above).
        conversion_factor = DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS / DriveTrainConstants.GEAR_RATIO

        # Right lead. inverted(True) because of how the gearboxes are
        # mounted: a physically-mirrored drivetrain means the left and right
        # gearboxes spin opposite directions for "both sides forward," so
        # exactly one side needs its sign flipped in software.
        right_lead_config = SparkMaxConfig()
        right_lead_config.inverted(True)
        right_lead_config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        right_lead_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        right_lead_config.encoder.positionConversionFactor(conversion_factor)
        right_lead_config.encoder.velocityConversionFactor(conversion_factor / 60.0)
        self._right_lead.configure(right_lead_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        # Left lead -- same idea, not inverted.
        left_lead_config = SparkMaxConfig()
        left_lead_config.inverted(False)
        left_lead_config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        left_lead_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        left_lead_config.encoder.positionConversionFactor(conversion_factor)
        left_lead_config.encoder.velocityConversionFactor(conversion_factor / 60.0)
        self._left_lead.configure(left_lead_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        # Follow motors: `.follow(lead_motor)` is the whole configuration --
        # this one call is what makes "command the lead, the follower copies
        # it" happen. No encoder conversion factors here, because this
        # code never reads a follower's encoder (see __init__ comment).
        left_follow_config = SparkMaxConfig()
        left_follow_config.follow(self._left_lead)
        left_follow_config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        left_follow_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        self._left_follow.configure(
            left_follow_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )

        right_follow_config = SparkMaxConfig()
        right_follow_config.follow(self._right_lead)
        right_follow_config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        right_follow_config.smartCurrentLimit(DriveTrainConstants.CURRENT_LIMIT)
        self._right_follow.configure(
            right_follow_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )

        # Start both encoders at exactly 0 meters traveled. Without this,
        # whatever the encoder happened to read when the robot was last
        # powered off would carry over.
        self._left_encoder.setPosition(0)
        self._right_encoder.setPosition(0)

    # ---- Plain hardware actions (no scheduling, no state machines) ----
    #
    # Everything below is intentionally "dumb": each method does exactly one
    # thing to the hardware, right now, and returns. Commands (in
    # commands/drivetrain_commands.py) call these repeatedly, in whatever
    # pattern they need, to build actual robot behavior over time.

    def drive(self, left: float, right: float) -> None:
        """Tank-drives at the given left/right duty cycles, each in [-1, 1].
        applyDeadband zeroes out small values -- without it, a joystick that
        doesn't return to *exactly* 0.0 when released would creep the robot."""
        self._driver.tankDrive(
            wpimath.applyDeadband(left, DriveTrainConstants.JOYSTICK_DEADBAND),
            wpimath.applyDeadband(right, DriveTrainConstants.JOYSTICK_DEADBAND),
        )

    def stop(self) -> None:
        self._driver.stopMotor()

    # ---- Sensors ----
    #
    # These all return SI units (meters, meters/second, degrees for angle --
    # there's no "imperial degrees") even though nothing else in FRC is
    # metric. That's deliberate: WPILib's own math expects meters, so
    # keeping this subsystem's internal numbers in meters means it can be
    # handed directly to kinematics/PID code without a conversion at every
    # call site. The conversion to feet (for humans) happens in exactly two
    # places: this file's periodic() telemetry, and
    # commands/drivetrain_commands.py's DriveDistanceCommand constructor,
    # which is the one place a "how many feet" number enters this
    # subsystem from the outside.

    def get_left_distance_meters(self) -> float:
        return self._left_encoder.getPosition()

    def get_right_distance_meters(self) -> float:
        return self._right_encoder.getPosition()

    def get_average_distance_meters(self) -> float:
        return (self.get_left_distance_meters() + self.get_right_distance_meters()) / 2.0

    def get_left_velocity_meters_per_second(self) -> float:
        return self._left_encoder.getVelocity()

    def get_right_velocity_meters_per_second(self) -> float:
        return self._right_encoder.getVelocity()

    def reset_encoders(self) -> None:
        self._left_encoder.setPosition(0)
        self._right_encoder.setPosition(0)

    def get_heading_degrees(self) -> float:
        # Negated for CCW-positive: the navX reports clockwise-positive by
        # default, but WPILib's convention (and this codebase's) is
        # counterclockwise-positive, so every raw reading gets flipped right
        # here -- the one place that has to know about that mismatch.
        return -self._gyro.getAngle()

    def reset_gyro(self) -> None:
        self._gyro.reset()

    def periodic(self) -> None:
        # periodic() runs every ~20ms for every subsystem, whether or not a
        # command is currently using it -- this is the right place for
        # "always keep this updated" bookkeeping like telemetry, as opposed
        # to logic that should only happen while a specific command is
        # active (that belongs in that command's execute()).
        self._telemetry_loop_counter += 1
        if self._telemetry_loop_counter >= DriveTrainConstants.TELEMETRY_PERIOD_LOOPS:
            self._telemetry_loop_counter = 0
            # Dashboard values are published in feet and feet/sec -- the
            # units a human glancing at Shuffleboard actually thinks in --
            # even though everything above this point works in meters.
            wpilib.SmartDashboard.putNumber(
                "DriveTrain/LeftDistFeet", self.get_left_distance_meters() / METERS_PER_FOOT
            )
            wpilib.SmartDashboard.putNumber(
                "DriveTrain/RightDistFeet", self.get_right_distance_meters() / METERS_PER_FOOT
            )
            wpilib.SmartDashboard.putNumber(
                "DriveTrain/LeftVelocityFPS", self.get_left_velocity_meters_per_second() / METERS_PER_FOOT
            )
            wpilib.SmartDashboard.putNumber(
                "DriveTrain/RightVelocityFPS", self.get_right_velocity_meters_per_second() / METERS_PER_FOOT
            )
            wpilib.SmartDashboard.putNumber("DriveTrain/HeadingDeg", self.get_heading_degrees())
