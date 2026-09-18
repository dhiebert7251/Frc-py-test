"""Robot-wide numerical/boolean constants for the teaching-bot proof of concept.

One namespace class per subsystem, same convention as the competition
port's constants.py -- see that repo's README "Naming and numbering
conventions" section. Nothing functional lives here, only numbers/IDs.

Assumptions worth knowing about (see README for the full list):
  * The shooter's Kraken is a Kraken X60. physics.py doesn't model the
    shooter at all, so this only matters if someone adds that later.
  * Every gear ratio, motor inversion, limit-switch/beam-break polarity, and
    PID gain below is a starting guess, marked TODO, meant to be corrected
    once the real robot exists to test against.
"""

import math

# WPILib's own math (wpimath) works in meters -- that's not a stylistic
# choice we get to opt out of, it's baked into DifferentialDriveKinematics,
# PIDController, etc. But FRC parts, and the humans driving/wrenching on the
# robot, think in inches/feet. This one constant is the single conversion
# point between those two worlds: every "feet" value entering this codebase
# (a command's constructor argument, a dashboard readout) gets multiplied or
# divided by this, once, right at that boundary -- see
# commands/drivetrain_commands.py and DriveTrain.periodic() for the two
# places that actually happen.
METERS_PER_FOOT = 0.3048


class OperatorConstants:
    DRIVER_CONTROLLER_PORT = 0
    OPERATOR_CONTROLLER_PORT = 1


class DriveTrainConstants:
    # CAN IDs -- 4x NEO 2.0 via REV SparkMax, 2 per side (lead + follower).
    LEFT_LEAD_CAN_ID = 20
    LEFT_FOLLOW_CAN_ID = 21
    RIGHT_LEAD_CAN_ID = 22
    RIGHT_FOLLOW_CAN_ID = 23

    CURRENT_LIMIT = 60  # amps

    # Physical dimensions.
    # 6-wheel "drop center" drivetrain: 3 wheels per side, the center wheel
    # mounted 1/4" LOWER than the front/back wheels. On a rigid frame that
    # means the center wheel touches down first, and only ONE of the front
    # or back wheels shares the ground with it at any moment (whichever end
    # the robot's weight happens to be biased toward) -- never all 3.
    # Effectively, each side only ever has 2 real contact points on the
    # ground, spaced closer together (center-to-front or center-to-back,
    # WHEEL_CENTER_SPACING_METERS) than the full front-to-back wheelbase
    # would be. A shorter ground-contact wheelbase means less wheel scrub
    # (sideways sliding) while turning, which is the entire point of
    # dropping the center wheel: 6-wheel traction/durability for driving
    # straight, without paying a 6-wheel-flat drivetrain's full turning
    # friction penalty. It doesn't change any of the kinematics math below:
    # WPILib's DifferentialDriveKinematics only cares about the distance
    # between the left and right wheels (TRACK_WIDTH_METERS), not how many
    # wheels are on a side or which ones are touching down.
    DROP_CENTER_WHEEL_DROP_METERS = 0.25 * 0.0254  # 1/4 inch, informational only

    WHEEL_DIAMETER_METERS = 6.0 * 0.0254  # 6 inches
    WHEEL_WIDTH_METERS = 1.0 * 0.0254  # 1 inch, informational only (not used in kinematics)
    WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * math.pi

    GEAR_RATIO = 8.4

    TRACK_WIDTH_METERS = 23.0 * 0.0254  # left-to-right wheel center distance
    WHEEL_CENTER_SPACING_METERS = 13.0 * 0.0254  # front-mid and mid-back spacing, per side

    ROBOT_LENGTH_METERS = 32.0 * 0.0254  # front-to-back, bumpers included
    ROBOT_WIDTH_METERS = 28.0 * 0.0254  # side-to-side, bumpers included
    ROBOT_MASS_KG = 118.0 * 0.45359237  # with bumpers

    JOYSTICK_DEADBAND = 0.05
    TELEMETRY_PERIOD_LOOPS = 5
    SPEED_SCALE = 0.7

    # Gyro-based turning (PID) -- see commands/drivetrain_commands.py's
    # TurnToAngleCommand.
    TURN_KP = 0.04
    TURN_KI = 0.0
    TURN_KD = 0.005
    TURN_TOLERANCE_DEGREES = 2.0

    # Distance-based driving (PID) -- see commands/drivetrain_commands.py's
    # DriveDistanceCommand. KP is deliberately conservative (a large error,
    # e.g. commanding 10 feet from a standstill, would otherwise demand full
    # power) and MAX_OUTPUT clamps the controller's output as a second,
    # independent safety margin on top of that.
    DRIVE_DISTANCE_KP = 1.5  # TODO: tune on the real robot -- starting point only
    DRIVE_DISTANCE_KI = 0.0
    DRIVE_DISTANCE_KD = 0.1
    DRIVE_DISTANCE_TOLERANCE_METERS = 0.05
    DRIVE_DISTANCE_MAX_OUTPUT = 0.6  # clamp: never command more than 60% power from this loop


class ShooterConstants:
    # CAN ID: 30s decade, same subsystem-family convention as the
    # competition bot (Shooter/Trigger both feed the same game piece path).
    FLYWHEEL_MOTOR_ID = 30

    FLYWHEEL_INVERTED = False  # TODO: verify on bench

    # 5 lb flywheel with 4x 4" compliant wheels as the shooting surface,
    # driven by a Kraken X60.
    FLYWHEEL_GEAR_RATIO = 1.0  # TODO: confirm -- assumed direct-drive from the Kraken X60 until measured
    SHOOTER_WHEEL_DIAMETER_METERS = 4.0 * 0.0254

    CURRENT_LIMIT = 40  # amps, TalonFX stator limit

    TARGET_RPM = 3000.0  # TODO: tune once the shooter is built
    RPM_TOLERANCE = 50.0

    SHOOTER_KP = 0.11  # TODO: tune -- starting point only, not measured
    SHOOTER_KI = 0.0
    SHOOTER_KD = 0.0
    SHOOTER_KV = 0.12


class TriggerConstants:
    # CAN ID: 30s decade, same family as Shooter.
    CAM_MOTOR_ID = 31

    CAM_MOTOR_INVERTED = False  # TODO: verify on bench
    CAM_CURRENT_LIMIT = 20  # amps, small NEO
    CAM_FIRE_SPEED = 0.6  # [-1, 1] duty cycle while firing

    # DIO ports.
    LIMIT_SWITCH_DIO_PORT = 0
    BEAM_BREAK_1_DIO_PORT = 1  # e.g. "ball loaded, waiting to fire"
    BEAM_BREAK_2_DIO_PORT = 2  # e.g. "ball at the shooter, ready to fire"

    LIMIT_SWITCH_INVERTED = False  # TODO: verify polarity on bench (NC vs NO wiring)
    BEAM_BREAK_1_INVERTED = False  # TODO: verify polarity on bench
    BEAM_BREAK_2_INVERTED = False  # TODO: verify polarity on bench

    # Safety timeout in case the limit switch never re-triggers (a jam, a
    # broken wire) -- without this, FireCommand could run the motor forever.
    # Applied as a `.withTimeout()` decorator where FireCommand is bound in
    # robotcontainer.py, not inside the command itself -- see that file for
    # why.
    FIRE_TIMEOUT_SECONDS = 2.0


class ElevatorConstants:
    # CAN ID: 40s decade -- a new subsystem family, one decade past Shooter/Trigger.
    LIFT_MOTOR_ID = 40

    LIFT_MOTOR_INVERTED = False  # TODO: verify on bench
    LIFT_CURRENT_LIMIT = 30  # amps -- Redline motors are small, keep this conservative

    # No encoder on this motor: it's a brushed Redline with no built-in
    # sensor, and no external encoder is installed. This subsystem is
    # entirely open-loop, driven by a limit switch at each end of travel --
    # see subsystems/elevator.py.
    RAISE_SPEED = 0.5  # [-1, 1] duty cycle while raising (spring-assisted -- needs less power)
    LOWER_SPEED = -0.7  # [-1, 1] duty cycle while lowering (pulling the rope in against the springs)

    TOP_LIMIT_SWITCH_DIO_PORT = 3
    BOTTOM_LIMIT_SWITCH_DIO_PORT = 4
    TOP_LIMIT_SWITCH_INVERTED = False  # TODO: verify polarity on bench
    BOTTOM_LIMIT_SWITCH_INVERTED = False  # TODO: verify polarity on bench


class GripperConstants:
    # CAN ID: 40s decade, same family as Elevator (it rides on the elevator).
    ROLLER_MOTOR_ID = 41

    ROLLER_MOTOR_INVERTED = False  # TODO: verify on bench
    ROLLER_CURRENT_LIMIT = 20  # amps, small NEO 550

    INTAKE_SPEED = 1.0
    EJECT_SPEED = -1.0


class Auto:
    # The two autonomous routines' actual distances/angle, in feet/degrees
    # (the "human" units) -- named here so they're easy to find and change
    # without hunting through autonomous/routines.py. Converted to meters
    # only inside DriveDistanceCommand, right at the WPILib-math boundary.
    DRIVE_FORWARD_ONLY_FEET = 10.0
    DRIVE_TURN_DRIVE_FIRST_LEG_FEET = 5.0
    DRIVE_TURN_DRIVE_TURN_DEGREES = 90.0  # positive = left (CCW), matches WPILib's convention
    DRIVE_TURN_DRIVE_SECOND_LEG_FEET = 3.0
