"""Robot-wide numerical/boolean constants.

Ported from Constants.java. Constants are grouped into namespace classes the
same way as the original — this class should not contain anything functional.
"""

import math

from wpimath.geometry import Pose2d, Rotation2d, Rotation3d, Transform3d, Translation3d


class OperatorConstants:
    DRIVER_CONTROLLER_PORT = 0
    OPERATOR_CONTROLLER_PORT = 1


class ClimberConstants:
    pass


class DriveTrainConstants:
    # CAN IDs
    LEFT_LEAD_CAN_ID = 20
    RIGHT_LEAD_CAN_ID = 22
    LEFT_FOLLOW_CAN_ID = 21
    RIGHT_FOLLOW_CAN_ID = 23

    # Motor config
    CURRENT_LIMIT = 60  # amps

    # Pose initialization — vision measurements up to this many seconds old are
    # accepted when seeding the pose at auto/teleop init. More generous than the
    # in-match freshness window because any recent fix beats defaulting to field origin.
    POSE_INIT_MAX_VISION_AGE_SECONDS = 5.0

    # Driving
    JOYSTICK_DEADBAND = 0.05
    TELEMETRY_PERIOD_LOOPS = 5

    # Physical dimensions
    GEAR_RATIO = 8.46
    WHEEL_DIAMETER_METERS = 0.1524  # 6 inches
    WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * math.pi
    TRACK_WIDTH_METERS = 0.546

    SPEED_SCALE = 0.7


class IntakeConstants:
    pass


class ShooterConstants:
    # CAN IDs
    SHOOTER_MOTOR_ID = 30  # TalonFX (Phoenix 6) — shooter wheel
    INTAKE_MOTOR_ID = 31  # SparkMAX — intake roller (primary + secondary linked, CCW only)
    TRIGGER_MOTOR_ID = 32  # SparkMAX — trigger/hopper (bidirectional)

    # Motor inversion — verify polarity on bench, flip here if wrong
    # Intake roller:  positive set() should = CCW (into robot)
    # Trigger motor:  positive set() should = CW  (hopper -> shooter)
    SHOOTER_INVERTED = False  # TODO: verify on bench
    INTAKE_MOTOR_INVERTED = False  # TODO: verify on bench; CCW must be positive
    TRIGGER_MOTOR_INVERTED = False  # TODO: verify on bench; CW must be positive

    # Intake roller (CAN 31) speeds — this motor runs CCW only
    INTAKE_SPEED = 1.0  # 100% clockwise — pulls ball from ground into robot
    INTAKE_EJECT_SPEED = -1.0  # 100% counterclockwise — reverses to push ball back out

    # Shooter pulley sizes (belt drive between Kraken and shooter wheel shaft)
    SHOOTER_MOTOR_PULLEY_TEETH = 26.0  # 15T pulley on Kraken shaft
    SHOOTER_SHAFT_PULLEY_TEETH = 32.0  # 30T pulley on shooter wheel shaft
    SHOOTER_GEAR_RATIO = SHOOTER_MOTOR_PULLEY_TEETH / SHOOTER_SHAFT_PULLEY_TEETH

    # Trigger/hopper motor (CAN 32) speeds — bidirectional
    TRIGGER_FEED_SPEED = -0.5  # shooting: trigger runs opposite intake (exhale)
    TRIGGER_INTAKE_SPEED = 0.5  # 100% clockwise — both motors same direction during intake
    TRIGGER_EJECT_SPEED = -0.5  # same direction as intake during exhale — both motors reverse together
    JAM_REVERSE_SPEED = -0.5  # CCW — trigger jam-clear reverse

    NOMINAL_VOLTAGE = 12
    SHOOTER_TELEMETRY_PERIOD_LOOPS = 5

    # Current limits
    SHOOTER_CURRENT_LIMIT = 60  # TalonFX stator limit (amps)
    INTAKE_MOTOR_CURRENT_LIMIT = 50  # SparkMAX smart current limit (amps)
    TRIGGER_MOTOR_CURRENT_LIMIT = 50  # SparkMAX smart current limit (amps)

    # Jam detection (trigger motor — most likely jam point)
    TRIGGER_SPIKE_THRESHOLD_AMPS = 50.0  # current spike triggers jam clear
    JAM_REVERSE_TIME_SEC = 0.25  # duration of jam-clear reverse

    # Shooter PID / feedforward — Phoenix 6 on-controller slot 0
    TARGET_RPM_10_FEET = 3150.0  # interpolated mechanism RPM at 10 ft
    SHOOTER_KP = 1.0  # proportional (starting value)
    SHOOTER_KI = 0.0  # integral
    SHOOTER_KD = 0.0  # derivative
    SHOOTER_KV = 0.12  # feedforward (tune first)
    RPM_TOLERANCE = 50.0  # within +/-50 RPM is considered ready

    # Distance to mechanism RPM mapping (distance in feet -> mechanism RPM).
    # Values calculated from projectile physics (70 deg launch, 18" launcher height,
    # 72" target height, 0.556 slip factor, 4" wheel diameter, calibrated from test data).
    # Update SHOOTER_MOTOR_PULLEY_TEETH/SHOOTER_SHAFT_PULLEY_TEETH when gearing changes.
    DISTANCES_FEET = (5, 7.5, 10, 12.5, 15, 17.5, 18.75)
    DISTANCE_RPM_MAP = (2150, 2400, 2600, 2850, 3050, 3200, 3300)


class SensorConstants:
    # Photo sensor (ball detection) — DIO port 1
    # Set PHOTO_SENSOR_ENABLED = True once the sensor is physically installed
    PHOTO_SENSOR_DIO_PORT = 1
    PHOTO_SENSOR_ENABLED = False  # disabled until installed
    PHOTO_SENSOR_INVERTED = False  # TODO: verify polarity on bench


class Auto:
    # Single authoritative max robot velocity.
    # 15 ft/s converted to m/s. Used for both PathPlanner speed limiting
    # and the PPLTVController gain-table upper bound so they stay in sync.
    MAX_ROBOT_VELOCITY_MPS = 15.0 * 0.3048  # 4.572 m/s

    # PathPlanner motion limits
    MAX_MODULE_SPEED = MAX_ROBOT_VELOCITY_MPS  # m/s
    MAX_ACCELERATION = 2.0  # m/s^2
    MAX_ANGULAR_VELOCITY = 540.0  # deg/s
    MAX_ANGULAR_ACCELERATION = 720.0  # deg/s^2

    # PPLTVController tuning defaults (state tolerances and control effort limits).
    # Live adjustment is available on SmartDashboard during test mode only.
    PPLTV_DT = 0.02
    PPLTV_MAX_VELOCITY = MAX_ROBOT_VELOCITY_MPS  # matches MAX_MODULE_SPEED
    PPLTV_Q_X = 0.0625
    PPLTV_Q_Y = 0.125
    PPLTV_Q_THETA = 0.75
    PPLTV_R_VEL = 1.0
    PPLTV_R_OMEGA = 2.0


class VisionConstants:
    # Camera names (must match PhotonVision configuration)
    FRONT_CAMERA_NAME = "Front_Camera"
    REAR_CAMERA_NAME = "Rear_Camera"
    DRIVER_CAMERA_NAME = "Driver_Camera"

    # Camera transforms (robot-to-camera)
    # Estimated placement: centerline, 20" (~0.508m) above ground, 1" (~0.0254m) from edge
    # Assumptions: Robot is ~28" (0.71m) bumper-to-bumper, cameras tilted 30 deg down

    # Front PhotonVision camera (Pi4 + PiCam v2)
    # TODO: Measure actual robot dimensions and camera mounting position
    ROBOT_TO_FRONT_CAM = Transform3d(
        Translation3d(0.102, -0.181, 0.089),  # forward, left, up (meters) # TODO: Verify measurements
        Rotation3d(0.0, math.radians(-30), 0.0),  # roll, pitch, yaw # TODO: Measure actual camera angles
    )

    # Rear PhotonVision camera (Pi5 + OV9281) — rotated 180 deg (facing backwards)
    # TODO: Measure actual robot dimensions and camera mounting position
    ROBOT_TO_REAR_CAM = Transform3d(
        Translation3d(-0.305, 0.0, 0.318),  # TODO: Verify measurements
        Rotation3d(0.0, math.radians(-30), math.radians(180)),  # TODO: Measure actual camera angles
    )

    # Vision measurement quality gating
    MAX_TAG_DISTANCE_METERS = 4.0  # TODO: Tune based on camera performance
    MAX_AMBIGUITY = 0.3  # TODO: Tune based on field testing
    MIN_TAGS_FOR_MULTI_TAG = 2
    MAX_VISION_AGE_SECONDS = 0.5
    TELEMETRY_PERIOD_LOOPS = 5

    # Standard deviations for pose estimation, as (x meters, y meters, heading radians)
    # tuples — this is the format wpimath's DifferentialDrivePoseEstimator expects in
    # RobotPy (there is no separate Matrix<N3, N1> type on this binding).
    SINGLE_TAG_CLOSE_STDDEVS = (0.5, 0.5, math.radians(10))  # TODO: Tune based on testing
    SINGLE_TAG_FAR_STDDEVS = (1.0, 1.0, math.radians(20))  # TODO: Tune based on testing
    MULTI_TAG_STDDEVS = (0.2, 0.2, math.radians(5))  # TODO: Tune based on testing

    # Hub positions derived from the 2026-rebuilt-welded AprilTag layout.
    # Blue hub core tags (18-21, 24-27) span X=4.02-5.23, Y=3.43-4.64 -> center ~= (4.63, 4.03)
    # Red  hub core tags ( 2- 5,  8-11) span X=11.31-12.52, Y=3.43-4.64 -> center ~= (11.92, 4.03)
    BLUE_HUB_POSE = Pose2d(4.63, 4.03, Rotation2d())
    RED_HUB_POSE = Pose2d(11.92, 4.03, Rotation2d())

    # Field zone boundaries (X-axis, meters).
    # Field runs X=0 (blue DS wall) -> X=16.541 (red DS wall).
    # Offensive zone for each alliance = between their driver station and their hub.
    BLUE_OFFENSIVE_MAX_X = 5.2  # m — blue hub outer edge toward center
    RED_OFFENSIVE_MIN_X = 11.3  # m — red  hub outer edge toward center

    HP_STATION_POSE = Pose2d(1.5, 7.5, Rotation2d.fromDegrees(180))  # TODO: Update with actual 2026 game positions
    TRENCH_POSE = Pose2d(2.5, 2.0, Rotation2d.fromDegrees(0))  # TODO: Update with actual 2026 game positions
    DEPOT_POSE = Pose2d(14.0, 2.0, Rotation2d.fromDegrees(180))  # TODO: Update with actual 2026 game positions
    OUTPOST_POSE = Pose2d(8.27, 0.5, Rotation2d.fromDegrees(90))  # TODO: Update with actual 2026 game positions
    TOWER_POSE = Pose2d(8.27, 7.5, Rotation2d.fromDegrees(270))  # TODO: Update with actual 2026 game positions
