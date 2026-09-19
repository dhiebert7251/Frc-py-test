package frc.robot;

/**
 * Robot-wide numerical/boolean constants for the teaching-bot proof of concept.
 *
 * <p>One nested class per subsystem, same convention as the real competition port
 * (2026_competition_code) -- see that repo's Constants.java. Nothing functional lives
 * here, only numbers/IDs.
 *
 * <p>A note for anyone coming from the Python sibling of this project
 * (teaching-bot-poc, in this same repo): Python's constants.py used a plain class per
 * subsystem with bare {@code ALL_CAPS = value} attributes -- Python doesn't require a
 * value to be typed, and a class attribute is "constant" purely by convention (nothing
 * stops code from reassigning it). Java has no such convention-only option: every field
 * needs a declared type ({@code int}, {@code double}, {@code boolean}, ...), and making
 * it an actual, enforced constant needs two keywords together:
 * <ul>
 *   <li>{@code static} -- the field belongs to the CLASS itself, not to any one
 *       instance. Without it, every {@code new DriveTrainConstants()} would get its own
 *       separate copy of {@code TRACK_WIDTH_METERS} -- exactly backwards from what a
 *       shared constant needs. (Nothing in this file is ever actually instantiated with
 *       {@code new} -- these classes exist purely to hold {@code static} fields.)</li>
 *   <li>{@code final} -- once assigned, this field can never be reassigned. This is
 *       what actually makes it a *constant* rather than just a shared variable; leaving
 *       it off would compile fine but silently allow some other file to change
 *       {@code TRACK_WIDTH_METERS} out from under every other file that reads it.</li>
 * </ul>
 * Every field below is {@code public static final}, in that order by convention:
 * access modifier first, then {@code static}, then {@code final}, then the type, then
 * the name.
 *
 * <p>Assumptions worth knowing about (see README for the full list):
 * <ul>
 *   <li>The shooter's Kraken is a Kraken X60. Nothing in this project's simulation
 *       models the shooter at all, so this only matters if someone adds that later.</li>
 *   <li>Every gear ratio, motor inversion, limit-switch polarity, and PID gain below is
 *       a starting guess, marked TODO, meant to be corrected once the real robot exists
 *       to test against.</li>
 * </ul>
 */
public final class Constants {

    // A private constructor with no body is the standard Java idiom for "this class is
    // never meant to be instantiated" -- since every member below is static, a caller
    // never needs a Constants object, only Constants.SomeNestedClass.SOME_FIELD.
    // Marking the constructor private (rather than just leaving the default public one)
    // makes that intent something the compiler enforces, not just something a comment
    // asks nicely for.
    private Constants() {}

    // wpimath (WPILib's own math library) works in meters -- that's not a stylistic
    // choice we get to opt out of, it's baked into DifferentialDriveKinematics,
    // PIDController, etc. But FRC parts, and the humans driving/wrenching on the robot,
    // think in inches/feet. This one constant is the single conversion point between
    // those two worlds: every "feet" value entering this codebase (a command's
    // constructor argument, a dashboard readout) gets multiplied or divided by this,
    // once, right at that boundary -- see commands/DriveDistanceCommand.java and
    // DriveTrain.periodic() for the two places that actually happen.
    public static final double METERS_PER_FOOT = 0.3048;

    public static final class OperatorConstants {
        private OperatorConstants() {}

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class DriveTrainConstants {
        private DriveTrainConstants() {}

        // CAN IDs -- 4x NEO 2.0 via REV SparkMax, 2 per side (lead + follower).
        public static final int LEFT_LEAD_CAN_ID = 20;
        public static final int LEFT_FOLLOW_CAN_ID = 21;
        public static final int RIGHT_LEAD_CAN_ID = 22;
        public static final int RIGHT_FOLLOW_CAN_ID = 23;

        public static final int CURRENT_LIMIT = 60; // amps

        // Physical dimensions.
        // 6-wheel "drop center" drivetrain: 3 wheels per side, the center wheel mounted
        // 1/4" LOWER than the front/back wheels. On a rigid frame that means the center
        // wheel touches down first, and only ONE of the front or back wheels shares the
        // ground with it at any moment (whichever end the robot's weight happens to be
        // biased toward) -- never all 3. Effectively, each side only ever has 2 real
        // contact points on the ground, spaced closer together (center-to-front or
        // center-to-back, WHEEL_CENTER_SPACING_METERS) than the full front-to-back
        // wheelbase would be. A shorter ground-contact wheelbase means less wheel scrub
        // (sideways sliding) while turning, which is the entire point of dropping the
        // center wheel: 6-wheel traction/durability for driving straight, without
        // paying a 6-wheel-flat drivetrain's full turning friction penalty. It doesn't
        // change any of the kinematics math below: WPILib's DifferentialDriveKinematics
        // only cares about the distance between the left and right wheels
        // (TRACK_WIDTH_METERS), not how many wheels are on a side or which ones are
        // touching down.
        public static final double DROP_CENTER_WHEEL_DROP_METERS = 0.25 * 0.0254; // 1/4 inch, informational only

        public static final double WHEEL_DIAMETER_METERS = 6.0 * 0.0254; // 6 inches
        public static final double WHEEL_WIDTH_METERS = 1.0 * 0.0254; // 1 inch, informational only
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final double GEAR_RATIO = 8.4;

        public static final double TRACK_WIDTH_METERS = 23.0 * 0.0254; // left-to-right wheel center distance
        public static final double WHEEL_CENTER_SPACING_METERS = 13.0 * 0.0254; // front-mid/mid-back spacing, per side

        public static final double ROBOT_LENGTH_METERS = 32.0 * 0.0254; // front-to-back, bumpers included
        public static final double ROBOT_WIDTH_METERS = 28.0 * 0.0254; // side-to-side, bumpers included
        public static final double ROBOT_MASS_KG = 118.0 * 0.45359237; // with bumpers

        public static final double JOYSTICK_DEADBAND = 0.05;
        public static final int TELEMETRY_PERIOD_LOOPS = 5;
        public static final double SPEED_SCALE = 0.7;

        // Gyro-based turning (PID) -- see commands/TurnToAngleCommand.java.
        public static final double TURN_KP = 0.04;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.005;
        public static final double TURN_TOLERANCE_DEGREES = 2.0;

        // Distance-based driving (PID) -- see commands/DriveDistanceCommand.java. KP is
        // deliberately conservative (a large error, e.g. commanding 10 feet from a
        // standstill, would otherwise demand full power) and MAX_OUTPUT clamps the
        // controller's output as a second, independent safety margin on top of that.
        public static final double DRIVE_DISTANCE_KP = 1.5; // TODO: tune on the real robot -- starting point only
        public static final double DRIVE_DISTANCE_KI = 0.0;
        public static final double DRIVE_DISTANCE_KD = 0.1;
        public static final double DRIVE_DISTANCE_TOLERANCE_METERS = 0.05;
        public static final double DRIVE_DISTANCE_MAX_OUTPUT = 0.6; // clamp: never command more than 60% power
    }

    public static final class ShooterConstants {
        private ShooterConstants() {}

        // CAN ID: 30s decade, same subsystem-family convention as the competition bot
        // (Shooter/Trigger both feed the same game piece path).
        public static final int FLYWHEEL_MOTOR_ID = 30;

        public static final boolean FLYWHEEL_INVERTED = false; // TODO: verify on bench

        // 5 lb flywheel with 4x 4" compliant wheels as the shooting surface, driven by
        // a Kraken X60.
        public static final double FLYWHEEL_GEAR_RATIO = 1.0; // TODO: confirm -- assumed direct-drive until measured
        public static final double SHOOTER_WHEEL_DIAMETER_METERS = 4.0 * 0.0254;

        public static final int CURRENT_LIMIT = 40; // amps, TalonFX stator limit

        public static final double TARGET_RPM = 3000.0; // TODO: tune once the shooter is built
        public static final double RPM_TOLERANCE = 50.0;

        public static final double SHOOTER_KP = 0.11; // TODO: tune -- starting point only, not measured
        public static final double SHOOTER_KI = 0.0;
        public static final double SHOOTER_KD = 0.0;
        public static final double SHOOTER_KV = 0.12;
    }

    public static final class TriggerConstants {
        private TriggerConstants() {}

        // CAN ID: 30s decade, same family as Shooter.
        public static final int CAM_MOTOR_ID = 31;

        public static final boolean CAM_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int CAM_CURRENT_LIMIT = 20; // amps, small NEO
        public static final double CAM_FIRE_SPEED = 0.6; // [-1, 1] duty cycle while firing

        // DIO ports.
        public static final int LIMIT_SWITCH_DIO_PORT = 0;
        public static final int BEAM_BREAK_1_DIO_PORT = 1; // e.g. "ball loaded, waiting to fire"
        public static final int BEAM_BREAK_2_DIO_PORT = 2; // e.g. "ball at the shooter, ready to fire"

        public static final boolean LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench (NC vs NO)
        public static final boolean BEAM_BREAK_1_INVERTED = false; // TODO: verify polarity on bench
        public static final boolean BEAM_BREAK_2_INVERTED = false; // TODO: verify polarity on bench

        // Safety timeout in case the limit switch never re-triggers (a jam, a broken
        // wire) -- without this, FireCommand could run the motor forever. Applied as a
        // `.withTimeout()` decorator where FireCommand is bound in RobotContainer.java,
        // not inside the command itself -- see that file for why.
        public static final double FIRE_TIMEOUT_SECONDS = 2.0;
    }

    public static final class ElevatorConstants {
        private ElevatorConstants() {}

        // CAN ID: 40s decade -- a new subsystem family, one decade past Shooter/Trigger.
        public static final int LIFT_MOTOR_ID = 40;

        public static final boolean LIFT_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int LIFT_CURRENT_LIMIT = 30; // amps -- Redline motors are small, keep this conservative

        // No encoder on this motor: it's a brushed Redline with no built-in sensor, and
        // no external encoder is installed. This subsystem is entirely open-loop,
        // driven by a limit switch at each end of travel -- see subsystems/Elevator.java.
        public static final double RAISE_SPEED = 0.5; // [-1, 1] duty cycle while raising (spring-assisted)
        public static final double LOWER_SPEED = -0.7; // [-1, 1] duty cycle while lowering (against spring tension)

        public static final int TOP_LIMIT_SWITCH_DIO_PORT = 3;
        public static final int BOTTOM_LIMIT_SWITCH_DIO_PORT = 4;
        public static final boolean TOP_LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench
        public static final boolean BOTTOM_LIMIT_SWITCH_INVERTED = false; // TODO: verify polarity on bench
    }

    public static final class GripperConstants {
        private GripperConstants() {}

        // CAN ID: 40s decade, same family as Elevator (it rides on the elevator).
        public static final int ROLLER_MOTOR_ID = 41;

        public static final boolean ROLLER_MOTOR_INVERTED = false; // TODO: verify on bench
        public static final int ROLLER_CURRENT_LIMIT = 20; // amps, small NEO 550

        public static final double INTAKE_SPEED = 1.0;
        public static final double EJECT_SPEED = -1.0;
    }

    public static final class Auto {
        private Auto() {}

        // The two autonomous routines' actual distances/angle, in feet/degrees (the
        // "human" units) -- named here so they're easy to find and change without
        // hunting through autonomous/AutoRoutines.java. Converted to meters only inside
        // DriveDistanceCommand, right at the WPILib-math boundary.
        public static final double DRIVE_FORWARD_ONLY_FEET = 10.0;
        public static final double DRIVE_TURN_DRIVE_FIRST_LEG_FEET = 5.0;
        public static final double DRIVE_TURN_DRIVE_TURN_DEGREES = 90.0; // positive = left (CCW)
        public static final double DRIVE_TURN_DRIVE_SECOND_LEG_FEET = 3.0;
    }
}
