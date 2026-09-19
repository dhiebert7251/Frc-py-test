package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrainConstants.*;
import static frc.robot.Constants.METERS_PER_FOOT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * DriveTrain subsystem -- 6-wheel drop-center differential (tank) drive.
 *
 * <p>Teaching-bot proof of concept.
 *
 * <p>A <b>subsystem</b> in the WPILib command-based framework represents one physical
 * mechanism and owns all the hardware objects for it (motor controllers, sensors). Its
 * job is narrow on purpose: know how to DO things right now (spin the motors at a given
 * power, report what a sensor currently reads) and nothing about WHEN or for HOW LONG
 * to do them. That "when/how long" logic -- including anything with real state, like a
 * PID loop -- lives in commands/, as explicit Command classes that call the plain
 * methods defined below. See the README's "Where do commands live?" section for the
 * full reasoning behind that split, and commands/ for this subsystem's four commands
 * (teleop drive, drive-to-distance, turn-to-angle, reset gyro).
 *
 * <p>{@code extends SubsystemBase} is Java's equivalent of Python's
 * {@code class DriveTrain(Subsystem):} -- {@code SubsystemBase} is the WPILib base
 * class that registers this object with the CommandScheduler and gives it a default,
 * do-nothing {@code periodic()} to override.
 *
 * <p>The {@code teaching-bot-poc-java} branch this one builds on stopped at raw
 * encoder distances and a raw gyro heading -- no kinematics, no odometry. This branch
 * adds {@code DifferentialDriveKinematics} (the math relating each wheel's own speed
 * to the whole robot's speed/turn rate) and pure encoder+gyro dead reckoning via
 * {@code DifferentialDriveOdometry} to track the robot's estimated (X, Y, heading)
 * position on the field -- exactly the same two pieces, in the same order, that the
 * Python sibling's {@code teaching-bot-odometry} branch adds to its own
 * {@code DriveTrain}. Vision-based drift correction is a deliberate <i>next</i>
 * lesson, not a starting one -- see the {@code teaching-bot-vision-java} branch.
 */
public class DriveTrain extends SubsystemBase {

    // ---- Motor groupings: 2 physical motors per side, "lead" + "follower" ----
    //
    // Each side of the drivetrain has 2 NEO 2.0 motors, but we only want to give the
    // software ONE number per side ("drive the left side at 50% power"), not have to
    // command two motors separately and keep them in sync by hand. REV's SparkMax
    // solves this with a lead/follower relationship, configured below in
    // configureMotors(): the "follow" motor is told, once, "always match whatever the
    // lead motor is doing" -- after that, our code only ever talks to the two LEAD
    // motors. The two FOLLOW motors exist as Java objects here only so we can configure
    // them once at startup; nothing in this file calls .set() or reads a sensor from a
    // follower again after the constructor runs.
    //
    // `private final` on every hardware field below: `private` means only this class's
    // own methods can reach it directly (an outside caller has to go through a public
    // method like drive() or getLeftDistanceMeters() instead); `final` means the field
    // is assigned exactly once -- here, right where it's declared -- and can never be
    // reassigned to point at a different SparkMax object afterward. Unlike Python,
    // where "private" is only the `_` naming convention this project's own style guide
    // enforces, Java's `private` is a real access restriction the compiler checks.
    private final SparkMax leftLead = new SparkMax(LEFT_LEAD_CAN_ID, MotorType.kBrushless);
    private final SparkMax leftFollow = new SparkMax(LEFT_FOLLOW_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightLead = new SparkMax(RIGHT_LEAD_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightFollow = new SparkMax(RIGHT_FOLLOW_CAN_ID, MotorType.kBrushless);

    // RelativeEncoder objects for the two lead motors. NEOs and NEO 2.0s both have a
    // built-in encoder inside the motor -- no separate sensor to wire up, unlike
    // Elevator's brushed Redline motor (see Elevator.java for that contrast).
    // getEncoder() with no arguments returns the motor's built-in one.
    //
    // PACKAGE-PRIVATE (no access modifier at all), not `private`, and that's a
    // deliberate exception to the `private` rule every other hardware field on this
    // page follows. DriveTrainTest.java (in this same frc.robot.subsystems package)
    // needs to poke these two objects' simulated position directly with
    // `.setPosition(...)` to test odometry and the PID commands without a real robot --
    // the Python sibling's equivalent test does the same thing by reaching past its
    // `_left_encoder`'s leading-underscore naming CONVENTION, since Python has no
    // enforced privacy to get past. Java's `private` is enforced by the compiler with
    // no such loophole, so getting the same test access here needs an actual, coarser
    // access level instead of a bypassable naming hint. Package-private is the
    // narrowest level that still works: any class in frc.robot.subsystems can reach
    // these fields, but nothing outside that package (including RobotContainer.java,
    // in frc.robot) can -- a real, if slightly wider, restriction, not merely a polite
    // request.
    final RelativeEncoder leftEncoder = leftLead.getEncoder();
    final RelativeEncoder rightEncoder = rightLead.getEncoder();

    // The navX2 is a gyroscope (and more -- accelerometer, magnetometer) that plugs
    // into the roboRIO's MXP port and reports over SPI. This is the only sensor on the
    // robot that isn't a motor's built-in encoder or a simple digital switch, which is
    // why it needs its own vendor library (Studica, imported above) instead of coming
    // from `com.revrobotics` or core `edu.wpi.first.wpilibj`.
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // DifferentialDrive is a small WPILib helper that takes "how fast should the
    // left/right side go" and turns that into calls on the two motors it wraps -- it
    // does NOT know about the follower motors at all, because it doesn't need to
    // (that's the whole point of having configured them to follow, below).
    private final DifferentialDrive driver = new DifferentialDrive(leftLead, rightLead);

    // ---- Kinematics and odometry ----
    //
    // DifferentialDriveKinematics only needs one number -- the track width
    // (left-to-right wheel spacing) -- to convert between "each wheel's own speed" and
    // "the whole robot's forward speed and turn rate" (a ChassisSpeeds). It doesn't
    // track anything over time by itself; getChassisSpeeds() below is the only place
    // this project currently uses it.
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    // DifferentialDriveOdometry is the part that actually accumulates position OVER
    // TIME. Every loop, periodic() below feeds it the current gyro heading and both
    // encoder distances, and it integrates those into a running pose estimate -- dead
    // reckoning, the same technique ships have used for centuries: no outside
    // reference, just "I know my heading and how far each wheel has turned, so here's
    // where I must be now." Constructed here with the encoders already zeroed (see
    // configureMotors() below, called from the constructor before this field is
    // initialized) and the gyro's current heading, starting pose defaulted to
    // Pose2d() (X=0, Y=0, heading=0) -- a stand-in field origin until resetPose() sets
    // a real one.
    private final DifferentialDriveOdometry odometry;

    // Field2d is a Shuffleboard/Glass widget that draws the robot as an icon on a
    // picture of the field, at whatever pose you last gave it -- registered once in
    // the constructor (not every loop) so Shuffleboard doesn't see it
    // appear/disappear/duplicate.
    private final Field2d field = new Field2d();

    // Used by periodic() below to only publish telemetry every Nth loop instead of
    // every ~20ms -- SmartDashboard/NetworkTables traffic adds up, and nothing reads
    // these values fast enough to need them every single loop. Not `final`: this one
    // field IS reassigned, every loop, in periodic() below.
    private int telemetryLoopCounter = 0;

    /**
     * The constructor -- Java calls this automatically for {@code new DriveTrain()}.
     * Unlike every command class in this project (see commands/TeleopDriveCommand.java
     * for the full explanation of constructor syntax), this constructor takes no
     * parameters at all: DriveTrain doesn't need anything handed to it from the
     * outside to build itself, since every value it needs (CAN IDs, current limits)
     * comes from {@code Constants.DriveTrainConstants} instead.
     *
     * <p>{@code odometry} is assigned here, in the constructor BODY, rather than
     * inline at its field declaration the way {@code kinematics} and {@code field}
     * are above -- it's the one field whose initial value depends on calling
     * {@code getHeadingDegrees()}/{@code getLeftDistanceMeters()}/
     * {@code getRightDistanceMeters()}, which in turn need {@code configureMotors()}
     * to have already zeroed the encoders. Java runs field initializers and the
     * constructor body in the order they're written, top to bottom, so
     * {@code configureMotors()} has to be called first, right here, before
     * {@code odometry} can be built from a known-zero starting state.
     */
    public DriveTrain() {
        configureMotors();

        odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeadingDegrees()),
            getLeftDistanceMeters(),
            getRightDistanceMeters()
        );

        SmartDashboard.putData("Field", field);
    }

    /**
     * One-time SparkMax setup for all 4 drive motors, called once from the
     * constructor. Nothing in here runs again after startup. {@code private}: this is
     * an internal implementation detail, never meant to be called from outside this
     * class -- unlike {@code drive()}/{@code stop()}/the getters below, which are
     * {@code public} because commands need to call them.
     */
    private void configureMotors() {
        // A SparkMaxConfig object describes a full desired configuration; it doesn't
        // take effect until passed to .configure(). This "build a config object, then
        // apply it" two-step (rather than one call per setting) is REVLib's pattern for
        // every SparkMax on this robot -- identical in Java and Python, just with
        // camelCase method names either way (REVLib's Java API was never snake_cased).
        //
        // positionConversionFactor/velocityConversionFactor rescale the raw "motor
        // shaft rotations" the encoder actually measures into "meters the robot has
        // driven" -- multiplying by wheel circumference accounts for one wheel
        // rotation = one circumference of travel, and dividing by the gear ratio
        // accounts for the motor spinning GEAR_RATIO times for every one wheel
        // rotation. Only the LEAD motors' encoders are configured this way, since those
        // are the only encoders this code ever reads (see the field comments above).
        double conversionFactor = WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO;

        // Right lead. inverted(true) because of how the gearboxes are mounted: a
        // physically-mirrored drivetrain means the left and right gearboxes spin
        // opposite directions for "both sides forward," so exactly one side needs its
        // sign flipped in software.
        SparkMaxConfig rightLeadConfig = new SparkMaxConfig();
        rightLeadConfig.inverted(true);
        rightLeadConfig.idleMode(IdleMode.kCoast);
        rightLeadConfig.smartCurrentLimit(CURRENT_LIMIT);
        rightLeadConfig.encoder.positionConversionFactor(conversionFactor);
        rightLeadConfig.encoder.velocityConversionFactor(conversionFactor / 60.0);
        rightLead.configure(rightLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Left lead -- same idea, not inverted.
        SparkMaxConfig leftLeadConfig = new SparkMaxConfig();
        leftLeadConfig.inverted(false);
        leftLeadConfig.idleMode(IdleMode.kCoast);
        leftLeadConfig.smartCurrentLimit(CURRENT_LIMIT);
        leftLeadConfig.encoder.positionConversionFactor(conversionFactor);
        leftLeadConfig.encoder.velocityConversionFactor(conversionFactor / 60.0);
        leftLead.configure(leftLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follow motors: `.follow(leadMotor)` is the whole configuration -- this one
        // call is what makes "command the lead, the follower copies it" happen. No
        // encoder conversion factors here, because this code never reads a follower's
        // encoder (see the field comments above).
        SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
        leftFollowConfig.follow(leftLead);
        leftFollowConfig.idleMode(IdleMode.kCoast);
        leftFollowConfig.smartCurrentLimit(CURRENT_LIMIT);
        leftFollow.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
        rightFollowConfig.follow(rightLead);
        rightFollowConfig.idleMode(IdleMode.kCoast);
        rightFollowConfig.smartCurrentLimit(CURRENT_LIMIT);
        rightFollow.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Start both encoders at exactly 0 meters traveled. Without this, whatever the
        // encoder happened to read when the robot was last powered off would carry
        // over.
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    // ---- Plain hardware actions (no scheduling, no state machines) ----
    //
    // Everything below is intentionally "dumb": each method does exactly one thing to
    // the hardware, right now, and returns. Commands (in commands/) call these
    // repeatedly, in whatever pattern they need, to build actual robot behavior over
    // time.

    /**
     * Tank-drives at the given left/right duty cycles, each in [-1, 1].
     * {@code MathUtil.applyDeadband} zeroes out small values -- without it, a joystick
     * that doesn't return to <i>exactly</i> 0.0 when released would creep the robot.
     *
     * @param left left-side duty cycle, [-1, 1]
     * @param right right-side duty cycle, [-1, 1]
     */
    public void drive(double left, double right) {
        driver.tankDrive(
            MathUtil.applyDeadband(left, JOYSTICK_DEADBAND),
            MathUtil.applyDeadband(right, JOYSTICK_DEADBAND)
        );
    }

    public void stop() {
        driver.stopMotor();
    }

    // ---- Sensors ----
    //
    // These all return SI units (meters, meters/second, degrees for angle -- there's
    // no "imperial degrees") even though nothing else in FRC is metric. That's
    // deliberate: WPILib's own math expects meters, so keeping this subsystem's
    // internal numbers in meters means it can be handed directly to kinematics/PID
    // code without a conversion at every call site. The conversion to feet (for
    // humans) happens in exactly two places: this file's periodic() telemetry, and
    // commands/DriveDistanceCommand.java's constructor, which is the one place a "how
    // many feet" number enters this subsystem from the outside.

    public double getLeftDistanceMeters() {
        return leftEncoder.getPosition();
    }

    public double getRightDistanceMeters() {
        return rightEncoder.getPosition();
    }

    public double getAverageDistanceMeters() {
        return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
    }

    public double getLeftVelocityMetersPerSecond() {
        return leftEncoder.getVelocity();
    }

    public double getRightVelocityMetersPerSecond() {
        return rightEncoder.getVelocity();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getHeadingDegrees() {
        // Negated for CCW-positive: the navX reports clockwise-positive by default,
        // but WPILib's convention (and this codebase's) is counterclockwise-positive,
        // so every raw reading gets flipped right here -- the one place that has to
        // know about that mismatch.
        return -gyro.getAngle();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
    }

    /**
     * The whole robot's forward speed (m/s) and turn rate (rad/s), computed from the
     * two wheel speeds via {@code DifferentialDriveKinematics}. Nothing in this
     * project currently drives from this -- it's here as the other half of what
     * kinematics is for, alongside odometry.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    // ---- Pose (odometry) ----
    //
    // getPose() is DriveTrain's best current estimate of where the robot is on the
    // field, as a Pose2d (X meters, Y meters, heading). It's built entirely on
    // encoder+gyro dead reckoning -- nothing corrects this against reality yet (see
    // the class-level doc comment above for why that's a deliberate next lesson, not
    // a gap in this branch).

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Tells odometry "the robot is actually at this pose right now" -- used once at
     * the start of autonomous once a starting position is known. Resets the encoders
     * too: distance is measured <i>since the last reset</i>, so an old encoder reading
     * and a freshly reset pose would disagree about where "zero" is.
     */
    public void resetPose(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(Rotation2d.fromDegrees(getHeadingDegrees()), 0.0, 0.0, pose);
    }

    /**
     * {@code @Override} tells the compiler "this method is meant to replace a method
     * of the same name/signature on the parent class ({@code SubsystemBase})" -- if a
     * typo meant this didn't actually match anything on the parent (e.g.
     * {@code periodc()}), the compiler would flag it as an error instead of silently
     * creating an unrelated new method that never gets called. {@code periodic()} runs
     * every ~20ms for every subsystem, whether or not a command is currently using it
     * -- this is the right place for "always keep this updated" bookkeeping like
     * telemetry, as opposed to logic that should only happen while a specific command
     * is active (that belongs in that command's {@code execute()}).
     */
    @Override
    public void periodic() {
        // Odometry has to be fed every single loop, not just on the slower telemetry
        // schedule below -- skipping updates would mean missing however much the
        // robot moved during the skipped loops, which is exactly the kind of small,
        // silent error that makes dead-reckoned position drift over a match.
        odometry.update(Rotation2d.fromDegrees(getHeadingDegrees()), getLeftDistanceMeters(), getRightDistanceMeters());
        field.setRobotPose(getPose());

        telemetryLoopCounter++;
        if (telemetryLoopCounter >= TELEMETRY_PERIOD_LOOPS) {
            telemetryLoopCounter = 0;
            // Dashboard values are published in feet and feet/sec -- the units a
            // human glancing at Shuffleboard actually thinks in -- even though
            // everything above this point works in meters.
            SmartDashboard.putNumber("DriveTrain/LeftDistFeet", getLeftDistanceMeters() / METERS_PER_FOOT);
            SmartDashboard.putNumber("DriveTrain/RightDistFeet", getRightDistanceMeters() / METERS_PER_FOOT);
            SmartDashboard.putNumber("DriveTrain/LeftVelocityFPS", getLeftVelocityMetersPerSecond() / METERS_PER_FOOT);
            SmartDashboard.putNumber("DriveTrain/RightVelocityFPS", getRightVelocityMetersPerSecond() / METERS_PER_FOOT);
            SmartDashboard.putNumber("DriveTrain/HeadingDeg", getHeadingDegrees());
            Pose2d pose = getPose();
            SmartDashboard.putNumber("DriveTrain/PoseXFeet", pose.getX() / METERS_PER_FOOT);
            SmartDashboard.putNumber("DriveTrain/PoseYFeet", pose.getY() / METERS_PER_FOOT);
        }
    }
}
