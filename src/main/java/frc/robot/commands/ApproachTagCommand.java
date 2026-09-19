package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.*;
import static frc.robot.Constants.METERS_PER_FOOT;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.Optional;

/**
 * Drives to a point a fixed distance in front of an AprilTag, then turns to face it --
 * optionally rotated some number of degrees off of "directly facing it."
 *
 * <p>This is this project's one genuinely cross-subsystem command: it needs both
 * {@code DriveTrain} (to actually move) and {@code Vision} (to know where a tag is).
 * Earlier branches of this project explicitly said no command needed more than one
 * subsystem -- that was true until this one. Decisions like that are provisional, not
 * permanent: once the situation changes (adding vision), the right response is to
 * update the decision, not defend it. See the README's "Where do commands live?"
 * section for the fuller version of this note.
 *
 * <p>This constructor only calls {@code addRequirements(drivetrain)}, not {@code vision}
 * -- it only ever READS from Vision (asking "where is this tag?"), never commands it,
 * and {@code addRequirements()} exists to prevent two commands from fighting over
 * something they both DRIVE, not something they both read.
 *
 * <p>Three phases, run one after another inside this single command (the same kind of
 * internal state machine as {@code FireCommand}, just with three states instead of
 * two -- a plain {@code enum} here rather than {@code FireCommand}'s boolean flag,
 * since there are more than two states to name):
 *
 * <ol>
 *   <li>{@code TURN_TO_TARGET} -- turn in place to face the point we're driving to.
 *   <li>{@code DRIVE_TO_TARGET} -- drive straight to that point.
 *   <li>{@code FACE_TAG} -- turn in place to the final heading.
 * </ol>
 *
 * <p>Nothing about <i>where</i> to go is known until this command actually starts:
 * {@code initialize()} reads the robot's current estimated pose (from
 * {@code DriveTrain.getPose()}) and the tag's known field position (from
 * {@code Vision.getTagPose()}) to compute the target point and final heading, the
 * same loop this command is scheduled -- unlike {@code DriveDistanceCommand}/
 * {@code TurnToAngleCommand}, whose targets are fixed numbers known when they're
 * constructed.
 *
 * <p>Why one parameterized class instead of two nearly-identical ones (the way
 * {@code RaiseElevatorCommand}/{@code LowerElevatorCommand} are two separate classes
 * for what's conceptually "the same page, in the other direction"): those two
 * commands' bodies are almost entirely different numbers, three lines each. This
 * command's three phases -- computing a target point, turning to a bearing, driving a
 * distance, turning to a final heading -- are the same ~90 lines of state-machine
 * logic for both of this project's example uses; the only thing that differs between
 * "stop 3 feet away, facing the tag" and "stop 5 feet away, then turn 45 degrees off
 * of facing it" is one number, {@code faceOffsetDegrees}. Duplicating that logic to
 * keep two separate classes would risk the two copies drifting out of sync the next
 * time one gets a bugfix -- see RobotContainer.java for the two named instances this
 * project actually binds.
 */
public class ApproachTagCommand extends Command {

    // PACKAGE-PRIVATE (no access modifier), not `private` -- the same deliberate
    // exception documented on DriveTrain's leftEncoder/rightEncoder fields
    // (subsystems/DriveTrain.java). ApproachTagCommandTest.java (in this same
    // frc.robot.commands package) needs to read `phase`/`targetPoint`/`turnPid`/
    // `drivePid` directly to check this command's state-machine progress the same
    // way the Python sibling's test reaches its `_phase`/`_target_point`/
    // `_turn_pid`/`_drive_pid` past Python's naming-convention-only privacy. Java's
    // `private` has no such loophole, so getting the same test access here needs an
    // actual, coarser access level -- package-private is the narrowest one that
    // still works.
    enum Phase {
        TURN_TO_TARGET,
        DRIVE_TO_TARGET,
        FACE_TAG,
        FAILED,
        DONE
    }

    private final DriveTrain drivetrain;
    private final Vision vision;
    private final int tagId;
    private final double standoffMeters;

    // Positive faceOffsetDegrees means "turn right (clockwise) from facing the tag
    // directly" -- the opposite sign from this codebase's CCW-positive convention, so
    // it's negated once, right here, at the boundary where a human-facing "turn
    // right" number enters this command.
    private final double faceOffsetDegrees;

    // Reuses DriveTrain's own turn/drive PID gains rather than introducing a second,
    // separately-tuned set -- this command does the exact same two kinds of motion
    // (turn in place, drive straight) that TurnToAngleCommand/DriveDistanceCommand
    // already tune gains for. Package-private for the same test-access reason as
    // `Phase` above.
    final PIDController turnPid;
    final PIDController drivePid;

    Phase phase = Phase.DONE;
    Translation2d targetPoint = new Translation2d();
    private double finalHeadingDegrees = 0.0;
    private double driveStartDistanceMeters = 0.0;

    /**
     * @param drivetrain the subsystem this command drives -- passed to
     *     {@code addRequirements()} so the scheduler knows this command owns it
     * @param vision read-only: used to look up {@code tagId}'s known field position
     * @param tagId which AprilTag to approach
     * @param standoffFeet how far from the tag to stop, in feet, measured along the
     *     tag's own facing direction
     * @param faceOffsetDegrees how many degrees off of "directly facing the tag" the
     *     final heading should be, positive = clockwise (right)
     */
    public ApproachTagCommand(
        DriveTrain drivetrain, Vision vision, int tagId, double standoffFeet, double faceOffsetDegrees
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.tagId = tagId;
        this.standoffMeters = standoffFeet * METERS_PER_FOOT;
        this.faceOffsetDegrees = -faceOffsetDegrees;
        addRequirements(drivetrain);

        turnPid = new PIDController(TURN_KP, TURN_KI, TURN_KD);
        turnPid.enableContinuousInput(-180, 180);
        turnPid.setTolerance(TURN_TOLERANCE_DEGREES);

        drivePid = new PIDController(DRIVE_DISTANCE_KP, DRIVE_DISTANCE_KI, DRIVE_DISTANCE_KD);
        drivePid.setTolerance(DRIVE_DISTANCE_TOLERANCE_METERS);
    }

    /**
     * Overload matching this project's two actual bindings (both use
     * {@code faceOffsetDegrees = 0.0}, "stop facing the tag directly") -- Java has no
     * default-parameter-value syntax the way Python's {@code face_offset_degrees:
     * float = 0.0} does, so an overload is the idiomatic Java equivalent: a second,
     * shorter constructor that just calls the full one with a fixed value for the
     * argument callers usually don't need to give.
     */
    public ApproachTagCommand(DriveTrain drivetrain, Vision vision, int tagId, double standoffFeet) {
        this(drivetrain, vision, tagId, standoffFeet, 0.0);
    }

    @Override
    public void initialize() {
        Optional<Pose3d> tagPose = vision.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            // An unknown tag ID -- nothing to approach. Ending immediately (isFinished()
            // checks for this phase) is safer than guessing; see the README's
            // "Assumptions that need bench verification" for why this is worth a
            // dashboard warning in a real robot, not just a silently-do-nothing command.
            phase = Phase.FAILED;
            return;
        }

        Pose2d tagPose2d = tagPose.get().toPose2d();
        // A tag's pose "faces" outward, away from the tag surface -- the standoff
        // point is that far along the tag's own facing direction, and facing the tag
        // from there means pointing the opposite way (180 degrees from how the tag
        // itself faces).
        Rotation2d tagFacing = tagPose2d.getRotation();
        targetPoint = tagPose2d.getTranslation()
            .plus(new Translation2d(standoffMeters, 0.0).rotateBy(tagFacing));
        finalHeadingDegrees = tagFacing.plus(Rotation2d.fromDegrees(180.0 + faceOffsetDegrees)).getDegrees();

        beginTurnToTarget();
    }

    private void beginTurnToTarget() {
        Translation2d currentTranslation = drivetrain.getPose().getTranslation();
        Translation2d delta = targetPoint.minus(currentTranslation);
        double bearingDegrees = Math.toDegrees(Math.atan2(delta.getY(), delta.getX()));

        turnPid.reset();
        turnPid.setSetpoint(bearingDegrees);
        phase = Phase.TURN_TO_TARGET;
    }

    private void beginDriveToTarget() {
        // Distance-to-go is recomputed here, once, from the pose the robot actually
        // ended the turn at -- not assumed from the turn's own target, since a real
        // turn won't land exactly on its setpoint.
        double distanceMeters = drivetrain.getPose().getTranslation().getDistance(targetPoint);
        driveStartDistanceMeters = drivetrain.getAverageDistanceMeters();

        drivePid.reset();
        drivePid.setSetpoint(distanceMeters);
        phase = Phase.DRIVE_TO_TARGET;
    }

    private void beginFaceTag() {
        turnPid.reset();
        turnPid.setSetpoint(finalHeadingDegrees);
        phase = Phase.FACE_TAG;
    }

    @Override
    public void execute() {
        switch (phase) {
            case TURN_TO_TARGET -> {
                double output = turnPid.calculate(drivetrain.getHeadingDegrees());
                drivetrain.drive(-output, output);
                if (turnPid.atSetpoint()) {
                    beginDriveToTarget();
                }
            }
            case DRIVE_TO_TARGET -> {
                // Same relative-distance measurement DriveDistanceCommand uses -- see
                // that command's initialize() doc comment for why this can't just
                // reset the encoders to zero instead.
                double distanceThisPhase = drivetrain.getAverageDistanceMeters() - driveStartDistanceMeters;
                double output = drivePid.calculate(distanceThisPhase);
                output = Math.max(-DRIVE_DISTANCE_MAX_OUTPUT, Math.min(DRIVE_DISTANCE_MAX_OUTPUT, output));
                drivetrain.drive(output, output);
                if (drivePid.atSetpoint()) {
                    beginFaceTag();
                }
            }
            case FACE_TAG -> {
                double output = turnPid.calculate(drivetrain.getHeadingDegrees());
                drivetrain.drive(-output, output);
            }
            case FAILED, DONE -> {
                // Nothing to do -- isFinished() ends the command.
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (phase == Phase.FAILED || phase == Phase.DONE) {
            return true;
        }
        return phase == Phase.FACE_TAG && turnPid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
