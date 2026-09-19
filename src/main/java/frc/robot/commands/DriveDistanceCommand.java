package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.*;
import static frc.robot.Constants.METERS_PER_FOOT;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/**
 * Drives straight to a target distance, given in FEET, using a PID loop on the average
 * of the two drive encoders.
 *
 * <p>Why PID instead of "drive at a fixed speed until the encoder says you're far
 * enough" (this command's first version): a fixed speed either overshoots -- the
 * motors are still at full speed right up to the exact instant the target is crossed,
 * so the robot coasts/slams past it -- or forces someone to guess a "stop early to
 * leave room for coasting" fudge factor. A PID controller instead recalculates "how
 * hard should I push" every loop from how much distance is left, so the commanded
 * speed naturally tapers off as the target gets close instead of being all full power
 * then all stop.
 */
public class DriveDistanceCommand extends Command {

    private final DriveTrain drivetrain;
    // `double targetDistanceMeters` and `PIDController pid` are both computed/built in
    // the constructor and then never reassigned, so both could be declared `final`
    // exactly like `drivetrain` above -- they're written without `final` here only
    // because the constructor computes/builds them from a parameter rather than
    // receiving them directly, which is a distinction of readability preference, not
    // one Java's compiler cares about.
    private final double targetDistanceMeters;
    private final PIDController pid;

    // Not `final`: set fresh every time this command starts, in initialize() below --
    // see that method's doc comment for why this field exists at all.
    private double startDistanceMeters;

    /**
     * {@code double distanceFeet} -- the same {@code name: type} idea as every other
     * parameter in this project, just using one of Java's built-in primitive types
     * ({@code double}, a 64-bit decimal number) instead of a class like
     * {@code DriveTrain}. Feet-to-meters conversion happens exactly once, right here,
     * at the boundary where a "human" feet value enters this command -- see
     * {@code Constants.METERS_PER_FOOT}.
     */
    public DriveDistanceCommand(DriveTrain drivetrain, double distanceFeet) {
        this.drivetrain = drivetrain;
        this.targetDistanceMeters = distanceFeet * METERS_PER_FOOT;
        addRequirements(drivetrain);

        pid = new PIDController(DRIVE_DISTANCE_KP, DRIVE_DISTANCE_KI, DRIVE_DISTANCE_KD);
        pid.setTolerance(DRIVE_DISTANCE_TOLERANCE_METERS);
    }

    @Override
    public void initialize() {
        // initialize() runs exactly once, the instant this command is scheduled (not
        // when it's constructed in the constructor, which for autonomous commands
        // happens once at RobotContainer startup, possibly minutes before the command
        // actually runs).
        //
        // This records the CURRENT encoder reading as a baseline rather than calling
        // drivetrain.resetEncoders() to zero it -- a tempting shortcut this command
        // used to take, until DriveTrain grew a pose estimator (see
        // subsystems/DriveTrain.java) that reads these same encoders every loop.
        // Odometry measures distance *since its last reset*, so zeroing the encoders
        // out from under it looks exactly like the robot teleporting back near the
        // origin -- a real bug this command caused for every leg after the first in
        // the drive-turn-drive autonomous routine, since fixed by measuring a
        // relative distance instead of an absolute one.
        startDistanceMeters = drivetrain.getAverageDistanceMeters();
        pid.reset();
        pid.setSetpoint(targetDistanceMeters);
    }

    @Override
    public void execute() {
        // PIDController.calculate(measurement) returns "how hard to push" based on the
        // error between `measurement` and the setpoint given in initialize(). It's
        // clamped to +/-DRIVE_DISTANCE_MAX_OUTPUT as a second, independent safety
        // margin on top of tuning KP conservatively -- a large distance error
        // (commanding 10 feet from a dead stop) should never be able to demand more
        // than that fraction of full power.
        double distanceThisLeg = drivetrain.getAverageDistanceMeters() - startDistanceMeters;
        double output = pid.calculate(distanceThisLeg);
        output = Math.max(-DRIVE_DISTANCE_MAX_OUTPUT, Math.min(DRIVE_DISTANCE_MAX_OUTPUT, output));
        drivetrain.drive(output, output);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    /**
     * {@code boolean interrupted} -- the CommandScheduler fills this parameter in for
     * you: {@code true} if this command got cut off early (the driver grabbed the
     * joystick mid-autonomous, the match ended, the robot got disabled), {@code false}
     * if {@code isFinished()} returned {@code true} on its own. {@code end()} runs
     * exactly once either way, which is exactly why stopping the motors belongs here
     * rather than only handling the "finished normally" path -- this command doesn't
     * need to tell the two cases apart, but {@code end()} always receives this
     * parameter regardless of whether a command reads it.
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
