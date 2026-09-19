package frc.robot.autonomous;

import static frc.robot.Constants.Auto.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.DriveTrain;

/**
 * The two autonomous routines for the teaching-bot proof of concept.
 *
 * <p>Both are built entirely from commands/DriveDistanceCommand.java and
 * commands/TurnToAngleCommand.java -- no PathPlanner, no vision, just wheel encoders
 * and a gyro. All distances/angles are in feet/degrees, taken straight from
 * {@code Constants.Auto}; {@code DriveDistanceCommand} converts feet to meters
 * internally (see its constructor), so nothing in this file ever touches metric units.
 *
 * <p>{@code final class AutoRoutines} with a {@code private AutoRoutines() {}}
 * constructor and only {@code static} methods is Java's usual stand-in for a Python
 * module of free functions: Python's {@code autonomous/routines.py} could just define
 * {@code def drive_forward_only(drivetrain): ...} at the top level of a file, since
 * Python allows functions to exist outside any class. Java requires every method to
 * live inside some class, so a class that is never instantiated (only ever referenced
 * as {@code AutoRoutines.driveForwardOnly(...)}) is the idiomatic way to group a small
 * set of related, state-free functions the way Python would with a module.
 */
public final class AutoRoutines {
    private AutoRoutines() {}

    /** Drives straight forward {@code Auto.DRIVE_FORWARD_ONLY_FEET} feet, then stops. */
    public static Command driveForwardOnly(DriveTrain drivetrain) {
        return new DriveDistanceCommand(drivetrain, DRIVE_FORWARD_ONLY_FEET);
    }

    /**
     * Drives forward, turns, drives forward again:
     * {@code Auto.DRIVE_TURN_DRIVE_FIRST_LEG_FEET} feet -&gt; turn
     * {@code Auto.DRIVE_TURN_DRIVE_TURN_DEGREES} degrees (positive = left) -&gt;
     * {@code Auto.DRIVE_TURN_DRIVE_SECOND_LEG_FEET} feet.
     */
    public static Command driveTurnDrive(DriveTrain drivetrain) {
        return Commands.sequence(
            new DriveDistanceCommand(drivetrain, DRIVE_TURN_DRIVE_FIRST_LEG_FEET),
            new TurnToAngleCommand(drivetrain, DRIVE_TURN_DRIVE_TURN_DEGREES),
            new DriveDistanceCommand(drivetrain, DRIVE_TURN_DRIVE_SECOND_LEG_FEET)
        );
    }
}
