package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/**
 * Resets the navX gyro's heading to 0 -- bound to the driver's Back button. Do this
 * before every autonomous run, with the robot pointed the way it should be for that
 * run's "0 degrees."
 *
 * <p>A one-shot action still gets a full class here, on purpose (see the README's
 * "Where do commands live?" section): {@code initialize()} does the actual work, and
 * {@code isFinished()} returns {@code true} immediately so the command scheduler ends
 * it the very next loop after that -- there's no {@code execute()} at all, since
 * there's nothing to repeat.
 */
public class ResetGyroCommand extends Command {

    private final DriveTrain drivetrain;

    /**
     * Same constructor pattern as TeleopDriveCommand.java, just with one parameter
     * instead of two, and no explicit {@code super(...)} call needed (see that file's
     * constructor for the full explanation of both).
     */
    public ResetGyroCommand(DriveTrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
