package frc.robot.commands;

import static frc.robot.Constants.GripperConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/**
 * The simplest command in this project: hold a fixed speed while bound (whileTrue in
 * RobotContainer.java), stop when released. No sensors, no looping math -- a good
 * first command file to read.
 */
public class IntakeCommand extends Command {

    private final Gripper gripper;

    /**
     * See TeleopDriveCommand.java's constructor for the full explanation of every
     * piece of syntax here: {@code public}, the {@code Gripper gripper} parameter's
     * mandatory type, the implicit no-arg {@code super()} call Java inserts since none
     * is written, and why a constructor is written with no return type at all (not
     * even {@code void}).
     */
    public IntakeCommand(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.setSpeed(INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        // Always false: this command is meant to be bound with whileTrue, so it only
        // stops when the button is released, which the scheduler handles by calling
        // end() below instead.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // See DriveDistanceCommand.java's end() for what the `interrupted` parameter
        // means -- this command doesn't need to look at its value, but every command's
        // end() method receives it regardless.
        gripper.stop();
    }
}
