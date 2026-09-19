package frc.robot.commands;

import static frc.robot.Constants.GripperConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/** Same shape as IntakeCommand.java, opposite direction -- see that file for the full
 * explanation of this class's constructor and lifecycle methods. */
public class EjectCommand extends Command {

    private final Gripper gripper;

    public EjectCommand(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.setSpeed(EJECT_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }
}
