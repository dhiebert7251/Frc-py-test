package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.LOWER_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/** See RaiseElevatorCommand.java for the shared reasoning behind this pair. */
public class LowerElevatorCommand extends Command {

    private final Elevator elevator;

    public LowerElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (elevator.isAtBottom()) {
            elevator.stop();
        } else {
            elevator.setSpeed(LOWER_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
