package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.RAISE_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * RaiseElevatorCommand and LowerElevatorCommand (in LowerElevatorCommand.java) are
 * near-identical on purpose: both are meant to be bound with whileTrue (see
 * RobotContainer.java), so {@code isFinished()} always returns {@code false} and the
 * command only stops when the button is released (which interrupts it, calling
 * {@code end()}) or when {@code execute()} itself detects a limit switch and calls
 * {@code stop()}. That second check matters even though the command is also
 * "supposed" to stop when the button is released: it protects the mechanism the
 * instant it reaches a limit, without waiting on the operator to notice and let go.
 */
public class RaiseElevatorCommand extends Command {

    private final Elevator elevator;

    public RaiseElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (elevator.isAtTop()) {
            elevator.stop();
        } else {
            elevator.setSpeed(RAISE_SPEED);
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
