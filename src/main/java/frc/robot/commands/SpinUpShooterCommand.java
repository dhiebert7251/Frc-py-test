package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.TARGET_RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Only one command for Shooter: spin the flywheel up to a fixed target speed, and hold
 * it there until interrupted. Notice this class has no {@code execute()} at all --
 * that's not an omission. Phoenix 6's velocity control is closed-loop on the TalonFX
 * itself (see subsystems/Shooter.java), so this command only has to say "go to this
 * speed" once ({@code initialize()}) and "stop" once ({@code end()}) -- there's nothing
 * to redo every 20ms loop the way DriveTrain's open-loop, duty-cycle commands need.
 */
public class SpinUpShooterCommand extends Command {

    private final Shooter shooter;

    public SpinUpShooterCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetRpm(TARGET_RPM);
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted (the operator presses the toggle button again -- see
        // RobotContainer.java's toggleOnTrue binding).
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
