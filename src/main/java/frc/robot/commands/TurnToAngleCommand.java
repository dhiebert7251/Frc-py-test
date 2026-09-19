package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/**
 * PID-turns to an absolute heading, given in degrees (CCW-positive, 0 = whatever
 * heading the navX gyro was last reset to). Positive = turn left.
 */
public class TurnToAngleCommand extends Command {

    private final DriveTrain drivetrain;
    private final double targetDegrees;
    private final PIDController pid;

    public TurnToAngleCommand(DriveTrain drivetrain, double targetDegrees) {
        this.drivetrain = drivetrain;
        this.targetDegrees = targetDegrees;
        addRequirements(drivetrain);

        pid = new PIDController(TURN_KP, TURN_KI, TURN_KD);
        // A heading wraps around at +/-180 degrees. Without this next line, turning
        // from 179 degrees to -179 degrees (really just a 2-degree turn) would look to
        // a naive PID controller like a 358-degree turn the "long way around."
        // enableContinuousInput tells it to treat the range as a circle instead of a
        // straight line.
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(TURN_TOLERANCE_DEGREES);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(targetDegrees);
    }

    @Override
    public void execute() {
        double output = pid.calculate(drivetrain.getHeadingDegrees());
        // Turning in place: equal and opposite power to each side. Positive output
        // steers left, so the left side goes backward while the right side goes
        // forward.
        drivetrain.drive(-output, output);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
