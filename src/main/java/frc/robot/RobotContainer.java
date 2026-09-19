package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.TriggerConstants.FIRE_TIMEOUT_SECONDS;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoChooser;
import frc.robot.commands.EjectCommand;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LowerElevatorCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.SpinUpShooterCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trigger;

/**
 * RobotContainer for the teaching-bot proof of concept.
 *
 * <p>Wires the five subsystems together, sets teleop default commands and button
 * bindings, and builds the autonomous chooser. See README.md for the full
 * controller-binding table and subsystem/command maps -- this file is meant to be read
 * start-to-finish as the map of the whole robot.
 */
public class RobotContainer {

    // `public final` fields (not `private`): unlike every subsystem/command field seen
    // so far, these are deliberately visible outside this class -- Robot.java (and, in
    // a JUnit test, a test class) needs to reach `robotContainer.drivetrain` directly
    // to poke at simulated hardware. `final` still means each is assigned exactly once,
    // right here in the constructor below.
    public final DriveTrain drivetrain = new DriveTrain();
    public final Shooter shooter = new Shooter();
    public final Trigger trigger = new Trigger();
    public final Elevator elevator = new Elevator();
    public final Gripper gripper = new Gripper();

    private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    /**
     * {@code CommandXboxController} is the current, non-deprecated way to bind
     * buttons to commands for an Xbox-style controller as of WPILib 2026 -- the same
     * class name in both the Java and Python bindings (Python's is a thin wrapper
     * around this very Java/C++ implementation, which is why the class names and
     * method names on it already match almost exactly between the two languages,
     * unlike REVLib/Phoenix6/Studica's separately-written Java and Python APIs).
     */
    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();

        autoChooser = AutoChooser.build(drivetrain);
    }

    private void configureDefaultCommands() {
        // A subsystem's default command runs whenever no other command needs that
        // subsystem -- here, that means "whenever the driver isn't running an
        // autonomous/other DriveTrain command, tank drive from the sticks."
        drivetrain.setDefaultCommand(new TeleopDriveCommand(drivetrain, driverController));
    }

    /**
     * Configure button-to-command bindings.
     *
     * <p>Driver (port 0) -- drive only:
     * <ul>
     *   <li>Left Y / Right Y = tank drive</li>
     *   <li>Back = reset gyro heading to 0 (do this before autonomous!)</li>
     * </ul>
     *
     * <p>Operator (port 1) -- everything else:
     * <ul>
     *   <li>A = toggle shooter spin-up</li>
     *   <li>B = fire trigger</li>
     *   <li>X = gripper intake while held</li>
     *   <li>Y = gripper eject while held</li>
     *   <li>Right Bumper = raise elevator while held</li>
     *   <li>Left Bumper = lower elevator while held</li>
     * </ul>
     *
     * <p>Every binding below schedules a named Command class -- none of them build a
     * command inline with a lambda, including the one-shot gyro reset (see
     * ResetGyroCommand.java's docstring for why a one-shot action still gets a full
     * class in this project).
     */
    private void configureBindings() {
        driverController.back().onTrue(new ResetGyroCommand(drivetrain));

        operatorController.a().toggleOnTrue(new SpinUpShooterCommand(shooter));

        // FireCommand has no timeout of its own -- `.withTimeout()` is a decorator
        // that wraps ANY command (see FireCommand.java's docstring), applied here at
        // the one place this command is actually bound to a button, using the safety
        // timeout defined in TriggerConstants.
        operatorController.b().onTrue(new FireCommand(trigger).withTimeout(FIRE_TIMEOUT_SECONDS));

        operatorController.x().whileTrue(new IntakeCommand(gripper));
        operatorController.y().whileTrue(new EjectCommand(gripper));
        operatorController.rightBumper().whileTrue(new RaiseElevatorCommand(elevator));
        operatorController.leftBumper().whileTrue(new LowerElevatorCommand(elevator));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
