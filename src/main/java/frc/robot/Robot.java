package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Entry point for the teaching-bot proof of concept.
 *
 * <p><b>Corrected during a post-hoc code-review pass:</b> this class used to extend a
 * class called {@code TimedCommandRobot}, imported from
 * {@code edu.wpi.first.wpilibj2.command}. That class does not exist in Java WPILib --
 * it's a RobotPy-only convenience ({@code commands2.TimedCommandRobot}, in the Python
 * bindings) that automatically calls {@code CommandScheduler.getInstance().run()}
 * every loop; there's no Java equivalent that does the same thing implicitly. The
 * mistake would have failed to compile with "cannot find symbol," and even patched to
 * compile, nothing would have called the scheduler at all -- autonomous and teleop
 * commands would never actually run.
 *
 * <p>The fix: extend the real {@link TimedRobot} directly, and call the scheduler
 * explicitly, once, from an overridden {@code robotPeriodic()} below -- exactly what
 * every WPILib Java command-based robot does, including this team's own real
 * competition port (whose {@code Robot.java} extends AdvantageKit's
 * {@code LoggedRobot}, itself a {@code TimedRobot} subclass, and does this same
 * explicit call).
 *
 * <p>No AdvantageKit, no vision-specific logging (unlike the real competition port's
 * {@code Robot.java}) -- just {@link DataLogManager} for on-disk + NetworkTables
 * logging, matching the Python teaching-bot's {@code robot.py} exactly.
 */
public class Robot extends TimedRobot {
    // `Command` (an interface/abstract class) is the TYPE; `m_autonomousCommand` can
    // hold `null` (no autonomous command selected) or any object that implements
    // Command. Python's equivalent used `Optional[Command] = None` as a type hint --
    // Java has no separate "nullable" annotation built into the language the way
    // Python's `Optional[X]` is; ANY non-primitive Java type (anything that isn't
    // `int`/`double`/`boolean`/etc.) can already hold `null`, so `Command` alone is the
    // whole type, and `null` is a value it can take on without any extra syntax.
    private Command m_autonomousCommand;

    // `RobotContainer` is a type WE wrote (see RobotContainer.java) -- Java doesn't
    // distinguish "a class from the standard library" from "a class from this project"
    // in its syntax at all; both are used exactly the same way once imported.
    public RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for
     * any initialization code.
     */
    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        m_robotContainer = new RobotContainer();
    }

    /**
     * Runs every ~20ms, no matter what mode the robot is in -- this is the one place
     * {@code CommandScheduler.getInstance().run()} has to be called from. It's what
     * actually polls button bindings, starts newly-scheduled commands, runs already-
     * scheduled commands' {@code execute()}, checks {@code isFinished()}, and calls
     * every registered subsystem's {@code periodic()}. Without this override, nothing
     * in the command-based framework -- not a single command, not a single
     * subsystem's {@code periodic()} -- would ever run.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /** This autonomous runs the autonomous command selected by {@link RobotContainer}. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure autonomous stops running when teleop starts. If you want
        // autonomous to continue until interrupted by another command, remove this.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
}
