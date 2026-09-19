package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.TimedCommandRobot;

/**
 * Entry point for the teaching-bot proof of concept.
 *
 * <p>{@code TimedCommandRobot} does two things a plain {@code TimedRobot} would leave
 * to you: it calls {@code CommandScheduler.getInstance().run()} every loop
 * automatically (Python's {@code commands2.TimedCommandRobot} does the same), and it
 * still gives you the familiar {@code robotInit()}/{@code autonomousInit()}/
 * {@code teleopInit()}/... callback methods to override.
 *
 * <p>No AdvantageKit, no vision-specific logging (unlike the real competition port's
 * {@code Robot.java}, which extends AdvantageKit's {@code LoggedRobot}) -- just
 * {@link DataLogManager} for on-disk + NetworkTables logging, matching the Python
 * teaching-bot's {@code robot.py} exactly.
 */
public class Robot extends TimedCommandRobot {
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
