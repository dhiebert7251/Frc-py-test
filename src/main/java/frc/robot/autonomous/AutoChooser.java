package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveTrain;

/**
 * Builds the SmartDashboard autonomous-routine chooser for the teaching-bot proof of
 * concept: two real routines plus a "Do Nothing" default.
 */
public final class AutoChooser {
    private AutoChooser() {}

    private static final String DO_NOTHING_NAME = "Do Nothing";
    private static final String DRIVE_FORWARD_NAME = "Drive Forward 10 ft";
    private static final String DRIVE_TURN_DRIVE_NAME = "Drive 5ft, Turn Left 90, Drive 3ft";

    /**
     * {@code SendableChooser<Command>} -- the angle brackets are a Java GENERIC type
     * parameter: this says "a SendableChooser whose options are all Command objects,"
     * the same idea as Python's {@code SendableChooser} which can hold any type of
     * option but here is used consistently with Command values. Unlike Python (which
     * doesn't check this at all at runtime), Java's compiler uses the {@code <Command>}
     * to guarantee every option ever added to (or read from) this specific chooser
     * really is a Command, catching a wrong-type mistake at compile time instead of
     * only when the mistaken value is actually used.
     */
    public static SendableChooser<Command> build(DriveTrain drivetrain) {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption(DO_NOTHING_NAME, Commands.none());
        chooser.addOption(DRIVE_FORWARD_NAME, AutoRoutines.driveForwardOnly(drivetrain));
        chooser.addOption(DRIVE_TURN_DRIVE_NAME, AutoRoutines.driveTurnDrive(drivetrain));
        SmartDashboard.putData("Auto Chooser", chooser);
        return chooser;
    }
}
