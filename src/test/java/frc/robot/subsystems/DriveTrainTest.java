package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnToAngleCommand;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for DriveTrain's encoder-distance bookkeeping and its PID autonomous
 * commands -- a Java translation of the Python sibling's {@code test_drivetrain.py}.
 *
 * <p>This test class lives in {@code frc.robot.subsystems} (not {@code frc.robot},
 * where most of this project's classes live) specifically so it can reach
 * DriveTrain's package-private {@code leftEncoder}/{@code rightEncoder} fields
 * directly -- see the comment on those fields in DriveTrain.java for why they aren't
 * simply {@code private} the way every other hardware field in this project is.
 *
 * <p>This project has no physics simulation wired into its Gradle build the way the
 * Python sibling's {@code physics.py} is wired into {@code robotpy sim} (writing an
 * equivalent {@code simulationPeriodic()} model is a good exercise, not done here) --
 * so unlike the Python tests, nothing here overwrites a poked encoder or gyro value on
 * its own. That actually makes these tests SIMPLER than their Python counterparts:
 * there's no physics-engine-races-the-scheduler gotcha to work around, since nothing
 * is racing.
 */
class DriveTrainTest {

    private RobotContainer robotContainer;
    private DriveTrain drivetrain;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        drivetrain = robotContainer.drivetrain;
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    private void enable() {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
    }

    private void step(double seconds) {
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    @Test
    void averageDistanceStartsAtZero() {
        assertEquals(0.0, drivetrain.getAverageDistanceMeters());
    }

    @Test
    void resetEncodersZeroesDistance() {
        drivetrain.resetEncoders();
        assertEquals(0.0, drivetrain.getLeftDistanceMeters());
        assertEquals(0.0, drivetrain.getRightDistanceMeters());
    }

    @Test
    void driveDistanceCommandFinishesOnceTargetReached() {
        // A command can only be scheduled while the robot is enabled (the default
        // runsWhenDisabled() is false), so enable it first.
        enable();
        step(0.02);

        DriveDistanceCommand command = new DriveDistanceCommand(drivetrain, 1.0); // 1 foot
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled()); // nowhere near the target yet

        // Simulate the robot having driven all the way there by writing the target
        // distance straight onto both encoders -- same RelativeEncoder.setPosition()
        // call configureMotors() itself uses to zero them at startup, just called with
        // a nonzero value here. This is the direct Java equivalent of the Python
        // test's `drivetrain._left_encoder.setPosition(target_meters)`.
        double targetMeters = 1.0 * Constants.METERS_PER_FOOT;
        drivetrain.leftEncoder.setPosition(targetMeters);
        drivetrain.rightEncoder.setPosition(targetMeters);

        step(0.1);
        assertFalse(command.isScheduled());
    }

    @Test
    void turnToAngleCommandFinishesOnceHeadingReached() {
        enable();
        step(0.02);

        // navX simulation is reached differently from the SparkMax encoders above:
        // Studica's AHRS exposes its simulated yaw through WPILib's generic
        // SimDeviceSim registry under the name "navX-Sensor[4]" rather than through a
        // method on the AHRS object itself -- the same mechanism (and the same
        // device name) the real competition port's own physics simulation uses, and
        // the same one the Python sibling's test pokes via
        // `wpilib.simulation.SimDeviceSim("navX-Sensor[4]")`.
        SimDeviceSim navxSim = new SimDeviceSim("navX-Sensor[4]");

        TurnToAngleCommand command = new TurnToAngleCommand(drivetrain, 90.0);
        command.schedule();
        step(0.1);
        assertTrue(command.isScheduled());

        // getHeadingDegrees() negates the raw navX yaw (see DriveTrain), so -90 raw
        // yaw simulates having reached +90 degrees heading.
        navxSim.getDouble("Yaw").set(-90.0);
        step(0.1);
        assertFalse(command.isScheduled());
    }

    @Test
    void autoRoutinesBuildWithoutError() {
        assertNotNull(AutoRoutines.driveForwardOnly(drivetrain));
        assertNotNull(AutoRoutines.driveTurnDrive(drivetrain));
        assertTrue(Constants.Auto.DRIVE_FORWARD_ONLY_FEET > 0);
    }
}
