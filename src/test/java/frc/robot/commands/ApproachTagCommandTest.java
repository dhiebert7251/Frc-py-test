package frc.robot.commands;

import static frc.robot.Constants.METERS_PER_FOOT;
import static frc.robot.Constants.VisionConstants.EXAMPLE_TAG_ID;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for ApproachTagCommand's target computation and 3-phase state machine --
 * a Java translation of the Python sibling's {@code test_vision_commands.py}.
 *
 * <p>This test class lives in {@code frc.robot.commands} (where {@code
 * ApproachTagCommand} itself lives), specifically so it can reach that command's
 * package-private {@code phase}/{@code targetPoint}/{@code turnPid}/{@code drivePid}
 * fields directly -- see the comment on those fields in ApproachTagCommand.java for
 * why they aren't simply {@code private} the way most fields in this project are.
 *
 * <p>This project has no physics simulation wired into its Gradle build (see
 * DriveTrainTest.java's class-level comment), so unlike the Python tests, nothing
 * here overwrites a poked encoder or gyro value on its own -- there's no
 * physics-engine-races-the-scheduler gotcha to work around.
 */
class ApproachTagCommandTest {

    private RobotContainer robotContainer;
    private DriveTrain drivetrain;
    private Vision vision;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        drivetrain = robotContainer.drivetrain;
        vision = robotContainer.vision;
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
    void failsForUnknownTag() {
        enable();
        step(0.02);

        ApproachTagCommand command = new ApproachTagCommand(drivetrain, vision, 9999, 3.0);
        command.schedule();
        step(0.02);
        assertFalse(command.isScheduled());
    }

    @Test
    void computesStandoffPoint() {
        Pose2d tagPose = vision.getTagPose(EXAMPLE_TAG_ID).get().toPose2d();

        ApproachTagCommand command = new ApproachTagCommand(drivetrain, vision, EXAMPLE_TAG_ID, 3.0);
        command.initialize();

        assertEquals(ApproachTagCommand.Phase.TURN_TO_TARGET, command.phase);
        double distanceFromTag = command.targetPoint.getDistance(tagPose.getTranslation());
        assertEquals(3.0 * METERS_PER_FOOT, distanceFromTag, 0.01);
    }

    @Test
    void progressesThroughAllPhases() {
        enable();
        step(0.02);

        SimDeviceSim navxSim = new SimDeviceSim("navX-Sensor[4]");
        var yawSim = navxSim.getDouble("Yaw");

        ApproachTagCommand command = new ApproachTagCommand(drivetrain, vision, EXAMPLE_TAG_ID, 3.0);
        command.schedule();
        step(0.02);
        assertTrue(command.isScheduled());
        assertEquals(ApproachTagCommand.Phase.TURN_TO_TARGET, command.phase);

        // Fake having turned exactly to the bearing TURN_TO_TARGET is aiming for.
        // getHeadingDegrees() negates the raw navX yaw (see DriveTrain), so the sign
        // is flipped here to match.
        double bearingDegrees = command.turnPid.getSetpoint();
        yawSim.set(-bearingDegrees);
        step(0.02);
        assertEquals(ApproachTagCommand.Phase.DRIVE_TO_TARGET, command.phase);

        // Fake having driven the exact distance this phase is targeting. This test
        // can't reach DriveTrain's package-private leftEncoder/rightEncoder fields
        // directly the way DriveTrainTest.java does -- it lives in
        // frc.robot.commands (to reach ApproachTagCommand's own package-private
        // phase/targetPoint/turnPid/drivePid state above), not
        // frc.robot.subsystems, and package-private access can't span both at
        // once. See setEncoderPositionsForTest()'s doc comment in DriveTrain.java.
        double targetDistance = command.drivePid.getSetpoint();
        drivetrain.setEncoderPositionsForTest(targetDistance, targetDistance);
        step(0.02);
        assertEquals(ApproachTagCommand.Phase.FACE_TAG, command.phase);

        // Fake having turned to the final heading -- facing the tag directly in this
        // case, since this command was built with no offset.
        double finalHeadingDegrees = command.turnPid.getSetpoint();
        yawSim.set(-finalHeadingDegrees);
        step(0.02);
        assertFalse(command.isScheduled());
    }
}
