package frc.robot;

import static frc.robot.Constants.VisionConstants.EXAMPLE_TAG_ID;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the Vision subsystem's tag-pose lookup and availability reporting --
 * a Java translation of the Python sibling's {@code test_vision.py}.
 *
 * <p>There's no real camera or PhotonVision coprocessor in this test environment, so
 * these tests only cover what doesn't require actual camera data: the AprilTag field
 * layout lookup (a fixed table, not something the camera has to see), and the
 * "nothing has arrived yet" default state. A disconnected {@code PhotonCamera}
 * reports a placeholder result with a near-zero timestamp rather than throwing or
 * returning {@code null}, which is exactly the quirk {@code Vision.periodic()}'s
 * {@code timestamp > 0} check exists to not be fooled by -- see that file for the
 * full explanation. Vision needs no package-private access changes to test, unlike
 * DriveTrain -- everything this test needs is already public.
 */
class VisionTest {

    private RobotContainer robotContainer;
    private Vision vision;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        vision = robotContainer.vision;
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    @Test
    void getTagPoseReturnsKnownTag() {
        assertTrue(vision.getTagPose(EXAMPLE_TAG_ID).isPresent());
    }

    @Test
    void getTagPoseReturnsEmptyForUnknownTag() {
        assertTrue(vision.getTagPose(9999).isEmpty());
    }

    @Test
    void noMeasurementWithoutCameraData() {
        CommandScheduler.getInstance().run();
        assertTrue(vision.getBestVisionMeasurementIfFresh().isEmpty());
    }

    @Test
    void visionNotAvailableWithoutCameraData() {
        CommandScheduler.getInstance().run();
        assertFalse(vision.isAnyVisionAvailable());
    }
}
