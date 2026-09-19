package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionMeasurement;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision subsystem -- one PhotonVision camera doing AprilTag pose estimation.
 *
 * <p>Teaching-bot proof of concept. Simpler than the competition bot's Vision: one
 * forward-facing camera instead of two (front + rear), so there's no "which camera's
 * measurement do we trust more this loop" arbitration to carry -- just "does the one
 * camera have a usable measurement right now."
 *
 * <p>Like every other subsystem in this project, this one only exposes plain
 * actions/queries ({@code getBestVisionMeasurementIfFresh()}, {@code getTagPose()},
 * {@code isAnyVisionAvailable()}) -- {@code DriveTrain} calls these to fuse a vision
 * fix into its pose estimator (see subsystems/DriveTrain.java), and
 * commands/ApproachTagCommand.java calls {@code getTagPose()} to find out where a
 * specific tag actually is on the field.
 *
 * <p><b>A deliberate difference from the real competition port's own {@code Vision}
 * subsystem, worth flagging explicitly:</b> that file constructs
 * {@code PhotonPoseEstimator} with an explicit
 * {@code PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR} argument and calls a single
 * {@code poseEstimator.update(result)} per camera. This file instead constructs
 * {@code PhotonPoseEstimator} with just the field layout and the camera transform
 * (no strategy argument), and calls the strategy-specific
 * {@code estimateCoprocMultiTagPose(result)} method directly, falling back to
 * {@code estimateLowestAmbiguityPose(result)} when multi-tag PNP data isn't
 * available. Both patterns exist in PhotonLib for the 2026 season -- this file's
 * choice was the one actually verified end to end in this project's Python sibling
 * (constructed, called, and passed 4/4 tests against the real installed
 * {@code photonlibpy==2026.3.4} package in the session that wrote it), so it's the
 * pattern this Java file mirrors, even though it differs from what the competition
 * repo's own (unverified against this exact vendor version) code happens to use. See
 * the README's Verification status section for the full reasoning.
 */
public class Vision extends SubsystemBase {

    // AprilTagFieldLayout is a known, fixed map of tag ID -> field position -- it
    // doesn't need the camera to see anything. Loaded once here so getTagPose() below
    // is a plain lookup, usable even for a tag the camera has never actually detected.
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_CAMERA);

    private double lastResultTimestamp = -1.0;
    private double lastFreshFrameFpgaTimestamp = -1.0;

    // Cached once per loop in periodic(), read by every caller this cycle -- avoids
    // recomputing the same pose estimate multiple times if more than one thing asks
    // for it in the same 20ms. `Optional<VisionMeasurement>` (not a nullable
    // `VisionMeasurement` that might be `null`) is the idiomatic Java way to say "this
    // might not have a value" -- callers are pushed toward handling the empty case
    // explicitly (`.isPresent()`, `.map(...)`, `.orElse(...)`) instead of risking a
    // `NullPointerException` from forgetting to check for `null`.
    private Optional<VisionMeasurement> cachedMeasurement = Optional.empty();

    private int telemetryLoopCounter = 0;

    /**
     * Looks up a tag's known field position, independent of whether the camera
     * currently sees it. {@code Optional<Pose3d>} is empty for an unknown tag ID --
     * {@code AprilTagFieldLayout.getTagPose(int)} itself already returns an
     * {@code Optional}, so this method just passes that straight through.
     */
    public Optional<Pose3d> getTagPose(int tagId) {
        return fieldLayout.getTagPose(tagId);
    }

    /**
     * The best measurement cached this loop, or empty if there wasn't one, or it's
     * older than {@code VisionConstants.MAX_VISION_AGE_SECONDS}.
     */
    public Optional<VisionMeasurement> getBestVisionMeasurementIfFresh() {
        return cachedMeasurement.filter(this::isMeasurementFresh);
    }

    private boolean isMeasurementFresh(VisionMeasurement measurement) {
        return (Timer.getFPGATimestamp() - measurement.timestampSeconds()) <= MAX_VISION_AGE_SECONDS;
    }

    /**
     * True if the camera has produced a fresh frame recently -- not the same as "a
     * tag is currently visible": a connected camera pointed at a blank wall is
     * available but sees nothing.
     */
    public boolean isAnyVisionAvailable() {
        return lastFreshFrameFpgaTimestamp >= 0
            && (Timer.getFPGATimestamp() - lastFreshFrameFpgaTimestamp) <= 0.5;
    }

    private Optional<VisionMeasurement> computeMeasurement(PhotonPipelineResult result) {
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
        if (estimatedPose.isEmpty()) {
            estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
        }
        if (estimatedPose.isEmpty()) {
            return Optional.empty();
        }

        EstimatedRobotPose pose = estimatedPose.get();
        Pose2d pose2d = pose.estimatedPose.toPose2d();

        double fieldLength = fieldLayout.getFieldLength();
        double fieldWidth = fieldLayout.getFieldWidth();
        if (pose2d.getX() < 0 || pose2d.getX() > fieldLength || pose2d.getY() < 0 || pose2d.getY() > fieldWidth) {
            return Optional.empty(); // a pose off the field is never real
        }

        List<PhotonTrackedTarget> targets = result.getTargets();
        if (targets.size() == 1 && result.getBestTarget().getPoseAmbiguity() > MAX_AMBIGUITY) {
            return Optional.empty();
        }

        int numTags = pose.targetsUsed.size();
        Matrix<N3, N1> stdDevs;
        if (numTags >= MIN_TAGS_FOR_MULTI_TAG) {
            stdDevs = MULTI_TAG_STDDEVS;
        } else {
            double averageDistance = averageTagDistance(targets, pose2d);
            if (averageDistance > MAX_TAG_DISTANCE_METERS) {
                return Optional.empty();
            }
            stdDevs = averageDistance < 2.0 ? SINGLE_TAG_CLOSE_STDDEVS : SINGLE_TAG_FAR_STDDEVS;
        }

        return Optional.of(new VisionMeasurement(pose2d, pose.timestampSeconds, stdDevs, numTags));
    }

    private double averageTagDistance(List<PhotonTrackedTarget> targets, Pose2d robotPose) {
        double totalDistance = 0.0;
        int validTagCount = 0;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                totalDistance += robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
                validTagCount++;
            }
        }
        return validTagCount == 0 ? Double.POSITIVE_INFINITY : totalDistance / validTagCount;
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        double timestamp = result.getTimestampSeconds();

        // A camera that has never sent a real result reports a placeholder timestamp
        // near zero (verified directly against the Python sibling's installed
        // photonlibpy package: -1e-06, not the -1.0 this class starts
        // lastResultTimestamp at) -- the `timestamp > 0` check is what keeps that
        // placeholder from being mistaken for an actual fresh frame the very first
        // time periodic() runs.
        if (timestamp > 0 && timestamp > lastResultTimestamp) {
            lastResultTimestamp = timestamp;
            lastFreshFrameFpgaTimestamp = Timer.getFPGATimestamp();
            cachedMeasurement = computeMeasurement(result);
        }

        telemetryLoopCounter++;
        if (telemetryLoopCounter >= TELEMETRY_PERIOD_LOOPS) {
            telemetryLoopCounter = 0;
            SmartDashboard.putBoolean("Vision/Available", isAnyVisionAvailable());
            SmartDashboard.putBoolean("Vision/HasMeasurement", cachedMeasurement.isPresent());
            cachedMeasurement.ifPresent(
                measurement -> SmartDashboard.putNumber("Vision/NumTagsUsed", measurement.numTagsUsed())
            );
        }
    }
}
