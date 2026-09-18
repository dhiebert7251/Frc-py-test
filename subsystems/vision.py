"""Vision subsystem -- one PhotonVision camera doing AprilTag pose estimation.

Teaching-bot proof of concept. Simpler than the competition bot's Vision:
one forward-facing camera instead of two (front + rear), so there's no
"which camera's measurement do we trust more this loop" arbitration to
carry -- just "does the one camera have a usable measurement right now."

Like every other subsystem in this project, this one only exposes plain
actions/queries (get_best_vision_measurement_if_fresh(), get_tag_pose(),
is_any_vision_available()) -- DriveTrain calls these to fuse a vision fix
into its pose estimator (see subsystems/drivetrain.py), and
commands/vision_commands.py's ApproachTagCommand calls get_tag_pose() to
find out where a specific tag actually is on the field.

Note on photonlibpy vs. Java photonlib, carried over from the competition
bot's Vision subsystem: the 2026 photonlibpy release removed PoseStrategy
from PhotonPoseEstimator's constructor. Instead of picking a strategy once,
you call the estimate method for the strategy you want, per result. This
tries a multi-tag estimate first, falling back to a lowest-ambiguity
single-tag estimate when multi-tag PNP data isn't available.
"""
from __future__ import annotations

from typing import Optional

import wpilib
from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose3d

from constants import VisionConstants
from vision_measurement import VisionMeasurement


class Vision(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # AprilTagFieldLayout is a known, fixed map of tag ID -> field
        # position -- it doesn't need the camera to see anything. Loaded
        # once here so get_tag_pose() below is a plain lookup, usable even
        # for a tag the camera has never actually detected.
        self._field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

        self._camera = PhotonCamera(VisionConstants.CAMERA_NAME)
        self._pose_estimator = PhotonPoseEstimator(self._field_layout, VisionConstants.ROBOT_TO_CAMERA)

        self._last_result_timestamp = -1.0
        self._last_fresh_frame_fpga_ts = -1.0

        # Cached once per loop in periodic(), read by every caller this
        # cycle -- avoids recomputing the same pose estimate multiple times
        # if more than one thing asks for it in the same 20ms.
        self._cached_measurement: Optional[VisionMeasurement] = None

        self._telemetry_loop_counter = 0

    def get_tag_pose(self, tag_id: int) -> Optional[Pose3d]:
        """Looks up a tag's known field position, independent of whether
        the camera currently sees it. Returns None for an unknown tag ID."""
        return self._field_layout.getTagPose(tag_id)

    def get_best_vision_measurement_if_fresh(self) -> Optional[VisionMeasurement]:
        """The best measurement cached this loop, or None if there wasn't
        one, or it's older than VisionConstants.MAX_VISION_AGE_SECONDS."""
        measurement = self._cached_measurement
        if measurement is None:
            return None
        age_seconds = wpilib.Timer.getFPGATimestamp() - measurement.timestamp_seconds
        if age_seconds > VisionConstants.MAX_VISION_AGE_SECONDS:
            return None
        return measurement

    def is_any_vision_available(self) -> bool:
        """True if the camera has produced a fresh frame recently -- not
        the same as "a tag is currently visible": a connected camera
        pointed at a blank wall is available but sees nothing."""
        return (
            self._last_fresh_frame_fpga_ts >= 0
            and (wpilib.Timer.getFPGATimestamp() - self._last_fresh_frame_fpga_ts) <= 0.5
        )

    def _compute_measurement(self, result) -> Optional[VisionMeasurement]:
        if not result.hasTargets():
            return None

        estimated_pose = self._pose_estimator.estimateCoprocMultiTagPose(result)
        if estimated_pose is None:
            estimated_pose = self._pose_estimator.estimateLowestAmbiguityPose(result)
        if estimated_pose is None:
            return None

        pose2d = estimated_pose.estimatedPose.toPose2d()

        field_length = self._field_layout.getFieldLength()
        field_width = self._field_layout.getFieldWidth()
        if not (0 <= pose2d.X() <= field_length and 0 <= pose2d.Y() <= field_width):
            return None  # a pose off the field is never real

        targets = result.getTargets()
        if len(targets) == 1 and result.getBestTarget().getPoseAmbiguity() > VisionConstants.MAX_AMBIGUITY:
            return None

        num_tags = len(estimated_pose.targetsUsed)
        if num_tags >= VisionConstants.MIN_TAGS_FOR_MULTI_TAG:
            std_devs = VisionConstants.MULTI_TAG_STDDEVS
        else:
            average_distance = self._average_tag_distance(targets, pose2d)
            if average_distance > VisionConstants.MAX_TAG_DISTANCE_METERS:
                return None
            std_devs = (
                VisionConstants.SINGLE_TAG_CLOSE_STDDEVS
                if average_distance < 2.0
                else VisionConstants.SINGLE_TAG_FAR_STDDEVS
            )

        return VisionMeasurement(pose2d, estimated_pose.timestampSeconds, std_devs, num_tags)

    def _average_tag_distance(self, targets, robot_pose) -> float:
        distances = []
        for target in targets:
            tag_pose = self._field_layout.getTagPose(target.getFiducialId())
            if tag_pose is not None:
                distances.append(robot_pose.translation().distance(tag_pose.toPose2d().translation()))
        return sum(distances) / len(distances) if distances else float("inf")

    def periodic(self) -> None:
        result = self._camera.getLatestResult()
        timestamp = result.getTimestampSeconds()

        # A camera that has never sent a real result reports a placeholder
        # timestamp near zero (verified directly: -1e-06, not the -1.0 this
        # class starts _last_result_timestamp at) -- the `timestamp > 0`
        # check is what keeps that placeholder from being mistaken for an
        # actual fresh frame the very first time periodic() runs.
        if timestamp > 0 and timestamp > self._last_result_timestamp:
            self._last_result_timestamp = timestamp
            self._last_fresh_frame_fpga_ts = wpilib.Timer.getFPGATimestamp()
            self._cached_measurement = self._compute_measurement(result)

        self._telemetry_loop_counter += 1
        if self._telemetry_loop_counter >= VisionConstants.TELEMETRY_PERIOD_LOOPS:
            self._telemetry_loop_counter = 0
            wpilib.SmartDashboard.putBoolean("Vision/Available", self.is_any_vision_available())
            measurement = self._cached_measurement
            wpilib.SmartDashboard.putBoolean("Vision/HasMeasurement", measurement is not None)
            if measurement is not None:
                wpilib.SmartDashboard.putNumber("Vision/NumTagsUsed", measurement.num_tags_used)
