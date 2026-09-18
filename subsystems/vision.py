"""Vision subsystem — dual PhotonVision cameras with AprilTag pose estimation.

Ported from Vision.java.

Note on photonlibpy vs. Java photonlib: the 2026 photonlibpy release removed
PoseStrategy from PhotonPoseEstimator's constructor. Instead of picking a
strategy once, you call the estimate method for the strategy you want, per
result. The Java code used MULTI_TAG_PNP_ON_COPROCESSOR, which internally
falls back to a lowest-ambiguity single-tag estimate when multi-tag PNP data
isn't available. That fallback is done explicitly here:
estimateCoprocMultiTagPose() first, then estimateLowestAmbiguityPose().
"""

from __future__ import annotations

from typing import Optional

import wpilib
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose2d, Rotation2d
from commands2 import Subsystem

from constants import VisionConstants

from vision_measurement import VisionMeasurement

CAMERA_STALE_TIMEOUT_SECONDS = 0.5


class Vision(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Class fields
        self._last_front_timestamp = -1.0
        self._last_rear_timestamp = -1.0
        self._last_front_fpga_ts = -1.0  # FPGA-time seconds, not wall-clock ms
        self._last_rear_fpga_ts = -1.0
        self._telemetry_loop_counter = 0

        # Per-loop caches — computed once in periodic(), read by all callers this cycle.
        self._cached_measurement: Optional[VisionMeasurement] = None
        # List indexed by tag ID (1-30). O(1) lookup, index 0 unused.
        self._cached_tags_visible: list[bool] = [False] * 31
        self._cached_front_result: Optional[PhotonPipelineResult] = None
        self._cached_rear_result: Optional[PhotonPipelineResult] = None

        # Load AprilTag field layout
        self._field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

        # Initialize cameras
        self._front_camera = PhotonCamera(VisionConstants.FRONT_CAMERA_NAME)
        self._rear_camera = PhotonCamera(VisionConstants.REAR_CAMERA_NAME)

        # Initialize pose estimators
        self._front_pose_estimator = PhotonPoseEstimator(
            self._field_layout, VisionConstants.ROBOT_TO_FRONT_CAM
        )
        self._rear_pose_estimator = PhotonPoseEstimator(
            self._field_layout, VisionConstants.ROBOT_TO_REAR_CAM
        )

    def get_best_vision_measurement(self) -> Optional[VisionMeasurement]:
        """Returns the best vision measurement cached this loop by periodic()."""
        return self._cached_measurement

    def get_best_vision_measurement_if_fresh(self) -> Optional[VisionMeasurement]:
        """Returns the best cached vision measurement if it is recent enough."""
        measurement = self._cached_measurement
        if measurement is not None and self._is_measurement_fresh(measurement):
            return measurement
        return None

    def _is_measurement_fresh(self, measurement: VisionMeasurement) -> bool:
        return (
            wpilib.Timer.getFPGATimestamp() - measurement.timestamp_seconds
        ) <= VisionConstants.MAX_VISION_AGE_SECONDS

    def _compute_best_vision_measurement(self) -> Optional[VisionMeasurement]:
        """Computes the best vision measurement from all cameras. Called once per loop."""
        front_result = self._get_front_result()
        rear_result = self._get_rear_result()

        front_measurement = self._process_camera_result(
            self._front_pose_estimator,
            self._front_camera,
            front_result,
            VisionConstants.FRONT_CAMERA_NAME,
        )
        rear_measurement = self._process_camera_result(
            self._rear_pose_estimator,
            self._rear_camera,
            rear_result,
            VisionConstants.REAR_CAMERA_NAME,
        )

        # If both cameras have measurements, choose the best one
        if front_measurement is not None and rear_measurement is not None:
            front = front_measurement
            rear = rear_measurement

            # Prefer multi-tag over single-tag
            if (
                front.num_tags_used >= VisionConstants.MIN_TAGS_FOR_MULTI_TAG
                and rear.num_tags_used < VisionConstants.MIN_TAGS_FOR_MULTI_TAG
            ):
                return front_measurement
            if (
                rear.num_tags_used >= VisionConstants.MIN_TAGS_FOR_MULTI_TAG
                and front.num_tags_used < VisionConstants.MIN_TAGS_FOR_MULTI_TAG
            ):
                return rear_measurement

            # Both multi-tag or both single-tag: prefer closer distance
            if front.average_distance < rear.average_distance:
                return front_measurement
            return rear_measurement

        # Return whichever is present
        return front_measurement if front_measurement is not None else rear_measurement

    def _process_camera_result(
        self,
        pose_estimator: PhotonPoseEstimator,
        camera: PhotonCamera,
        result: PhotonPipelineResult,
        camera_name: str,
    ) -> Optional[VisionMeasurement]:
        """Process a single camera's result and create a VisionMeasurement if valid."""
        self._update_camera_freshness(camera, result)

        if not result.hasTargets():
            return None

        estimated_pose = pose_estimator.estimateCoprocMultiTagPose(result)
        if estimated_pose is None:
            estimated_pose = pose_estimator.estimateLowestAmbiguityPose(result)

        if estimated_pose is None:
            return None

        pose2d = estimated_pose.estimatedPose.toPose2d()

        # Compute average distance exactly once; reuse for quality gate,
        # std-dev selection, and the VisionMeasurement. This also ensures
        # infinity (unknown tag IDs) is caught by the quality gate before
        # it can reach _calculate_standard_deviations().
        avg_distance = self._calculate_average_tag_distance(result.getTargets(), pose2d)
        best_target_ambiguity = result.getBestTarget().getPoseAmbiguity()

        # Quality gating
        if not self._should_use_measurement(estimated_pose, result, avg_distance):
            wpilib.SmartDashboard.putString(
                f"Vision/{camera_name}CamStatus", "Rejected (quality gate)"
            )
            return None

        std_devs = self._calculate_standard_deviations(estimated_pose, avg_distance)
        num_tags = len(estimated_pose.targetsUsed)

        return VisionMeasurement(
            pose2d,
            estimated_pose.timestampSeconds,
            std_devs,
            best_target_ambiguity,
            num_tags,
            avg_distance,
        )

    def _update_camera_freshness(self, camera: PhotonCamera, result: PhotonPipelineResult) -> None:
        """Track camera frame freshness based on result timestamp. Uses FPGA time
        throughout — same domain as PhotonVision timestamps."""
        timestamp_seconds = result.getTimestampSeconds()
        now_fpga = wpilib.Timer.getFPGATimestamp()

        if camera is self._front_camera and timestamp_seconds > self._last_front_timestamp:
            self._last_front_timestamp = timestamp_seconds
            self._last_front_fpga_ts = now_fpga
        elif camera is self._rear_camera and timestamp_seconds > self._last_rear_timestamp:
            self._last_rear_timestamp = timestamp_seconds
            self._last_rear_fpga_ts = now_fpga

    def _should_use_measurement(
        self, pose, result: PhotonPipelineResult, avg_distance: float
    ) -> bool:
        """Determine if a measurement should be used based on quality criteria."""
        pose2d = pose.estimatedPose.toPose2d()

        field_length = self._field_layout.getFieldLength()
        field_width = self._field_layout.getFieldWidth()

        if pose2d.X() < 0 or pose2d.X() > field_length or pose2d.Y() < 0 or pose2d.Y() > field_width:
            return False

        # For single-tag detections, check ambiguity
        if len(result.getTargets()) == 1:
            ambiguity = result.getBestTarget().getPoseAmbiguity()
            if ambiguity > VisionConstants.MAX_AMBIGUITY:
                return False

        # Reject if beyond max distance; also catches infinity (unknown tag IDs)
        if avg_distance > VisionConstants.MAX_TAG_DISTANCE_METERS:
            return False

        return True

    def _calculate_standard_deviations(
        self, pose, average_distance: float
    ) -> tuple[float, float, float]:
        """Multi-tag detections and closer distances get higher trust (lower std dev)."""
        num_tags = len(pose.targetsUsed)

        if num_tags >= VisionConstants.MIN_TAGS_FOR_MULTI_TAG:
            return VisionConstants.MULTI_TAG_STDDEVS

        if average_distance < 2.0:
            return VisionConstants.SINGLE_TAG_CLOSE_STDDEVS
        return VisionConstants.SINGLE_TAG_FAR_STDDEVS

    def _calculate_average_tag_distance(
        self, targets: list[PhotonTrackedTarget], robot_pose: Pose2d
    ) -> float:
        """Calculate average distance from robot to all detected tags."""
        if not targets:
            return 0.0

        total_distance = 0.0
        valid_tag_count = 0

        for target in targets:
            tag_pose = self._field_layout.getTagPose(target.getFiducialId())
            if tag_pose is not None:
                total_distance += robot_pose.translation().distance(
                    tag_pose.toPose2d().translation()
                )
                valid_tag_count += 1

        # If no valid tag IDs were in the layout, treat as invalid/very poor measurement
        if valid_tag_count == 0:
            return float("inf")

        return total_distance / valid_tag_count

    def _get_front_result(self) -> PhotonPipelineResult:
        if self._cached_front_result is not None:
            return self._cached_front_result
        return self._front_camera.getLatestResult()

    def _get_rear_result(self) -> PhotonPipelineResult:
        if self._cached_rear_result is not None:
            return self._cached_rear_result
        return self._rear_camera.getLatestResult()

    def get_best_target(self) -> Optional[PhotonTrackedTarget]:
        """Get the best target from all cameras (highest confidence/lowest ambiguity)."""
        front_result = self._get_front_result()
        rear_result = self._get_rear_result()

        best_target: Optional[PhotonTrackedTarget] = None
        lowest_ambiguity = float("inf")

        if front_result.hasTargets():
            front_target = front_result.getBestTarget()
            if front_target.getPoseAmbiguity() < lowest_ambiguity:
                best_target = front_target
                lowest_ambiguity = front_target.getPoseAmbiguity()

        if rear_result.hasTargets():
            rear_target = rear_result.getBestTarget()
            if rear_target.getPoseAmbiguity() < lowest_ambiguity:
                best_target = rear_target
                lowest_ambiguity = rear_target.getPoseAmbiguity()

        return best_target

    def is_tag_visible(self, tag_id: int) -> bool:
        """Check if a specific AprilTag is visible in any camera."""
        return 1 <= tag_id <= 30 and self._cached_tags_visible[tag_id]

    def get_visible_tags(self) -> list[int]:
        """Returns a list of all currently visible AprilTag IDs from all cameras."""
        return [i for i in range(1, 31) if self._cached_tags_visible[i]]

    def _compute_visible_tags(self) -> list[bool]:
        """Builds the visible-tag list from both cameras. Called once per loop
        from periodic() when a new frame has arrived."""
        visible = [False] * 31

        front_result = self._get_front_result()
        if front_result.hasTargets():
            for target in front_result.getTargets():
                tag_id = target.getFiducialId()
                if 1 <= tag_id <= 30:
                    visible[tag_id] = True

        rear_result = self._get_rear_result()
        if rear_result.hasTargets():
            for target in rear_result.getTargets():
                tag_id = target.getFiducialId()
                if 1 <= tag_id <= 30:
                    visible[tag_id] = True

        return visible

    def get_distance_to_pose(self, target_pose: Pose2d) -> Optional[float]:
        """Get distance from current best pose estimate to a target pose."""
        measurement = self.get_best_vision_measurement_if_fresh()
        if measurement is None:
            return None
        return measurement.estimated_pose.translation().distance(target_pose.translation())

    def get_yaw_to_pose(self, target_pose: Pose2d) -> Optional[Rotation2d]:
        """Get yaw angle from current best pose estimate to a target pose."""
        measurement = self.get_best_vision_measurement_if_fresh()
        if measurement is None:
            return None
        current_pose = measurement.estimated_pose
        translation = target_pose.translation() - current_pose.translation()
        return Rotation2d(translation.X(), translation.Y()) - current_pose.rotation()

    def is_aligned_with_target(self, target_pose: Pose2d, tolerance_deg: float) -> bool:
        """Check if robot is aligned with a target pose within tolerance."""
        yaw = self.get_yaw_to_pose(target_pose)
        if yaw is None:
            return False
        return abs(yaw.degrees()) < tolerance_deg

    def get_alliance_hub_pose(self) -> Pose2d:
        """Returns the hub pose for the current alliance (blue by default if unknown)."""
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            return VisionConstants.RED_HUB_POSE
        return VisionConstants.BLUE_HUB_POSE

    def get_distance_to_hub(self) -> Optional[float]:
        return self.get_distance_to_pose(self.get_alliance_hub_pose())

    def is_aligned_with_hub(self, tolerance_deg: float) -> bool:
        return self.is_aligned_with_target(self.get_alliance_hub_pose(), tolerance_deg)

    def get_distance_to_hp_station(self) -> Optional[float]:
        return self.get_distance_to_pose(VisionConstants.HP_STATION_POSE)

    def is_aligned_with_hp_station(self, tolerance_deg: float) -> bool:
        return self.is_aligned_with_target(VisionConstants.HP_STATION_POSE, tolerance_deg)

    def get_distance_to_trench(self) -> Optional[float]:
        return self.get_distance_to_pose(VisionConstants.TRENCH_POSE)

    def is_aligned_with_trench(self, tolerance_deg: float) -> bool:
        return self.is_aligned_with_target(VisionConstants.TRENCH_POSE, tolerance_deg)

    def get_distance_to_depot(self) -> Optional[float]:
        return self.get_distance_to_pose(VisionConstants.DEPOT_POSE)

    def is_aligned_with_depot(self, tolerance_deg: float) -> bool:
        return self.is_aligned_with_target(VisionConstants.DEPOT_POSE, tolerance_deg)

    def get_distance_to_outpost(self) -> Optional[float]:
        return self.get_distance_to_pose(VisionConstants.OUTPOST_POSE)

    def is_aligned_with_outpost(self, tolerance_deg: float) -> bool:
        return self.is_aligned_with_target(VisionConstants.OUTPOST_POSE, tolerance_deg)

    def get_distance_to_tower(self) -> Optional[float]:
        return self.get_distance_to_pose(VisionConstants.TOWER_POSE)

    def is_aligned_with_tower(self, tolerance_deg: float) -> bool:
        return self.is_aligned_with_target(VisionConstants.TOWER_POSE, tolerance_deg)

    def _update_telemetry(self) -> None:
        """Update telemetry to SmartDashboard."""
        # Camera connection status
        wpilib.SmartDashboard.putBoolean(
            "Vision/FrontCamConnected", self._is_camera_connected(self._front_camera)
        )
        wpilib.SmartDashboard.putBoolean(
            "Vision/RearCamConnected", self._is_camera_connected(self._rear_camera)
        )

        # Visible tags — built on demand from the cached list
        visible_tag_list = self.get_visible_tags()
        wpilib.SmartDashboard.putNumber("Vision/NumTagsVisible", len(visible_tag_list))
        wpilib.SmartDashboard.putString("Vision/VisibleTags", str(visible_tag_list))

        # Best measurement — read from per-loop cache
        best_measurement = self._cached_measurement

        if best_measurement is not None:
            pose = best_measurement.estimated_pose

            wpilib.SmartDashboard.putNumber("Vision/BestPoseX", pose.X())
            wpilib.SmartDashboard.putNumber("Vision/BestPoseY", pose.Y())
            wpilib.SmartDashboard.putNumber("Vision/BestPoseTheta", pose.rotation().degrees())
            wpilib.SmartDashboard.putNumber("Vision/NumTagsUsed", best_measurement.num_tags_used)
            wpilib.SmartDashboard.putNumber("Vision/AvgTagDistance", best_measurement.average_distance)

            hub_pose = self.get_alliance_hub_pose()
            hub_distance = self.get_distance_to_pose(hub_pose)
            if hub_distance is not None:
                wpilib.SmartDashboard.putNumber("Vision/HubDistance", hub_distance)
            wpilib.SmartDashboard.putBoolean(
                "Vision/HubAligned", self.is_aligned_with_target(hub_pose, 2.0)
            )

            hp_distance = self.get_distance_to_pose(VisionConstants.HP_STATION_POSE)
            if hp_distance is not None:
                wpilib.SmartDashboard.putNumber("Vision/HPStationDistance", hp_distance)
            wpilib.SmartDashboard.putBoolean(
                "Vision/HPStationAligned",
                self.is_aligned_with_target(VisionConstants.HP_STATION_POSE, 2.0),
            )

            trench_distance = self.get_distance_to_pose(VisionConstants.TRENCH_POSE)
            if trench_distance is not None:
                wpilib.SmartDashboard.putNumber("Vision/TrenchDistance", trench_distance)
            wpilib.SmartDashboard.putBoolean(
                "Vision/TrenchAligned",
                self.is_aligned_with_target(VisionConstants.TRENCH_POSE, 2.0),
            )

            depot_distance = self.get_distance_to_pose(VisionConstants.DEPOT_POSE)
            if depot_distance is not None:
                wpilib.SmartDashboard.putNumber("Vision/DepotDistance", depot_distance)
            wpilib.SmartDashboard.putBoolean(
                "Vision/DepotAligned",
                self.is_aligned_with_target(VisionConstants.DEPOT_POSE, 2.0),
            )

            outpost_distance = self.get_distance_to_pose(VisionConstants.OUTPOST_POSE)
            if outpost_distance is not None:
                wpilib.SmartDashboard.putNumber("Vision/OutpostDistance", outpost_distance)
            wpilib.SmartDashboard.putBoolean(
                "Vision/OutpostAligned",
                self.is_aligned_with_target(VisionConstants.OUTPOST_POSE, 2.0),
            )

            tower_distance = self.get_distance_to_pose(VisionConstants.TOWER_POSE)
            if tower_distance is not None:
                wpilib.SmartDashboard.putNumber("Vision/TowerDistance", tower_distance)
            wpilib.SmartDashboard.putBoolean(
                "Vision/TowerAligned",
                self.is_aligned_with_target(VisionConstants.TOWER_POSE, 2.0),
            )
        else:
            wpilib.SmartDashboard.putNumber("Vision/BestPoseX", 0.0)
            wpilib.SmartDashboard.putNumber("Vision/BestPoseY", 0.0)
            wpilib.SmartDashboard.putNumber("Vision/BestPoseTheta", 0.0)
            wpilib.SmartDashboard.putNumber("Vision/NumTagsUsed", 0)
            wpilib.SmartDashboard.putNumber("Vision/AvgTagDistance", 0.0)

            wpilib.SmartDashboard.putNumber("Vision/HubDistance", -1.0)
            wpilib.SmartDashboard.putBoolean("Vision/HubAligned", False)
            wpilib.SmartDashboard.putNumber("Vision/HPStationDistance", float("nan"))
            wpilib.SmartDashboard.putBoolean("Vision/HPStationAligned", False)
            wpilib.SmartDashboard.putNumber("Vision/TrenchDistance", float("nan"))
            wpilib.SmartDashboard.putBoolean("Vision/TrenchAligned", False)
            wpilib.SmartDashboard.putNumber("Vision/DepotDistance", float("nan"))
            wpilib.SmartDashboard.putBoolean("Vision/DepotAligned", False)
            wpilib.SmartDashboard.putNumber("Vision/OutpostDistance", float("nan"))
            wpilib.SmartDashboard.putBoolean("Vision/OutpostAligned", False)
            wpilib.SmartDashboard.putNumber("Vision/TowerDistance", float("nan"))
            wpilib.SmartDashboard.putBoolean("Vision/TowerAligned", False)

    def _is_camera_connected(self, camera: PhotonCamera) -> bool:
        """Check if a camera is connected by verifying fresh frames are still arriving.
        Uses FPGA time (same domain as PhotonVision timestamps) instead of wall-clock."""
        now_fpga = wpilib.Timer.getFPGATimestamp()

        if camera is self._front_camera:
            return (
                self._last_front_fpga_ts >= 0
                and (now_fpga - self._last_front_fpga_ts) <= CAMERA_STALE_TIMEOUT_SECONDS
            )
        if camera is self._rear_camera:
            return (
                self._last_rear_fpga_ts >= 0
                and (now_fpga - self._last_rear_fpga_ts) <= CAMERA_STALE_TIMEOUT_SECONDS
            )

        return False

    def is_any_vision_available(self) -> bool:
        """Returns true if at least one camera is producing fresh frames."""
        return self._is_camera_connected(self._front_camera) or self._is_camera_connected(
            self._rear_camera
        )

    def periodic(self) -> None:
        self._cached_front_result = self._front_camera.getLatestResult()
        self._cached_rear_result = self._rear_camera.getLatestResult()

        # Skip heavy recomputation when neither camera has produced a new frame.
        front_ts = self._cached_front_result.getTimestampSeconds()
        rear_ts = self._cached_rear_result.getTimestampSeconds()
        if front_ts > self._last_front_timestamp or rear_ts > self._last_rear_timestamp:
            self._cached_measurement = self._compute_best_vision_measurement()
            self._cached_tags_visible = self._compute_visible_tags()

        self._telemetry_loop_counter += 1
        if self._telemetry_loop_counter >= VisionConstants.TELEMETRY_PERIOD_LOOPS:
            self._telemetry_loop_counter = 0
            self._update_telemetry()
