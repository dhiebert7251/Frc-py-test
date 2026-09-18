"""Container for vision-based pose measurements from PhotonVision.

Ported from VisionMeasurement.java. Used to pass vision data from the Vision
subsystem to the drivetrain's pose estimator.

standard_deviations is a plain (x_meters, y_meters, heading_radians) tuple
rather than a Matrix<N3, N1> — RobotPy's DifferentialDrivePoseEstimator
takes standard deviations as a 3-tuple, so there is no matrix type to mirror.
"""

from dataclasses import dataclass

from wpimath.geometry import Pose2d


@dataclass(frozen=True)
class VisionMeasurement:
    estimated_pose: Pose2d
    timestamp_seconds: float
    standard_deviations: tuple[float, float, float]
    best_target_ambiguity: float
    num_tags_used: int
    average_distance: float
