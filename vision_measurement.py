"""Container for a single vision-based pose measurement from PhotonVision.

Teaching-bot proof of concept. Simpler than the competition bot's version:
one camera means there's no "which of two cameras' measurements do we
trust more" arbitration to carry alongside the data, so this only needs
what DriveTrain's pose estimator and ApproachTagCommand actually use.

standard_deviations is a plain (x_meters, y_meters, heading_radians) tuple,
not a Matrix<N3, N1> -- RobotPy's DifferentialDrivePoseEstimator takes
standard deviations as a 3-tuple, so there is no matrix type to mirror.
"""
from dataclasses import dataclass

from wpimath.geometry import Pose2d


@dataclass(frozen=True)
class VisionMeasurement:
    estimated_pose: Pose2d
    timestamp_seconds: float
    standard_deviations: tuple[float, float, float]
    num_tags_used: int
