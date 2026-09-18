"""Unit tests for the Vision subsystem's tag-pose lookup and availability
reporting.

There's no real camera or PhotonVision coprocessor in simulation, so these
tests only cover what doesn't require actual camera data: the AprilTag
field layout lookup (a fixed table, not something the camera has to see),
and the "nothing has arrived yet" default state. A disconnected
PhotonCamera reports a placeholder result with a near-zero timestamp
(verified directly: -1e-06) rather than throwing or returning None, which
is exactly the quirk `Vision.periodic()`'s `timestamp > 0` check exists to
not be fooled by -- see that file for the full explanation.
"""

from constants import VisionConstants


def test_get_tag_pose_returns_known_tag(control, robot):
    with control.run_robot():
        vision = robot.robot_container.vision
        assert vision.get_tag_pose(VisionConstants.EXAMPLE_TAG_ID) is not None


def test_get_tag_pose_returns_none_for_unknown_tag(control, robot):
    with control.run_robot():
        vision = robot.robot_container.vision
        assert vision.get_tag_pose(9999) is None


def test_no_measurement_without_camera_data(control, robot):
    with control.run_robot():
        vision = robot.robot_container.vision
        control.step_timing(seconds=0.1, autonomous=False, enabled=False)
        assert vision.get_best_vision_measurement_if_fresh() is None


def test_vision_not_available_without_camera_data(control, robot):
    with control.run_robot():
        vision = robot.robot_container.vision
        control.step_timing(seconds=0.1, autonomous=False, enabled=False)
        assert not vision.is_any_vision_available()
