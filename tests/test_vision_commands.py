"""Unit tests for ApproachTagCommand's target computation and 3-phase state
machine.

Same physics.py caveat as tests/test_drivetrain.py applies here: a poked
encoder/gyro value is read by the scheduled command before physics.py's own
recomputation overwrites it a moment later, so these tests check
`command._phase` (this command's own Python state, untouched by physics.py)
immediately after each poke-and-step, rather than trying to read a raw
sensor value back out afterward.
"""

import pytest
import wpilib.simulation

from commands.vision_commands import ApproachTagCommand
from constants import METERS_PER_FOOT, VisionConstants


def test_approach_tag_command_fails_for_unknown_tag(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        vision = robot.robot_container.vision
        control.step_timing(seconds=0.02, autonomous=True, enabled=True)

        command = ApproachTagCommand(drivetrain, vision, 9999, 3.0)
        command.schedule()
        control.step_timing(seconds=0.02, autonomous=True, enabled=True)
        assert not command.isScheduled()


def test_approach_tag_command_computes_standoff_point(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        vision = robot.robot_container.vision
        tag_pose = vision.get_tag_pose(VisionConstants.EXAMPLE_TAG_ID).toPose2d()

        command = ApproachTagCommand(drivetrain, vision, VisionConstants.EXAMPLE_TAG_ID, 3.0)
        command.initialize()

        assert command._phase == "TURN_TO_TARGET"
        distance_from_tag = command._target_point.distance(tag_pose.translation())
        assert distance_from_tag == pytest.approx(3.0 * METERS_PER_FOOT, abs=0.01)


def test_approach_tag_command_progresses_through_all_phases(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        vision = robot.robot_container.vision
        control.step_timing(seconds=0.02, autonomous=True, enabled=True)

        navx_sim = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        yaw_sim = navx_sim.getDouble("Yaw")

        command = ApproachTagCommand(drivetrain, vision, VisionConstants.EXAMPLE_TAG_ID, 3.0)
        command.schedule()
        control.step_timing(seconds=0.02, autonomous=True, enabled=True)
        assert command.isScheduled()
        assert command._phase == "TURN_TO_TARGET"

        # Fake having turned exactly to the bearing TURN_TO_TARGET is aiming
        # for. get_heading_degrees() negates the raw navX yaw (see
        # DriveTrain), so the sign is flipped here to match.
        bearing_degrees = command._turn_pid.getSetpoint()
        yaw_sim.set(-bearing_degrees)
        control.step_timing(seconds=0.02, autonomous=True, enabled=True)
        assert command._phase == "DRIVE_TO_TARGET"

        # Fake having driven the exact distance this phase is targeting.
        target_distance = command._drive_pid.getSetpoint()
        drivetrain._left_encoder.setPosition(target_distance)
        drivetrain._right_encoder.setPosition(target_distance)
        control.step_timing(seconds=0.02, autonomous=True, enabled=True)
        assert command._phase == "FACE_TAG"

        # Fake having turned to the final heading -- facing the tag directly
        # in this case, since this command was built with no offset.
        final_heading_degrees = command._turn_pid.getSetpoint()
        yaw_sim.set(-final_heading_degrees)
        control.step_timing(seconds=0.02, autonomous=True, enabled=True)
        assert not command.isScheduled()
