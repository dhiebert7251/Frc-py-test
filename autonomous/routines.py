"""The two autonomous routines for the teaching-bot proof of concept.

Both use DriveTrain's encoder-based drive_distance_command() and gyro-based
turn_to_angle_command() -- no PathPlanner, no vision, just wheel encoders and
a gyro. See constants.Auto for the actual distances/angle used.
"""
from commands2 import Command, cmd

from constants import Auto
from subsystems.drivetrain import DriveTrain

FEET_TO_METERS = 0.3048


def drive_forward_only(drivetrain: DriveTrain) -> Command:
    """Drives straight forward Auto.DRIVE_FORWARD_ONLY_FEET feet, then stops."""
    return drivetrain.drive_distance_command(Auto.DRIVE_FORWARD_ONLY_FEET * FEET_TO_METERS)


def drive_turn_drive(drivetrain: DriveTrain) -> Command:
    """Drives forward, turns, drives forward again:
    Auto.DRIVE_TURN_DRIVE_FIRST_LEG_FEET feet -> turn
    Auto.DRIVE_TURN_DRIVE_TURN_DEGREES degrees (positive = left) ->
    Auto.DRIVE_TURN_DRIVE_SECOND_LEG_FEET feet."""
    return cmd.sequence(
        drivetrain.drive_distance_command(Auto.DRIVE_TURN_DRIVE_FIRST_LEG_FEET * FEET_TO_METERS),
        drivetrain.turn_to_angle_command(Auto.DRIVE_TURN_DRIVE_TURN_DEGREES),
        drivetrain.drive_distance_command(Auto.DRIVE_TURN_DRIVE_SECOND_LEG_FEET * FEET_TO_METERS),
    )
