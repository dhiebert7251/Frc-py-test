"""The two autonomous routines for the teaching-bot proof of concept.

Both are built entirely from commands/drivetrain_commands.py's two PID
commands -- no PathPlanner, no vision, just wheel encoders and a gyro. All
distances/angles are in feet/degrees, taken straight from constants.Auto;
DriveDistanceCommand converts feet to meters internally (see its
docstring), so nothing in this file ever touches metric units.
"""
from commands2 import Command, cmd

from commands.drivetrain_commands import DriveDistanceCommand, TurnToAngleCommand
from constants import Auto
from subsystems.drivetrain import DriveTrain


def drive_forward_only(drivetrain: DriveTrain) -> Command:
    """Drives straight forward Auto.DRIVE_FORWARD_ONLY_FEET feet, then stops."""
    return DriveDistanceCommand(drivetrain, Auto.DRIVE_FORWARD_ONLY_FEET)


def drive_turn_drive(drivetrain: DriveTrain) -> Command:
    """Drives forward, turns, drives forward again:
    Auto.DRIVE_TURN_DRIVE_FIRST_LEG_FEET feet -> turn
    Auto.DRIVE_TURN_DRIVE_TURN_DEGREES degrees (positive = left) ->
    Auto.DRIVE_TURN_DRIVE_SECOND_LEG_FEET feet."""
    return cmd.sequence(
        DriveDistanceCommand(drivetrain, Auto.DRIVE_TURN_DRIVE_FIRST_LEG_FEET),
        TurnToAngleCommand(drivetrain, Auto.DRIVE_TURN_DRIVE_TURN_DEGREES),
        DriveDistanceCommand(drivetrain, Auto.DRIVE_TURN_DRIVE_SECOND_LEG_FEET),
    )
