"""Builds the SmartDashboard autonomous-routine chooser for the teaching-bot
proof of concept: two real routines plus a "Do Nothing" default.
"""
import wpilib
from commands2 import cmd

import autonomous.routines
from subsystems.drivetrain import DriveTrain

DO_NOTHING_NAME = "Do Nothing"
DRIVE_FORWARD_NAME = "Drive Forward 10 ft"
DRIVE_TURN_DRIVE_NAME = "Drive 5ft, Turn Left 90, Drive 3ft"


def build(drivetrain: DriveTrain) -> wpilib.SendableChooser:
    chooser = wpilib.SendableChooser()
    chooser.setDefaultOption(DO_NOTHING_NAME, cmd.none())
    chooser.addOption(DRIVE_FORWARD_NAME, autonomous.routines.drive_forward_only(drivetrain))
    chooser.addOption(DRIVE_TURN_DRIVE_NAME, autonomous.routines.drive_turn_drive(drivetrain))
    wpilib.SmartDashboard.putData("Auto Chooser", chooser)
    return chooser
