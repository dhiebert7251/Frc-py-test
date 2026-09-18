"""Autonomous chooser construction.

Wraps PathPlanner's AutoBuilder-generated chooser with a "Do Nothing"
default, split out of RobotContainer to mirror how established RobotPy
teams keep autonomous-only wiring separate from teleop bindings. Must be
called after DriveTrain's constructor has already run
AutoBuilder.configure() -- buildAutoChooser() raises otherwise.
"""

import wpilib
from commands2 import Command, cmd
from pathplannerlib.auto import AutoBuilder

DO_NOTHING_NAME = "Do Nothing"


def build() -> tuple[wpilib.SendableChooser, Command]:
    """Returns (chooser, do_nothing_auto).

    AutoBuilder.buildAutoChooser() raises if AutoBuilder.configure() didn't
    succeed (e.g. no deploy/pathplanner/settings.json yet -- DriveTrain
    already catches that and logs a warning, matching the Java source).
    The Java source calls the equivalent of buildAutoChooser() unguarded
    right after, which would crash RobotContainer's constructor under that
    same condition; this falls back to a chooser offering only "Do Nothing"
    instead, since the whole point of the fallback (and of this port's test
    suite) is for the robot to still come up when PathPlanner isn't set up.
    """
    do_nothing_auto = cmd.waitUntil(wpilib.DriverStation.isTeleopEnabled).withName(DO_NOTHING_NAME)

    if AutoBuilder.isConfigured():
        chooser = AutoBuilder.buildAutoChooser()
    else:
        chooser = wpilib.SendableChooser()
    chooser.setDefaultOption(DO_NOTHING_NAME, do_nothing_auto)

    return chooser, do_nothing_auto
