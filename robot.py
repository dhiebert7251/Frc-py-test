"""Entry point for the teaching-bot proof of concept.

Same commands2.TimedCommandRobot pattern as the competition bot -- no
AdvantageKit, no vision-specific logging, just wpilib.DataLogManager for
on-disk + NetworkTables logging.

2027 alpha preview (verified against robotpy==2027.0.0a6 in this session):
this file's whole shape -- a robotInit() that runs once, then
mode-specific *Init()/*Periodic() callbacks the scheduler fires forever
after -- is already changing. TimedRobot in 2027.0.0a6 has NO robotInit()
method at all anymore (checked directly: it's simply not in the class), and
adds a 4th "Utility" mode alongside disabled/autonomous/teleop (utilityInit/
utilityPeriodic/utilityExit, isUtility()). wpilib.MatchState and
wpilib.RobotState also already exist as their own top-level classes in
2027.0.0a6, splitting apart what today is all bundled into
wpilib.DriverStation. None of this is usable yet, though: commands2 (and so
TimedCommandRobot, which this file actually subclasses) has no 2027 release
at all. So this file stays on the current, 2026-stable robotInit() pattern
below -- it's what actually runs -- with this note as a heads-up for
whoever eventually ports it forward.
"""
from __future__ import annotations

from typing import Optional

import wpilib
from commands2 import Command, CommandScheduler, TimedCommandRobot

from robotcontainer import RobotContainer


class Robot(TimedCommandRobot):
    def robotInit(self) -> None:
        wpilib.DataLogManager.start()
        wpilib.DataLogManager.logNetworkTables(True)
        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog())

        self._autonomous_command: Optional[Command] = None
        self.robot_container = RobotContainer()

    def autonomousInit(self) -> None:
        self._autonomous_command = self.robot_container.get_autonomous_command()
        if self._autonomous_command is not None:
            self._autonomous_command.schedule()

    def autonomousExit(self) -> None:
        if self._autonomous_command is not None:
            self._autonomous_command.cancel()

    def teleopInit(self) -> None:
        if self._autonomous_command is not None:
            self._autonomous_command.cancel()

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(Robot)
