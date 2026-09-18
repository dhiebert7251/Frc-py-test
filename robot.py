"""Entry point for the teaching-bot proof of concept.

Same commands2.TimedCommandRobot pattern as the competition bot -- no
AdvantageKit, no vision-specific logging, just wpilib.DataLogManager for
on-disk + NetworkTables logging.
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
