"""Ported from Robot.java.

AdvantageKit (org.littletonrobotics.junction) has no RobotPy port, so this
version drops it in favor of plain wpilib logging, per team decision: use
commands2.TimedCommandRobot (which schedules CommandScheduler automatically,
same as Java's CommandScheduler.getInstance().run() call in robotPeriodic)
plus wpilib.DataLogManager for on-disk + NetworkTables logging. There is no
AdvantageScope-style replay-log-swap in this version.
"""

from __future__ import annotations

from typing import Optional

import wpilib
from commands2 import Command, TimedCommandRobot

from robotcontainer import RobotContainer


class Robot(TimedCommandRobot):
    def robotInit(self) -> None:
        wpilib.DataLogManager.start()
        wpilib.DataLogManager.logNetworkTables(True)
        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog())

        self._autonomous_command: Optional[Command] = None
        self._robot_container = RobotContainer()

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        self._autonomous_command = self._robot_container.get_autonomous_command()

        if self._autonomous_command is not None:
            self._autonomous_command.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def autonomousExit(self) -> None:
        if self._autonomous_command is not None:
            self._autonomous_command.cancel()

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when teleop starts running.
        if self._autonomous_command is not None:
            self._autonomous_command.cancel()

        # Re-seed pose from vision at teleop start. Covers practice sessions where auto
        # is skipped, and catches any drift that accumulated during auto.
        self._robot_container.initialize_pose()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        from commands2 import CommandScheduler

        CommandScheduler.getInstance().cancelAll()
        print("Hello World")

    def testPeriodic(self) -> None:
        pass

    def simulationInit(self) -> None:
        pass

    def simulationPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
