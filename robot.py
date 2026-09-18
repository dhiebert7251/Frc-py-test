"""2027-alpha tracking placeholder -- NOT a working robot. See README.md.

This only proves that a bare wpilib.TimedRobot (no commands2 -- it has no
2027 release either) constructs and runs under the 2027 alpha HAL. It was
verified by calling robotInit() directly under a simulated HAL running
robotpy==2027.0.0a6 (the newest version this project's dev environment could
actually install).
"""

from __future__ import annotations

import wpilib


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        wpilib.SmartDashboard.putString("Status", "2027-alpha tracking placeholder -- see README.md")

    def robotPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
