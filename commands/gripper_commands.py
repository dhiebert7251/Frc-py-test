"""Commands for Gripper.

The simplest two commands in this project: hold a fixed speed while bound
(whileTrue in robotcontainer.py), stop when released. No sensors, no
looping math -- a good first command file to read.
"""
from __future__ import annotations

from commands2 import Command

from constants import GripperConstants
from subsystems.gripper import Gripper


class IntakeCommand(Command):
    def __init__(self, gripper: Gripper) -> None:
        super().__init__()
        self._gripper = gripper
        self.addRequirements(gripper)

    def execute(self) -> None:
        self._gripper.set_speed(GripperConstants.INTAKE_SPEED)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self._gripper.stop()


class EjectCommand(Command):
    def __init__(self, gripper: Gripper) -> None:
        super().__init__()
        self._gripper = gripper
        self.addRequirements(gripper)

    def execute(self) -> None:
        self._gripper.set_speed(GripperConstants.EJECT_SPEED)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self._gripper.stop()
