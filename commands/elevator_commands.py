"""Commands for Elevator.

RaiseElevatorCommand and LowerElevatorCommand are near-identical on
purpose: both are meant to be bound with whileTrue (see robotcontainer.py),
so isFinished() always returns False and the command only stops when the
button is released (which interrupts it, calling end()) or when execute()
itself detects a limit switch and calls stop(). That second check matters
even though the command is also "supposed" to stop when the button is
released: it protects the mechanism the instant it reaches a limit, without
waiting on the operator to notice and let go.
"""
from __future__ import annotations

from commands2 import Command

from constants import ElevatorConstants
from subsystems.elevator import Elevator


class RaiseElevatorCommand(Command):
    def __init__(self, elevator: Elevator) -> None:
        # `self`, `elevator: Elevator`, and `-> None` all follow the same
        # pattern explained in commands/gripper_commands.py's
        # IntakeCommand.__init__ and commands/drivetrain_commands.py's
        # TeleopDriveCommand.__init__ -- worth reading either of those
        # first if this shape (a typed parameter, a `-> None` return hint,
        # `super().__init__()` as the first line) is still new.
        super().__init__()
        self._elevator = elevator
        self.addRequirements(elevator)

    def execute(self) -> None:
        if self._elevator.is_at_top():
            self._elevator.stop()
        else:
            self._elevator.set_speed(ElevatorConstants.RAISE_SPEED)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        # `interrupted: bool` -- see DriveDistanceCommand.end() in
        # commands/drivetrain_commands.py for what this parameter means and
        # why end() always receives it, even when (like here) a command
        # doesn't need to look at its value.
        self._elevator.stop()


class LowerElevatorCommand(Command):
    def __init__(self, elevator: Elevator) -> None:
        super().__init__()
        self._elevator = elevator
        self.addRequirements(elevator)

    def execute(self) -> None:
        if self._elevator.is_at_bottom():
            self._elevator.stop()
        else:
            self._elevator.set_speed(ElevatorConstants.LOWER_SPEED)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self._elevator.stop()
