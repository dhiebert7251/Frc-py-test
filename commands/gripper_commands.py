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
        # __init__ is a "constructor" -- Python calls this method
        # automatically every time something builds a new IntakeCommand,
        # e.g. `IntakeCommand(self.gripper)` in robotcontainer.py. A few
        # pieces of syntax on this line are worth calling out individually,
        # since the exact same pattern repeats in every __init__ in this
        # project:
        #
        #   * `self` is the new IntakeCommand object being built. Every
        #     method defined on a class takes it as the first parameter,
        #     and Python fills it in for you automatically -- you never
        #     write it yourself at the call site (it's
        #     `IntakeCommand(gripper)`, not `IntakeCommand(self, gripper)`).
        #     Anything stored onto `self` here, like `self._gripper` below,
        #     is remembered for as long as this particular IntakeCommand
        #     object exists, and every other method on the class
        #     (execute(), isFinished(), end()) can read it back through
        #     that same `self`.
        #   * `gripper: Gripper` is a parameter written as `name: Type`.
        #     `gripper` is what this method calls the value passed in;
        #     `: Gripper` is a TYPE HINT saying that value should be a
        #     `Gripper` object (see subsystems/gripper.py). Python does
        #     NOT enforce this at runtime -- nothing crashes if the wrong
        #     type gets passed in -- but it lets an editor/IDE flag a
        #     mistake before the code ever runs, and it tells a human
        #     reading this file exactly what `gripper` is supposed to be
        #     without having to go hunt down every place this class gets
        #     constructed.
        #   * `-> None` after the closing `)` is the METHOD's own return
        #     type hint, saying this __init__ doesn't hand back a value
        #     (there's no `return something` anywhere inside it). Python
        #     constructors are never allowed to return anything other than
        #     None anyway, so this is really just making that fact visible
        #     to a reader. Most methods in this project are annotated this
        #     way; the ones that DO return something say so instead -- see
        #     isFinished() below, which returns a `bool`.
        #   * `super().__init__()` calls Command's OWN constructor first,
        #     before this class does anything else. Every Command subclass
        #     in this project starts with this exact line -- skipping it
        #     would leave the underlying commands2.Command machinery
        #     half-set-up, so it always has to come first.
        super().__init__()
        self._gripper = gripper
        self.addRequirements(gripper)

    def execute(self) -> None:
        self._gripper.set_speed(GripperConstants.INTAKE_SPEED)

    def isFinished(self) -> bool:
        # `-> bool` means this method hands back either True or False --
        # the CommandScheduler calls isFinished() every loop while this
        # command is running and ends the command the moment it returns
        # True. Always returning False here means "never finish on your
        # own" -- this command is meant to be bound with whileTrue (see
        # robotcontainer.py), so it only stops when the button is released,
        # which the scheduler handles by calling end() below instead.
        return False

    def end(self, interrupted: bool) -> None:
        # `interrupted: bool` is a parameter the CommandScheduler fills in
        # for you: True if this command got cut off early (the button was
        # released, another command that needs Gripper started, the robot
        # got disabled), False if it ended because isFinished() returned
        # True on its own. This command doesn't need to tell those two
        # cases apart -- either way, the roller should stop -- but end()
        # always receives this parameter, even when a command ignores it
        # like this one does.
        self._gripper.stop()


class EjectCommand(Command):
    def __init__(self, gripper: Gripper) -> None:
        # Same pattern as IntakeCommand.__init__ above -- see its comments
        # for what `self`, `gripper: Gripper`, `-> None`, and
        # `super().__init__()` each mean.
        super().__init__()
        self._gripper = gripper
        self.addRequirements(gripper)

    def execute(self) -> None:
        self._gripper.set_speed(GripperConstants.EJECT_SPEED)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self._gripper.stop()
