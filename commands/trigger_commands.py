"""Commands for Trigger.

FireCommand is the one genuinely tricky piece of logic in this whole
codebase, and it's a good example of why "just read the one sensor" isn't
always enough: the cam has exactly one sensor, a limit switch at "home,"
and reading it once at the start would immediately (and wrongly) report
"done," since the cam starts each fire cycle already at home. A full fire
cycle actually means: leave home, THEN come back to home. isFinished()
below tracks that as one bit of state, self._has_left_home, which is reset
every time this command starts over in initialize().

Writing this as an explicit class (instead of the nonlocal-closure version
an earlier draft of this file used) makes that one bit of state a plain,
named instance attribute instead of a variable captured in a closure --
worth comparing side by side with a mentor if this is the first stateful
command a rookie reads.
"""
from __future__ import annotations

from commands2 import Command

from subsystems.trigger import Trigger


class FireCommand(Command):
    """Runs the cam motor until it leaves and then returns to the home
    (limit-switch) position -- one full revolution.

    This class does NOT set its own timeout. A jammed cam or a broken
    switch wire means it would never see "returned home" and would run
    forever on its own; the safety timeout is applied as a `.withTimeout()`
    decorator at the one place this command is actually bound to a button,
    in robotcontainer.py. Decorators like `.withTimeout()`, `.andThen()`,
    and `.until()` work on ANY Command -- an explicit class like this one
    just as well as a `cmd.run(...)` one-liner -- which is why it doesn't
    matter that FireCommand and, say, DriveDistanceCommand build their
    behavior in very different ways internally.
    """

    def __init__(self, trigger: Trigger) -> None:
        # Same `self` / `trigger: Trigger` / `-> None` / `super().__init__()`
        # pattern as every other command's __init__ in this project -- see
        # commands/gripper_commands.py's IntakeCommand for the full
        # explanation. `self._has_left_home = False` here is a plain bool
        # (True/False) instance attribute, not a parameter -- it's not
        # something the caller passes in, just a starting value this
        # object keeps track of for itself, re-set every time initialize()
        # below runs.
        super().__init__()
        self._trigger = trigger
        self.addRequirements(trigger)
        self._has_left_home = False

    def initialize(self) -> None:
        self._has_left_home = False

    def execute(self) -> None:
        self._trigger.run_cam()

    def isFinished(self) -> bool:
        if not self._has_left_home:
            # Still waiting for the cam to leave home for the first time --
            # once it does, remember that and start watching for it to come
            # back.
            if not self._trigger.is_at_home():
                self._has_left_home = True
            return False
        return self._trigger.is_at_home()

    def end(self, interrupted: bool) -> None:
        self._trigger.stop_cam()
