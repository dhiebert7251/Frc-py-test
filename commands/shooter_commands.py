"""Commands for Shooter.

Only one command: spin the flywheel up to a fixed target speed, and hold
it there until interrupted. Notice this class has the exact same shape
(initialize/isFinished/end, no execute()) as ElevatorCommands' raise/lower
would if they didn't need to keep re-checking a limit switch every loop --
that's not a coincidence. It reflects a real difference in the hardware:
Shooter's flywheel speed is a closed loop running on the TalonFX itself
(see subsystems/shooter.py), so this command only has to say "go to this
speed" once and "stop" once. An open-loop, duty-cycle motor (DriveTrain's
teleop drive, Elevator, Gripper) has no such loop running on the motor
controller, so those commands DO need to keep re-commanding a speed every
execute() -- there's nothing holding that speed steady except this code
calling it repeatedly.
"""
from __future__ import annotations

from commands2 import Command

from constants import ShooterConstants
from subsystems.shooter import Shooter


class SpinUpShooterCommand(Command):
    def __init__(self, shooter: Shooter) -> None:
        super().__init__()
        self._shooter = shooter
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self._shooter.set_target_rpm(ShooterConstants.TARGET_RPM)

    def isFinished(self) -> bool:
        # Runs until interrupted (the operator presses the toggle button
        # again -- see robotcontainer.py's toggleOnTrue binding).
        return False

    def end(self, interrupted: bool) -> None:
        self._shooter.stop()
