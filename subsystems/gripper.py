"""Gripper subsystem -- spinning roller intake at the end of the elevator.

Teaching-bot proof of concept. Same intake/eject pattern as the competition
bot's Feeder subsystem, just one motor instead of two.
"""
from __future__ import annotations

import rev
from commands2 import Command, Subsystem, cmd

from constants import GripperConstants


class Gripper(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._roller_motor = rev.SparkMax(GripperConstants.ROLLER_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        roller_config = rev.SparkMaxConfig()
        roller_config.inverted(GripperConstants.ROLLER_MOTOR_INVERTED)
        roller_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        roller_config.smartCurrentLimit(GripperConstants.ROLLER_CURRENT_LIMIT)
        self._roller_motor.configure(
            roller_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )

    def stop(self) -> None:
        self._roller_motor.set(0.0)

    def intake_command(self) -> Command:
        return cmd.run(lambda: self._roller_motor.set(GripperConstants.INTAKE_SPEED), self).finallyDo(
            lambda interrupted: self.stop()
        )

    def eject_command(self) -> Command:
        return cmd.run(lambda: self._roller_motor.set(GripperConstants.EJECT_SPEED), self).finallyDo(
            lambda interrupted: self.stop()
        )
