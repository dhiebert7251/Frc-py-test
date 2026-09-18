"""Gripper subsystem -- spinning roller intake at the end of the elevator.

Teaching-bot proof of concept. The simplest subsystem here: one motor, no
sensors at all. set_speed()/stop() are the only two things it knows how to
do -- commands/gripper_commands.py's IntakeCommand/EjectCommand just pick
which speed to hold while a button is pressed.
"""
from __future__ import annotations

from commands2 import Subsystem
from rev import ResetMode, PersistMode, SparkBaseConfig, SparkMax, SparkMaxConfig

from constants import GripperConstants


class Gripper(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._roller_motor = SparkMax(GripperConstants.ROLLER_MOTOR_ID, SparkMax.MotorType.kBrushless)
        roller_config = SparkMaxConfig()
        roller_config.inverted(GripperConstants.ROLLER_MOTOR_INVERTED)
        roller_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        roller_config.smartCurrentLimit(GripperConstants.ROLLER_CURRENT_LIMIT)
        self._roller_motor.configure(roller_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

    def set_speed(self, speed: float) -> None:
        """speed is a duty cycle in [-1, 1]: positive intakes, negative
        ejects -- see GripperConstants.INTAKE_SPEED/EJECT_SPEED."""
        self._roller_motor.set(speed)

    def stop(self) -> None:
        self._roller_motor.set(0.0)
