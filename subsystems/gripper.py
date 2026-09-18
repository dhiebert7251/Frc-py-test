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
        # No parameters besides `self` here -- Gripper doesn't need
        # anything handed to it from the outside to build itself; every
        # value it needs (motor CAN ID, current limit, ...) comes from
        # GripperConstants instead. Compare this to
        # commands/gripper_commands.py's IntakeCommand.__init__, which DOES
        # take a parameter (`gripper: Gripper`) because a command needs to
        # be told WHICH Gripper object to act on.
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
        # `speed: float` is the same `name: type` pattern used for every
        # parameter in this project, just with a plain number type instead
        # of one of our own classes -- see
        # commands/drivetrain_commands.py's DriveDistanceCommand.__init__
        # for another example (`distance_feet: float`) with a comment
        # calling this out explicitly.
        self._roller_motor.set(speed)

    def stop(self) -> None:
        self._roller_motor.set(0.0)
