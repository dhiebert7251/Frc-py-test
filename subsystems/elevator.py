"""Elevator subsystem -- 2-stage single-mast elevator (AndyMark "Elevator in
a Box" style cascade rig), spring-assisted extension, motor+rope retraction.

Teaching-bot proof of concept. The Redline motor here is brushed and has no
encoder (a real one could add a through-bore/versa encoder later for
closed-loop positioning -- see README) -- this subsystem is entirely
open-loop, driven only by a limit switch at each end of travel. Raising
needs less motor power because the springs are doing most of the work;
lowering needs the motor to actively pull the rope in against that same
spring tension.

This subsystem only exposes plain actions (raise/lower a notch, stop,
check the limit switches) -- the "keep raising/lowering while a button is
held, but always stop at a limit switch even if the button is still held"
behavior lives in commands/elevator_commands.py.
"""
from __future__ import annotations

import wpilib
from commands2 import Subsystem
from rev import ResetMode, PersistMode, SparkBaseConfig, SparkMax, SparkMaxConfig

from constants import ElevatorConstants


class Elevator(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # kBrushed, not kBrushless: a Redline motor has physical brushes
        # (hence the name) and no built-in encoder, unlike every NEO in this
        # project. SparkMax can drive either motor type, but has to be told
        # which one it's talking to, since brushed and brushless motors are
        # commutated (have their windings energized in sequence) completely
        # differently in hardware.
        self._lift_motor = SparkMax(ElevatorConstants.LIFT_MOTOR_ID, SparkMax.MotorType.kBrushed)
        lift_config = SparkMaxConfig()
        lift_config.inverted(ElevatorConstants.LIFT_MOTOR_INVERTED)
        lift_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        lift_config.smartCurrentLimit(ElevatorConstants.LIFT_CURRENT_LIMIT)
        self._lift_motor.configure(lift_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        self._top_limit_switch = wpilib.DigitalInput(ElevatorConstants.TOP_LIMIT_SWITCH_DIO_PORT)
        self._bottom_limit_switch = wpilib.DigitalInput(ElevatorConstants.BOTTOM_LIMIT_SWITCH_DIO_PORT)

    def is_at_top(self) -> bool:
        raw = self._top_limit_switch.get()
        return not raw if ElevatorConstants.TOP_LIMIT_SWITCH_INVERTED else raw

    def is_at_bottom(self) -> bool:
        raw = self._bottom_limit_switch.get()
        return not raw if ElevatorConstants.BOTTOM_LIMIT_SWITCH_INVERTED else raw

    def set_speed(self, speed: float) -> None:
        """speed is a duty cycle in [-1, 1]: positive raises, negative
        lowers -- see ElevatorConstants.RAISE_SPEED/LOWER_SPEED."""
        self._lift_motor.set(speed)

    def stop(self) -> None:
        self._lift_motor.set(0.0)

    def periodic(self) -> None:
        wpilib.SmartDashboard.putBoolean("Elevator/AtTop", self.is_at_top())
        wpilib.SmartDashboard.putBoolean("Elevator/AtBottom", self.is_at_bottom())
