"""Elevator subsystem -- 2-stage single-mast elevator (AndyMark "Elevator in
a Box" style cascade rig), spring-assisted extension, motor+rope retraction.

Teaching-bot proof of concept. The Redline motor here is brushed and has no
encoder (a real one could add a through-bore/versa encoder later for
closed-loop positioning -- see README) -- this subsystem is entirely
open-loop, driven by a limit switch at each end of travel. Raising needs
less motor power because the springs are doing most of the work; lowering
needs the motor to actively pull the rope in against that same spring
tension.
"""
from __future__ import annotations

import rev
import wpilib
from commands2 import Command, Subsystem, cmd

from constants import ElevatorConstants


class Elevator(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._lift_motor = rev.SparkMax(ElevatorConstants.LIFT_MOTOR_ID, rev.SparkMax.MotorType.kBrushed)
        lift_config = rev.SparkMaxConfig()
        lift_config.inverted(ElevatorConstants.LIFT_MOTOR_INVERTED)
        lift_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        lift_config.smartCurrentLimit(ElevatorConstants.LIFT_CURRENT_LIMIT)
        self._lift_motor.configure(lift_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self._top_limit_switch = wpilib.DigitalInput(ElevatorConstants.TOP_LIMIT_SWITCH_DIO_PORT)
        self._bottom_limit_switch = wpilib.DigitalInput(ElevatorConstants.BOTTOM_LIMIT_SWITCH_DIO_PORT)

    def is_at_top(self) -> bool:
        raw = self._top_limit_switch.get()
        return not raw if ElevatorConstants.TOP_LIMIT_SWITCH_INVERTED else raw

    def is_at_bottom(self) -> bool:
        raw = self._bottom_limit_switch.get()
        return not raw if ElevatorConstants.BOTTOM_LIMIT_SWITCH_INVERTED else raw

    def stop(self) -> None:
        self._lift_motor.set(0.0)

    def raise_command(self) -> Command:
        """Raises while held, stopping automatically at the top even if the
        button is still held (protects the mechanism -- always stop at a
        limit switch, whether the command is "manual" or not)."""

        def _run() -> None:
            if self.is_at_top():
                self.stop()
            else:
                self._lift_motor.set(ElevatorConstants.RAISE_SPEED)

        return cmd.run(_run, self).finallyDo(lambda interrupted: self.stop())

    def lower_command(self) -> Command:
        """Lowers while held, stopping automatically at the bottom."""

        def _run() -> None:
            if self.is_at_bottom():
                self.stop()
            else:
                self._lift_motor.set(ElevatorConstants.LOWER_SPEED)

        return cmd.run(_run, self).finallyDo(lambda interrupted: self.stop())

    def periodic(self) -> None:
        wpilib.SmartDashboard.putBoolean("Elevator/AtTop", self.is_at_top())
        wpilib.SmartDashboard.putBoolean("Elevator/AtBottom", self.is_at_bottom())
