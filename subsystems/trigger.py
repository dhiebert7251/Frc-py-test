"""Trigger subsystem -- small NEO-driven cam that flicks a game piece into the shooter.

Teaching-bot proof of concept. The cam motor runs one full revolution per
"fire" -- the limit switch defines the cam's rest ("home") position, and
firing means: leave home, then come back to home. Two beam-break sensors
report loading status further upstream (not directly tied to the cam).
"""
from __future__ import annotations

import rev
import wpilib
from commands2 import Command, Subsystem, cmd

from constants import TriggerConstants


class Trigger(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._cam_motor = rev.SparkMax(TriggerConstants.CAM_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        cam_config = rev.SparkMaxConfig()
        cam_config.inverted(TriggerConstants.CAM_MOTOR_INVERTED)
        cam_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        cam_config.smartCurrentLimit(TriggerConstants.CAM_CURRENT_LIMIT)
        self._cam_motor.configure(cam_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self._limit_switch = wpilib.DigitalInput(TriggerConstants.LIMIT_SWITCH_DIO_PORT)
        self._beam_break_1 = wpilib.DigitalInput(TriggerConstants.BEAM_BREAK_1_DIO_PORT)
        self._beam_break_2 = wpilib.DigitalInput(TriggerConstants.BEAM_BREAK_2_DIO_PORT)

    def is_at_home(self) -> bool:
        raw = self._limit_switch.get()
        return not raw if TriggerConstants.LIMIT_SWITCH_INVERTED else raw

    def has_ball_at_stage_1(self) -> bool:
        raw = self._beam_break_1.get()
        return not raw if TriggerConstants.BEAM_BREAK_1_INVERTED else raw

    def has_ball_at_stage_2(self) -> bool:
        raw = self._beam_break_2.get()
        return not raw if TriggerConstants.BEAM_BREAK_2_INVERTED else raw

    def run_cam(self) -> None:
        self._cam_motor.set(TriggerConstants.CAM_FIRE_SPEED)

    def stop_cam(self) -> None:
        self._cam_motor.set(0.0)

    def fire_command(self) -> Command:
        """Runs the cam motor for one full revolution: leaves the home
        (limit-switch) position, then stops as soon as it returns to home.
        Has a timeout in case the limit switch never re-triggers (a jam or a
        broken wire), so this command can never run the motor forever."""
        has_left_home = False

        def _init() -> None:
            nonlocal has_left_home
            has_left_home = False

        def _finished() -> bool:
            nonlocal has_left_home
            if not has_left_home:
                if not self.is_at_home():
                    has_left_home = True
                return False
            return self.is_at_home()

        return (
            cmd.sequence(
                cmd.runOnce(_init, self),
                cmd.run(self.run_cam, self).until(_finished),
            )
            .withTimeout(TriggerConstants.FIRE_TIMEOUT_SECONDS)
            .finallyDo(lambda interrupted: self.stop_cam())
        )

    def periodic(self) -> None:
        wpilib.SmartDashboard.putBoolean("Trigger/AtHome", self.is_at_home())
        wpilib.SmartDashboard.putBoolean("Trigger/BallStage1", self.has_ball_at_stage_1())
        wpilib.SmartDashboard.putBoolean("Trigger/BallStage2", self.has_ball_at_stage_2())
