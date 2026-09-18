"""Trigger subsystem -- small NEO-driven cam that flicks a game piece into the shooter.

Teaching-bot proof of concept. The cam has exactly one sensor: a limit
switch that defines its rest ("home") position. This subsystem only
exposes plain actions/queries (run the cam motor, read the switch/beam
breaks) -- the "run until it's fired one full revolution" logic is real
state-machine behavior, so it lives in its own Command class,
commands/trigger_commands.py's FireCommand, rather than here.
"""
from __future__ import annotations

import wpilib
from commands2 import Subsystem
from rev import ResetMode, PersistMode, SparkBaseConfig, SparkMax, SparkMaxConfig

from constants import TriggerConstants


class Trigger(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._cam_motor = SparkMax(TriggerConstants.CAM_MOTOR_ID, SparkMax.MotorType.kBrushless)
        cam_config = SparkMaxConfig()
        cam_config.inverted(TriggerConstants.CAM_MOTOR_INVERTED)
        # Brake mode (not DriveTrain's Coast): when the cam motor is
        # commanded to 0, we want it to stop and hold position immediately,
        # not coast -- an idle cam swinging freely could drift off "home"
        # and throw off the next fire cycle's home-switch reading.
        cam_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        cam_config.smartCurrentLimit(TriggerConstants.CAM_CURRENT_LIMIT)
        self._cam_motor.configure(cam_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        # DigitalInput reads a single digital (on/off) signal from a roboRIO
        # DIO port -- the same class WPILib uses for any simple switch or
        # break-beam sensor, since electrically they're the same thing (a
        # circuit that's either open or closed).
        self._limit_switch = wpilib.DigitalInput(TriggerConstants.LIMIT_SWITCH_DIO_PORT)
        self._beam_break_1 = wpilib.DigitalInput(TriggerConstants.BEAM_BREAK_1_DIO_PORT)
        self._beam_break_2 = wpilib.DigitalInput(TriggerConstants.BEAM_BREAK_2_DIO_PORT)

    def is_at_home(self) -> bool:
        # Every switch/beam-break getter here follows the same shape:
        # read the raw electrical signal, then flip it if that particular
        # sensor's wiring reports "true" for the opposite of what we mean
        # (see the *_INVERTED constants and their TODOs -- this is exactly
        # the kind of thing that must be checked on the real robot, since
        # guessing wrong here silently inverts the sensor's meaning).
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

    def periodic(self) -> None:
        wpilib.SmartDashboard.putBoolean("Trigger/AtHome", self.is_at_home())
        wpilib.SmartDashboard.putBoolean("Trigger/BallStage1", self.has_ball_at_stage_1())
        wpilib.SmartDashboard.putBoolean("Trigger/BallStage2", self.has_ball_at_stage_2())
