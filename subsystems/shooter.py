"""Shooter subsystem -- single flywheel (Kraken/TalonFX), fixed target RPM.

Teaching-bot proof of concept. Simpler than the competition bot's Shooter:
no distance-based RPM table (no vision on this robot) -- just a single
configurable target speed. Demonstrates the same Phoenix 6 velocity-control
pattern the competition bot uses.
"""
from __future__ import annotations

import wpilib
from commands2 import Command, Subsystem, cmd
from phoenix6 import StatusCode
from phoenix6.configs import CurrentLimitsConfigs, MotorOutputConfigs, Slot0Configs, TalonFXConfiguration
from phoenix6.controls import NeutralOut, VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from constants import ShooterConstants


class Shooter(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._velocity_request = VelocityVoltage(0).with_slot(0)
        self._neutral_request = NeutralOut()

        self._target_rpm = 0.0

        self._flywheel_motor = TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID)
        flywheel_config = (
            TalonFXConfiguration()
            .with_motor_output(
                MotorOutputConfigs()
                .with_neutral_mode(NeutralModeValue.COAST)
                .with_inverted(
                    InvertedValue.CLOCKWISE_POSITIVE
                    if ShooterConstants.FLYWHEEL_INVERTED
                    else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                )
            )
            .with_current_limits(
                CurrentLimitsConfigs()
                .with_stator_current_limit(ShooterConstants.CURRENT_LIMIT)
                .with_stator_current_limit_enable(True)
            )
            .with_slot0(
                Slot0Configs()
                .with_k_p(ShooterConstants.SHOOTER_KP)
                .with_k_i(ShooterConstants.SHOOTER_KI)
                .with_k_d(ShooterConstants.SHOOTER_KD)
                .with_k_v(ShooterConstants.SHOOTER_KV)
            )
        )
        config_error: StatusCode = self._flywheel_motor.configurator.apply(flywheel_config)
        if not config_error.is_ok():
            wpilib.reportWarning(f"Shooter flywheel motor config failed: {config_error}")

    def set_target_rpm(self, rpm: float) -> None:
        self._target_rpm = rpm
        self._flywheel_motor.set_control(
            self._velocity_request.with_velocity(rpm / ShooterConstants.FLYWHEEL_GEAR_RATIO / 60.0)
        )

    def stop(self) -> None:
        self._target_rpm = 0.0
        self._flywheel_motor.set_control(self._neutral_request)

    def get_current_rpm(self) -> float:
        return self._flywheel_motor.get_velocity().value_as_double * 60.0 * ShooterConstants.FLYWHEEL_GEAR_RATIO

    def is_at_target_speed(self) -> bool:
        return self._target_rpm > 0 and abs(self.get_current_rpm() - self._target_rpm) <= ShooterConstants.RPM_TOLERANCE

    def spin_up_command(self) -> Command:
        """Toggle: spins the flywheel to ShooterConstants.TARGET_RPM, or coasts
        to a stop. Uses cmd.startEnd() rather than cmd.run() -- Phoenix 6's
        velocity control is closed-loop on the motor controller itself, so we
        only need to command it once at start and once at stop, unlike an
        open-loop duty-cycle command that must be re-commanded every loop."""
        return cmd.startEnd(
            lambda: self.set_target_rpm(ShooterConstants.TARGET_RPM),
            self.stop,
            self,
        )

    def periodic(self) -> None:
        wpilib.SmartDashboard.putNumber("Shooter/CurrentRPM", self.get_current_rpm())
        wpilib.SmartDashboard.putNumber("Shooter/TargetRPM", self._target_rpm)
        wpilib.SmartDashboard.putBoolean("Shooter/AtSpeed", self.is_at_target_speed())
