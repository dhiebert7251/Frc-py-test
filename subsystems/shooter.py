"""Shooter subsystem -- single flywheel (Kraken/TalonFX), fixed target RPM.

Teaching-bot proof of concept. Simpler than the competition bot's Shooter:
no distance-based RPM table (no vision on this robot) -- just a single
configurable target speed. Demonstrates Phoenix 6's velocity-control
pattern, and -- unlike DriveTrain's motors -- native snake_case naming: CTRE
chose to write phoenix6's Python bindings idiomatically instead of mirroring
their own Java API 1:1, which is why this file (and only this file) doesn't
mix camelCase vendor calls with our snake_case code the way every other
subsystem here does.

Like the other subsystems, this file only exposes plain hardware actions
(set_target_rpm(), stop(), the getters) -- the actual Command that uses them
lives in commands/shooter_commands.py.
"""
from __future__ import annotations

import wpilib
from commands2 import Subsystem
from phoenix6 import StatusCode
from phoenix6.configs import CurrentLimitsConfigs, MotorOutputConfigs, Slot0Configs, TalonFXConfiguration
from phoenix6.controls import NeutralOut, VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from constants import ShooterConstants


class Shooter(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # VelocityVoltage and NeutralOut are "control request" objects:
        # instead of calling a method with new arguments every loop (like
        # SparkMax's .set()), Phoenix 6 wants you to build one request object
        # per control mode and re-send it (via set_control(), below)
        # whenever you want to change or refresh what the motor is doing.
        # with_slot(0) picks which of the TalonFX's internal PID gain slots
        # (configured below as slot 0) this velocity request should use.
        self._velocity_request = VelocityVoltage(0).with_slot(0)
        self._neutral_request = NeutralOut()

        self._target_rpm = 0.0

        self._flywheel_motor = TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID)

        # Phoenix 6 configuration is one big object built up with chained
        # `.with_*()` calls (each one returns the same object back, which is
        # what lets them chain), then applied in one shot via
        # configurator.apply() below -- REVLib's SparkMaxConfig from
        # DriveTrain/Trigger/Elevator/Gripper is the same "build a config
        # object, then apply it" idea, just with REV's own method-naming
        # style instead of CTRE's.
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
                # Slot0Configs holds the PID(+velocity feedforward) gains the
                # TalonFX itself uses to run its OWN closed velocity loop, in
                # hardware, every control cycle -- much faster than this
                # Python code's ~20ms loop could. This is different from
                # DriveTrain's PID commands, which run the PID math in
                # Python and only send a duty cycle to the motor.
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
        """Commands the flywheel to spin at `rpm`. Because this is a
        closed-loop velocity request handled on the TalonFX itself (see the
        Slot0Configs comment above), this only needs to be called once when
        the target changes -- not every loop like an open-loop duty cycle
        motor would need."""
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

    def periodic(self) -> None:
        # RPM has no separate "imperial" form the way a distance does, so
        # unlike DriveTrain's telemetry there's no unit conversion to do here.
        wpilib.SmartDashboard.putNumber("Shooter/CurrentRPM", self.get_current_rpm())
        wpilib.SmartDashboard.putNumber("Shooter/TargetRPM", self._target_rpm)
        wpilib.SmartDashboard.putBoolean("Shooter/AtSpeed", self.is_at_target_speed())
