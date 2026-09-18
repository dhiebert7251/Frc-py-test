"""Feeder subsystem -- owns the intake roller (CAN 31) and trigger/hopper (CAN 32).

Ported from Feeder.java. Separated from the Shooter subsystem so intake/eject
commands can run concurrently with the shooter wheel spinning (they require
different subsystems).

Jam detection is implemented but disabled by default, matching the Java
version (update_jam_detection()/update_jam_clear() are not called from
periodic()).
"""

from enum import Enum, auto

import rev
import wpilib
from commands2 import Command, Subsystem, cmd

from constants import SensorConstants, ShooterConstants


class FeederState(Enum):
    IDLE = auto()
    INTAKE = auto()
    FEED = auto()
    EJECT = auto()
    JAM_CLEAR = auto()


class Feeder(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._current_state = FeederState.IDLE

        # Jam detection (monitored on trigger motor -- most likely jam point)
        self._spike_debounce_timer = wpilib.Timer()
        self._spike_debounce_running = False
        self._jam_reverse_timer = wpilib.Timer()
        self._state_before_jam = FeederState.IDLE

        self._telemetry_loop_counter = 0

        # SparkMAX intake roller -- CAN 31, open-loop, CCW = into robot
        self._intake_motor = rev.SparkMax(ShooterConstants.INTAKE_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        intake_config = rev.SparkMaxConfig()
        intake_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        intake_config.smartCurrentLimit(ShooterConstants.INTAKE_MOTOR_CURRENT_LIMIT)
        intake_config.inverted(ShooterConstants.INTAKE_MOTOR_INVERTED)
        intake_config.voltageCompensation(ShooterConstants.NOMINAL_VOLTAGE)
        intake_config.openLoopRampRate(0.15)
        intake_config_error = self._intake_motor.configure(
            intake_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )
        if intake_config_error != rev.REVLibError.kOk:
            wpilib.reportWarning(f"Intake motor (CAN 31) config failed: {intake_config_error}")

        self._intake_motor.clearFaults()

        # SparkMAX trigger/hopper -- CAN 32, open-loop, bidirectional
        self._trigger_motor = rev.SparkMax(ShooterConstants.TRIGGER_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        trigger_config = rev.SparkMaxConfig()
        trigger_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        trigger_config.smartCurrentLimit(ShooterConstants.TRIGGER_MOTOR_CURRENT_LIMIT)
        trigger_config.inverted(ShooterConstants.TRIGGER_MOTOR_INVERTED)
        trigger_config.voltageCompensation(ShooterConstants.NOMINAL_VOLTAGE)
        trigger_config.openLoopRampRate(0.15)
        trigger_config_error = self._trigger_motor.configure(
            trigger_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters
        )
        if trigger_config_error != rev.REVLibError.kOk:
            wpilib.reportWarning(f"Trigger motor (CAN 32) config failed: {trigger_config_error}")

        self._trigger_motor.clearFaults()

        # Photo sensor -- skip DIO allocation until the sensor is installed
        self._photo_sensor = (
            wpilib.DigitalInput(SensorConstants.PHOTO_SENSOR_DIO_PORT)
            if SensorConstants.PHOTO_SENSOR_ENABLED
            else None
        )

    # ---- Motor control ----

    def start_feed(self) -> None:
        """Run intake roller and trigger in feed direction (hopper -> shooter)."""
        self._current_state = FeederState.FEED
        self._intake_motor.set(ShooterConstants.INTAKE_SPEED)
        self._trigger_motor.set(ShooterConstants.TRIGGER_FEED_SPEED)

    def stop_all(self) -> None:
        """Stop both motors and return to idle. Always stops even during JAM_CLEAR."""
        self._intake_motor.set(0.0)
        self._trigger_motor.set(0.0)
        self._current_state = FeederState.IDLE

    # ---- Sensor ----

    def has_ball(self) -> bool:
        if self._photo_sensor is None:
            return False
        raw = self._photo_sensor.get()
        return not raw if SensorConstants.PHOTO_SENSOR_INVERTED else raw

    # ---- State access ----

    def get_state(self) -> FeederState:
        return self._current_state

    # ---- Jam detection ----

    def _update_jam_detection(self) -> None:
        if self._current_state == FeederState.JAM_CLEAR:
            return
        if self._current_state not in (FeederState.INTAKE, FeederState.FEED):
            if self._spike_debounce_running:
                self._spike_debounce_timer.stop()
                self._spike_debounce_running = False
            return

        spiking = self._trigger_motor.getOutputCurrent() > ShooterConstants.TRIGGER_SPIKE_THRESHOLD_AMPS
        if spiking:
            if not self._spike_debounce_running:
                self._spike_debounce_timer.reset()
                self._spike_debounce_timer.start()
                self._spike_debounce_running = True
            elif self._spike_debounce_timer.hasElapsed(0.1):
                self._trigger_jam_clear()
        else:
            self._spike_debounce_timer.stop()
            self._spike_debounce_running = False

    def _trigger_jam_clear(self) -> None:
        self._state_before_jam = self._current_state
        self._current_state = FeederState.JAM_CLEAR
        self._spike_debounce_running = False
        self._spike_debounce_timer.stop()
        self._jam_reverse_timer.reset()
        self._jam_reverse_timer.start()
        self._intake_motor.set(0.0)
        self._trigger_motor.set(ShooterConstants.JAM_REVERSE_SPEED)

    def _update_jam_clear(self) -> None:
        if self._current_state != FeederState.JAM_CLEAR:
            return
        if self._jam_reverse_timer.hasElapsed(ShooterConstants.JAM_REVERSE_TIME_SEC):
            self._jam_reverse_timer.stop()
            self._current_state = self._state_before_jam
            if self._current_state == FeederState.INTAKE:
                self._intake_motor.set(ShooterConstants.INTAKE_SPEED)
                self._trigger_motor.set(ShooterConstants.TRIGGER_INTAKE_SPEED)
            elif self._current_state == FeederState.FEED:
                self._intake_motor.set(ShooterConstants.INTAKE_SPEED)
                self._trigger_motor.set(ShooterConstants.TRIGGER_FEED_SPEED)
            else:
                self.stop_all()

    # ---- Commands ----

    def intake_command(self) -> Command:
        """Intake -- draw ball in from ground. Toggle with toggleOnTrue().
        Requires only Feeder, so it runs concurrently with the shooter wheel spinning.
        Uses run() so motors are re-commanded every 20 ms, preventing stalls from
        a single missed or current-limited startup command."""

        def _run() -> None:
            self._current_state = FeederState.INTAKE
            self._intake_motor.set(ShooterConstants.INTAKE_SPEED)
            self._trigger_motor.set(ShooterConstants.TRIGGER_INTAKE_SPEED)

        return cmd.run(_run, self).finallyDo(lambda interrupted: self.stop_all())

    def eject_command(self) -> Command:
        """Removal -- reverse both motors to expel ball. Toggle with toggleOnTrue().
        Requires only Feeder -- runs concurrently with Y spin-up."""

        def _run() -> None:
            self._current_state = FeederState.EJECT
            self._intake_motor.set(ShooterConstants.INTAKE_EJECT_SPEED)
            self._trigger_motor.set(ShooterConstants.TRIGGER_EJECT_SPEED)

        return cmd.run(_run, self).finallyDo(lambda interrupted: self.stop_all())

    def shoot_command(self) -> Command:
        """Feed ball to shooter -- intake holds ball while trigger drives toward
        shooter wheel."""

        def _run() -> None:
            self._current_state = FeederState.FEED
            self._intake_motor.set(ShooterConstants.INTAKE_SPEED)
            self._trigger_motor.set(ShooterConstants.TRIGGER_EJECT_SPEED)  # was TRIGGER_FEED_SPEED

        return cmd.run(_run, self).finallyDo(lambda interrupted: self.stop_all())

    # ---- Periodic ----

    def periodic(self) -> None:
        # Jam clear temporarily disabled -- matches Java source.
        # self._update_jam_detection()
        # self._update_jam_clear()
        self._telemetry_loop_counter += 1
        if self._telemetry_loop_counter >= ShooterConstants.SHOOTER_TELEMETRY_PERIOD_LOOPS:
            self._telemetry_loop_counter = 0
            self._log_telemetry()

    def _log_telemetry(self) -> None:
        wpilib.SmartDashboard.putBoolean("Feeder/Intake Active", self._current_state == FeederState.INTAKE)
        wpilib.SmartDashboard.putBoolean("Feeder/Eject Active", self._current_state == FeederState.EJECT)
        wpilib.SmartDashboard.putString("Feeder/State", self._current_state.name)
        wpilib.SmartDashboard.putNumber("Feeder/Intake Current (A)", self._intake_motor.getOutputCurrent())
        wpilib.SmartDashboard.putNumber("Feeder/Trigger Current (A)", self._trigger_motor.getOutputCurrent())
