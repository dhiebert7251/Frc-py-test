"""Shooter subsystem -- owns only the flywheel (CAN 30 TalonFX) and distance/zone logic.

Ported from Shooter.java. Intake and trigger motors live in the Feeder
subsystem so that intake/eject commands can run concurrently with the
shooter wheel spinning.

Note: phoenix6's 2026 Python bindings use snake_case (with_k_p, get_velocity,
set_control, ...) rather than the Java camelCase API -- this is the one
vendor library in this port that does not mirror Java naming.
"""

from __future__ import annotations

from typing import Optional

import wpilib
from commands2 import Command, Subsystem, cmd
from phoenix6 import StatusCode
from phoenix6.configs import (
    ClosedLoopRampsConfigs,
    CurrentLimitsConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import DutyCycleOut, NeutralOut, VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpimath.geometry import Pose2d

from constants import SensorConstants, ShooterConstants, VisionConstants
from vision_measurement import VisionMeasurement


class Shooter(Subsystem):
    def __init__(self, vision, drivetrain) -> None:
        super().__init__()

        self._vision = vision
        self._drivetrain = drivetrain

        # ---- Reusable control requests (avoid per-loop allocation) ----
        self._velocity_request = VelocityVoltage(0).with_slot(0)
        self._neutral_request = NeutralOut()
        self._duty_cycle_request = DutyCycleOut(0)

        # ---- State ----
        self._target_rpm = ShooterConstants.TARGET_RPM_10_FEET
        self._current_target_distance = 10.0

        # Cached tuning values -- apply() is only called when a value actually changes,
        # avoiding a blocking CAN write every loop iteration.
        self._cached_kp = ShooterConstants.SHOOTER_KP
        self._cached_ki = ShooterConstants.SHOOTER_KI
        self._cached_kd = ShooterConstants.SHOOTER_KD
        self._cached_kv = ShooterConstants.SHOOTER_KV

        self._pov_preset_set = False
        self._pov_preset_distance_ft = 10.0
        self._distance_source = "Default"
        self._vision_distance_ft = -1.0
        self._odometry_distance_ft = -1.0

        self._telemetry_loop_counter = 0

        # TalonFX shooter wheel -- CAN 30, Phoenix 6 on-controller velocity PID, slot 0
        self._shooter_motor = TalonFX(ShooterConstants.SHOOTER_MOTOR_ID)
        shooter_config = TalonFXConfiguration().with_motor_output(
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.COAST)
            .with_inverted(
                InvertedValue.CLOCKWISE_POSITIVE
                if ShooterConstants.SHOOTER_INVERTED
                else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
        ).with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(ShooterConstants.SHOOTER_CURRENT_LIMIT)
            .with_stator_current_limit_enable(True)
            .with_supply_current_limit(40)
            .with_supply_current_limit_enable(True)
        ).with_closed_loop_ramps(
            ClosedLoopRampsConfigs().with_voltage_closed_loop_ramp_period(0.25)
        ).with_slot0(
            Slot0Configs()
            .with_k_p(ShooterConstants.SHOOTER_KP)
            .with_k_i(ShooterConstants.SHOOTER_KI)
            .with_k_d(ShooterConstants.SHOOTER_KD)
            .with_k_v(ShooterConstants.SHOOTER_KV)
        )
        shooter_config_error: StatusCode = self._shooter_motor.configurator.apply(shooter_config)
        if not shooter_config_error.is_ok():
            wpilib.reportWarning(f"Shooter motor (CAN 30) config failed: {shooter_config_error}")

        # Seed SmartDashboard tuning entries with constant defaults
        wpilib.SmartDashboard.putNumber("Shooter/Tuning/kP", ShooterConstants.SHOOTER_KP)
        wpilib.SmartDashboard.putNumber("Shooter/Tuning/kI", ShooterConstants.SHOOTER_KI)
        wpilib.SmartDashboard.putNumber("Shooter/Tuning/kD", ShooterConstants.SHOOTER_KD)
        wpilib.SmartDashboard.putNumber("Shooter/Tuning/kV", ShooterConstants.SHOOTER_KV)

    # ---- Shooter wheel control ----

    def set_target_rpm(self, rpm: float) -> None:
        """Send a mechanism velocity target (RPM) to the TalonFX via Phoenix 6 velocity PID."""
        self._target_rpm = rpm
        self._shooter_motor.set_control(
            self._velocity_request.with_velocity(rpm / ShooterConstants.SHOOTER_GEAR_RATIO / 60.0)
        )

    def stop_shooter(self) -> None:
        """Coast the shooter wheel to a stop."""
        self._shooter_motor.set_control(self._neutral_request)

    def reverse_shooter(self) -> None:
        """Run shooter wheel in reverse at 50% duty cycle (unjam / back-spin)."""
        self._shooter_motor.set_control(self._duty_cycle_request.with_output(-0.5))

    def get_current_rpm(self) -> float:
        """Read actual shooter mechanism velocity in RPM (motor encoder RPM x gear ratio)."""
        return self._shooter_motor.get_velocity().value_as_double * 60.0 * ShooterConstants.SHOOTER_GEAR_RATIO

    def is_at_target_speed(self) -> bool:
        """True when the shooter is spinning and within RPM_TOLERANCE of the target."""
        return self._target_rpm > 0 and abs(self.get_current_rpm() - self._target_rpm) <= ShooterConstants.RPM_TOLERANCE

    def can_shoot(self, has_ball: bool) -> bool:
        """True when it is safe to feed. Zone/sensor check is currently bypassed,
        matching the Java source (`return true;`)."""
        return True

    # ---- Distance -> RPM table ----

    def get_rpm_from_distance(self, distance_feet: float) -> float:
        """Interpolate target RPM from the distance-to-RPM map."""
        distances = ShooterConstants.DISTANCES_FEET
        rpms = ShooterConstants.DISTANCE_RPM_MAP

        if distance_feet <= distances[0]:
            return rpms[0]
        if distance_feet >= distances[-1]:
            return rpms[-1]
        for i in range(len(distances) - 1):
            if distances[i] <= distance_feet <= distances[i + 1]:
                t = (distance_feet - distances[i]) / (distances[i + 1] - distances[i])
                return rpms[i] + t * (rpms[i + 1] - rpms[i])
        return rpms[-1]

    def set_distance_preset(self, distance_feet: float) -> None:
        """Pre-set the distance/RPM target without spinning the motor."""
        self._pov_preset_distance_ft = distance_feet
        self._current_target_distance = distance_feet
        self._target_rpm = self.get_rpm_from_distance(distance_feet)
        self._distance_source = "POV Preset"
        self._pov_preset_set = True

    def clear_distance_preset(self) -> None:
        self._pov_preset_set = False
        if self._distance_source == "POV Preset":
            self._distance_source = "Default"

    def get_target_distance(self) -> float:
        return self._current_target_distance

    # ---- Zone logic and distance resolution ----

    def _get_alliance_hub_pose(self) -> Pose2d:
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            return VisionConstants.RED_HUB_POSE
        return VisionConstants.BLUE_HUB_POSE

    def _is_vision_measurement_usable(self, measurement: VisionMeasurement) -> bool:
        return (
            measurement.num_tags_used != 1
            or measurement.best_target_ambiguity <= VisionConstants.MAX_AMBIGUITY
        )

    def can_spin_shooter(self) -> bool:
        """Returns true when the shooter is permitted to spin up. Zone restriction
        removed -- robot may shoot from the neutral zone (matches Java source)."""
        return True

    def _resolve_shooter_distance(self) -> None:
        """Resolve the best available shooting distance (feet).

        Autonomous: always 10 ft.
        Test mode: POV preset is locked for the session, else default 10 ft.
        Teleop: POV preset > vision > odometry > default 10 ft.
        """
        if wpilib.DriverStation.isAutonomous():
            self._current_target_distance = 10.0
            self._target_rpm = self.get_rpm_from_distance(10.0)
            self._distance_source = "Auto Default"
            self._vision_distance_ft = -1.0
            self._odometry_distance_ft = -1.0
            return

        hub_pose = self._get_alliance_hub_pose()

        # Always compute vision/odometry distances for telemetry, regardless of mode.
        self._vision_distance_ft = -1.0
        if self._vision is not None:
            fresh_measurement = self._vision.get_best_vision_measurement_if_fresh()
            if fresh_measurement is not None and self._is_vision_measurement_usable(fresh_measurement):
                meters = fresh_measurement.estimated_pose.translation().distance(hub_pose.translation())
                self._vision_distance_ft = meters / 0.3048

        self._odometry_distance_ft = -1.0
        if self._drivetrain is not None:
            meters = self._drivetrain.get_pose().translation().distance(hub_pose.translation())
            self._odometry_distance_ft = meters / 0.3048

        if wpilib.DriverStation.isTest():
            if self._pov_preset_set:
                self._current_target_distance = self._pov_preset_distance_ft
                self._target_rpm = self.get_rpm_from_distance(self._pov_preset_distance_ft)
                self._distance_source = "POV Preset (Test Lock)"
            else:
                self._current_target_distance = 10.0
                self._target_rpm = self.get_rpm_from_distance(10.0)
                self._distance_source = "Default"
            return

        # Priority 1: POV preset
        if self._pov_preset_set:
            self._current_target_distance = self._pov_preset_distance_ft
            self._target_rpm = self.get_rpm_from_distance(self._pov_preset_distance_ft)
            self._distance_source = "POV Preset"
            return

        # Priority 2: Vision
        if self._vision_distance_ft > 0:
            self._current_target_distance = self._vision_distance_ft
            self._target_rpm = self.get_rpm_from_distance(self._vision_distance_ft)
            self._distance_source = "Vision"
            return

        # Priority 3: Odometry
        if self._odometry_distance_ft > 0:
            self._current_target_distance = self._odometry_distance_ft
            self._target_rpm = self.get_rpm_from_distance(self._odometry_distance_ft)
            self._distance_source = "Odometry"
            return

        # Priority 4: Default
        self._current_target_distance = 10.0
        self._target_rpm = self.get_rpm_from_distance(10.0)
        self._distance_source = "Default"

    def resolve_distance_and_spin(self) -> None:
        """Resolve best distance and apply velocity command to the shooter wheel."""
        self._resolve_shooter_distance()
        self._shooter_motor.set_control(
            self._velocity_request.with_velocity(self._target_rpm / ShooterConstants.SHOOTER_GEAR_RATIO / 60.0)
        )

    # ---- Commands ----

    def spin_up_command(self) -> Command:
        """Spin up shooter wheel to distance-resolved RPM (toggle -- no feed).
        Requires only Shooter, so LB/LT/RB (Feeder commands) run concurrently."""

        def _run() -> None:
            if not self.can_spin_shooter():
                self.stop_shooter()
                return
            self.resolve_distance_and_spin()

        return cmd.run(_run, self).finallyDo(lambda interrupted: self.stop_shooter())

    # ---- Periodic ----

    def periodic(self) -> None:
        self._update_smartdashboard_tuning()
        self._telemetry_loop_counter += 1
        if self._telemetry_loop_counter >= ShooterConstants.SHOOTER_TELEMETRY_PERIOD_LOOPS:
            self._telemetry_loop_counter = 0
            self._log_telemetry()

    def _update_smartdashboard_tuning(self) -> None:
        """Read SmartDashboard PID values and push to TalonFX slot 0. Test mode only.
        apply() is a blocking CAN call (~10-100 ms) so it is only invoked when at
        least one value has actually changed."""
        if not wpilib.DriverStation.isTest():
            return
        kp = wpilib.SmartDashboard.getNumber("Shooter/Tuning/kP", ShooterConstants.SHOOTER_KP)
        ki = wpilib.SmartDashboard.getNumber("Shooter/Tuning/kI", ShooterConstants.SHOOTER_KI)
        kd = wpilib.SmartDashboard.getNumber("Shooter/Tuning/kD", ShooterConstants.SHOOTER_KD)
        kv = wpilib.SmartDashboard.getNumber("Shooter/Tuning/kV", ShooterConstants.SHOOTER_KV)
        if kp == self._cached_kp and ki == self._cached_ki and kd == self._cached_kd and kv == self._cached_kv:
            return
        self._cached_kp, self._cached_ki, self._cached_kd, self._cached_kv = kp, ki, kd, kv
        self._shooter_motor.configurator.apply(
            Slot0Configs().with_k_p(kp).with_k_i(ki).with_k_d(kd).with_k_v(kv)
        )

    def _log_telemetry(self) -> None:
        current_rpm = self.get_current_rpm()

        wpilib.SmartDashboard.putNumber("Shooter/Current RPM (Mech)", current_rpm)
        wpilib.SmartDashboard.putNumber("Shooter/Target RPM (Mech)", self._target_rpm)
        wpilib.SmartDashboard.putBoolean("Shooter/RPM At Speed", self.is_at_target_speed())
        wpilib.SmartDashboard.putNumber(
            "Shooter/Motor Current (A)", self._shooter_motor.get_supply_current().value_as_double
        )

        wpilib.SmartDashboard.putNumber("Shooter/Active Distance (ft)", self._current_target_distance)
        wpilib.SmartDashboard.putNumber("Shooter/POV Preset Distance (ft)", self._pov_preset_distance_ft)
        wpilib.SmartDashboard.putNumber("Shooter/Vision Distance (ft)", self._vision_distance_ft)
        wpilib.SmartDashboard.putNumber("Shooter/Odometry Distance (ft)", self._odometry_distance_ft)
        wpilib.SmartDashboard.putString("Shooter/Distance Source", self._distance_source)

        wpilib.SmartDashboard.putBoolean("Shooter/Shooter Enabled", self.can_spin_shooter())
