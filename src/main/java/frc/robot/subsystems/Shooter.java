package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter subsystem -- single flywheel (Kraken/TalonFX), fixed target RPM.
 *
 * <p>Teaching-bot proof of concept. Simpler than the competition bot's Shooter: no
 * distance-based RPM table (no vision on this robot) -- just a single configurable
 * target speed. Demonstrates Phoenix 6's velocity-control pattern.
 *
 * <p>Unlike REVLib and Studica's Java bindings, whose method names read almost
 * identically to their Python counterparts (just camelCase instead of snake_case),
 * Phoenix 6's Java enum constant names use their own capitalization
 * ({@code InvertedValue.Clockwise_Positive}) that differs even in capitalization
 * convention from the same enum in Python ({@code InvertedValue.CLOCKWISE_POSITIVE})
 * -- CTRE's Java and Python bindings were written somewhat independently. Worth noting
 * explicitly the first time a rookie moving between this Java project and its Python
 * sibling hits it.
 *
 * <p>Like the other subsystems, this file only exposes plain hardware actions
 * ({@code setTargetRpm()}, {@code stop()}, the getters) -- the actual Command that uses
 * them lives in commands/SpinUpShooterCommand.java.
 */
public class Shooter extends SubsystemBase {

    // VelocityVoltage and NeutralOut are "control request" objects: instead of calling
    // a method with new arguments every loop (like SparkMax's .set()), Phoenix 6 wants
    // you to build one request object per control mode and re-send it (via
    // setControl(), below) whenever you want to change or refresh what the motor is
    // doing. withSlot(0) picks which of the TalonFX's internal PID gain slots
    // (configured below as slot 0) this velocity request should use.
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private double targetRpm = 0.0;

    private final TalonFX flywheelMotor = new TalonFX(FLYWHEEL_MOTOR_ID);

    public Shooter() {
        // Phoenix 6 configuration is one big object built up with chained
        // `.with*()` calls (each one returns the same object back, which is what lets
        // them chain), then applied in one shot via getConfigurator().apply() below --
        // REVLib's SparkMaxConfig from DriveTrain/Trigger/Elevator/Gripper is the same
        // "build a config object, then apply it" idea, just with REV's own
        // method-naming style instead of CTRE's.
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(FLYWHEEL_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true))
            .withSlot0(
                // Slot0Configs holds the PID(+velocity feedforward) gains the TalonFX
                // itself uses to run its OWN closed velocity loop, in hardware, every
                // control cycle -- much faster than this Java code's ~20ms loop could.
                // This is different from DriveTrain's PID commands, which run the PID
                // math in Java and only send a duty cycle to the motor.
                new Slot0Configs()
                    .withKP(SHOOTER_KP)
                    .withKI(SHOOTER_KI)
                    .withKD(SHOOTER_KD)
                    .withKV(SHOOTER_KV));

        StatusCode configError = flywheelMotor.getConfigurator().apply(flywheelConfig);
        if (!configError.isOK()) {
            DriverStation.reportWarning("Shooter flywheel motor config failed: " + configError, false);
        }
    }

    /**
     * Commands the flywheel to spin at {@code rpm}. Because this is a closed-loop
     * velocity request handled on the TalonFX itself (see the Slot0Configs comment
     * above), this only needs to be called once when the target changes -- not every
     * loop like an open-loop duty cycle motor would need.
     */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        flywheelMotor.setControl(velocityRequest.withVelocity(rpm / FLYWHEEL_GEAR_RATIO / 60.0));
    }

    public void stop() {
        targetRpm = 0.0;
        flywheelMotor.setControl(neutralRequest);
    }

    public double getCurrentRpm() {
        return flywheelMotor.getVelocity().getValueAsDouble() * 60.0 * FLYWHEEL_GEAR_RATIO;
    }

    public boolean isAtTargetSpeed() {
        return targetRpm > 0 && Math.abs(getCurrentRpm() - targetRpm) <= RPM_TOLERANCE;
    }

    @Override
    public void periodic() {
        // RPM has no separate "imperial" form the way a distance does, so unlike
        // DriveTrain's telemetry there's no unit conversion to do here.
        SmartDashboard.putNumber("Shooter/CurrentRPM", getCurrentRpm());
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRpm);
        SmartDashboard.putBoolean("Shooter/AtSpeed", isAtTargetSpeed());
    }
}
