package frc.robot.subsystems;

import static frc.robot.Constants.TriggerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Trigger subsystem -- small NEO-driven cam that flicks a game piece into the shooter.
 *
 * <p>Teaching-bot proof of concept. The cam has exactly one sensor: a limit switch that
 * defines its rest ("home") position. This subsystem only exposes plain actions/queries
 * (run the cam motor, read the switch/beam breaks) -- the "run until it's fired one
 * full revolution" logic is real state-machine behavior, so it lives in its own Command
 * class, commands/FireCommand.java, rather than here.
 */
public class Trigger extends SubsystemBase {

    private final SparkMax camMotor = new SparkMax(CAM_MOTOR_ID, MotorType.kBrushless);

    // DigitalInput reads a single digital (on/off) signal from a roboRIO DIO port --
    // the same class WPILib uses for any simple switch or break-beam sensor, since
    // electrically they're the same thing (a circuit that's either open or closed).
    private final DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_DIO_PORT);
    private final DigitalInput beamBreak1 = new DigitalInput(BEAM_BREAK_1_DIO_PORT);
    private final DigitalInput beamBreak2 = new DigitalInput(BEAM_BREAK_2_DIO_PORT);

    public Trigger() {
        SparkMaxConfig camConfig = new SparkMaxConfig();
        camConfig.inverted(CAM_MOTOR_INVERTED);
        // Brake mode (not DriveTrain's Coast): when the cam motor is commanded to 0, we
        // want it to stop and hold position immediately, not coast -- an idle cam
        // swinging freely could drift off "home" and throw off the next fire cycle's
        // home-switch reading.
        camConfig.idleMode(IdleMode.kBrake);
        camConfig.smartCurrentLimit(CAM_CURRENT_LIMIT);
        camMotor.configure(camConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Every switch/beam-break getter here follows the same shape: read the raw
     * electrical signal, then flip it if that particular sensor's wiring reports
     * {@code true} for the opposite of what we mean (see the {@code *_INVERTED}
     * constants and their TODOs -- this is exactly the kind of thing that must be
     * checked on the real robot, since guessing wrong here silently inverts the
     * sensor's meaning). {@code cond ? a : b} is Java's ternary operator -- "if cond is
     * true, this whole expression's value is a, otherwise it's b" -- the closest Java
     * equivalent to Python's {@code a if cond else b}.
     */
    public boolean isAtHome() {
        boolean raw = limitSwitch.get();
        return LIMIT_SWITCH_INVERTED ? !raw : raw;
    }

    public boolean hasBallAtStage1() {
        boolean raw = beamBreak1.get();
        return BEAM_BREAK_1_INVERTED ? !raw : raw;
    }

    public boolean hasBallAtStage2() {
        boolean raw = beamBreak2.get();
        return BEAM_BREAK_2_INVERTED ? !raw : raw;
    }

    public void runCam() {
        camMotor.set(CAM_FIRE_SPEED);
    }

    public void stopCam() {
        camMotor.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Trigger/AtHome", isAtHome());
        SmartDashboard.putBoolean("Trigger/BallStage1", hasBallAtStage1());
        SmartDashboard.putBoolean("Trigger/BallStage2", hasBallAtStage2());
    }
}
