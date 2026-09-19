package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

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
 * Elevator subsystem -- 2-stage single-mast elevator (AndyMark "Elevator in a Box"
 * style cascade rig), spring-assisted extension, motor+rope retraction.
 *
 * <p>Teaching-bot proof of concept. The Redline motor here is brushed and has no
 * encoder (a real one could add a through-bore/versa encoder later for closed-loop
 * positioning -- see README) -- this subsystem is entirely open-loop, driven only by a
 * limit switch at each end of travel. Raising needs less motor power because the
 * springs are doing most of the work; lowering needs the motor to actively pull the
 * rope in against that same spring tension.
 *
 * <p>This subsystem only exposes plain actions (raise/lower a notch, stop, check the
 * limit switches) -- the "keep raising/lowering while a button is held, but always stop
 * at a limit switch even if the button is still held" behavior lives in
 * commands/RaiseElevatorCommand.java and commands/LowerElevatorCommand.java.
 */
public class Elevator extends SubsystemBase {

    // kBrushed, not kBrushless: a Redline motor has physical brushes (hence the name)
    // and no built-in encoder, unlike every NEO in this project. SparkMax can drive
    // either motor type, but has to be told which one it's talking to, since brushed
    // and brushless motors are commutated (have their windings energized in sequence)
    // completely differently in hardware.
    private final SparkMax liftMotor = new SparkMax(LIFT_MOTOR_ID, MotorType.kBrushed);

    private final DigitalInput topLimitSwitch = new DigitalInput(TOP_LIMIT_SWITCH_DIO_PORT);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_SWITCH_DIO_PORT);

    public Elevator() {
        SparkMaxConfig liftConfig = new SparkMaxConfig();
        liftConfig.inverted(LIFT_MOTOR_INVERTED);
        liftConfig.idleMode(IdleMode.kBrake);
        liftConfig.smartCurrentLimit(LIFT_CURRENT_LIMIT);
        liftMotor.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean isAtTop() {
        boolean raw = topLimitSwitch.get();
        return TOP_LIMIT_SWITCH_INVERTED ? !raw : raw;
    }

    public boolean isAtBottom() {
        boolean raw = bottomLimitSwitch.get();
        return BOTTOM_LIMIT_SWITCH_INVERTED ? !raw : raw;
    }

    /** speed is a duty cycle in [-1, 1]: positive raises, negative lowers -- see
     * {@code ElevatorConstants.RAISE_SPEED}/{@code LOWER_SPEED}. */
    public void setSpeed(double speed) {
        liftMotor.set(speed);
    }

    public void stop() {
        liftMotor.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator/AtTop", isAtTop());
        SmartDashboard.putBoolean("Elevator/AtBottom", isAtBottom());
    }
}
