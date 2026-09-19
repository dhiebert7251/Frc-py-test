package frc.robot.subsystems;

import static frc.robot.Constants.GripperConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Gripper subsystem -- spinning roller intake at the end of the elevator.
 *
 * <p>Teaching-bot proof of concept. The simplest subsystem here: one motor, no sensors
 * at all. {@code setSpeed()}/{@code stop()} are the only two things it knows how to do
 * -- commands/IntakeCommand.java and commands/EjectCommand.java just pick which speed
 * to hold while a button is pressed.
 */
public class Gripper extends SubsystemBase {

    private final SparkMax rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);

    /**
     * No parameters besides the implicit {@code this} here -- Gripper doesn't need
     * anything handed to it from the outside to build itself; every value it needs
     * (motor CAN ID, current limit, ...) comes from {@code GripperConstants} instead.
     * Compare this to commands/IntakeCommand.java's constructor, which DOES take a
     * parameter ({@code Gripper gripper}) because a command needs to be told WHICH
     * Gripper object to act on.
     */
    public Gripper() {
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.inverted(ROLLER_MOTOR_INVERTED);
        rollerConfig.idleMode(IdleMode.kBrake);
        rollerConfig.smartCurrentLimit(ROLLER_CURRENT_LIMIT);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * {@code speed} is a duty cycle in [-1, 1]: positive intakes, negative ejects -- see
     * {@code GripperConstants.INTAKE_SPEED}/{@code EJECT_SPEED}. The parameter is
     * written {@code double speed} rather than {@code speed: float} the way Python
     * wrote it -- Java always puts the type BEFORE the name, with no colon, for every
     * parameter and every field in this project; Python puts the type AFTER the name,
     * with a colon, and only when someone chooses to add the (optional) hint.
     */
    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void stop() {
        rollerMotor.set(0.0);
    }
}
