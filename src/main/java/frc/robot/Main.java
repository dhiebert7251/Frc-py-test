package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless
 * you know what you are doing, do not modify this file except to change the parameter
 * class to the startRobot call.
 *
 * <p>{@code public final class Main} with a {@code private Main() {}} constructor is
 * the standard WPILib idiom for "this class is never instantiated, only its
 * {@code main} method is ever called" -- the same {@code private} no-instances pattern
 * used throughout Constants.java, just applied to a class with actual behavior instead
 * of only constants.
 */
public final class Main {
    private Main() {}

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>{@code String... args} is Java's "varargs" syntax -- it lets this method be
     * called with any number of String arguments (including zero), collected into a
     * single {@code String[]} inside the method. {@code RobotBase.startRobot(Robot::new)}
     * is what actually builds and runs the robot: {@code Robot::new} is a method
     * reference -- shorthand for "a function that, when called, returns
     * {@code new Robot()}" -- which {@code startRobot} calls internally once it has set
     * up everything a robot program needs (the HAL, the scheduler loop) around it.
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
