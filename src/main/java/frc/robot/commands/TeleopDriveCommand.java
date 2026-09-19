package frc.robot.commands;

import static frc.robot.Constants.DriveTrainConstants.SPEED_SCALE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

/**
 * The default command: tank drive read straight from the driver controller's two
 * joystick Y-axes.
 *
 * <p>Takes the controller object itself, rather than two "give me the current
 * left/right stick value" suppliers -- a common alternative
 * ({@code DoubleSupplier} arguments filled in with a lambda like
 * {@code () -> -controller.getLeftY()} at the call site) that avoids naming this class
 * but requires understanding lambdas/method references to read. Calling
 * {@code driverController.getLeftY()} directly, right here, is one idea instead of two.
 *
 * <p>This is the simplest possible Command with real behavior. It has no state and
 * nothing to set up or clean up, so it only overrides {@code execute()} and
 * {@code isFinished()} -- there's no need to write empty {@code initialize()}/
 * {@code end()} methods just to have them; {@code Command}'s base class already
 * provides do-nothing versions. {@code isFinished()} always returns {@code false}
 * because a default command is meant to run forever, until some other command needs
 * DriveTrain and interrupts it.
 */
public class TeleopDriveCommand extends Command {

    // `private final` fields, one per constructor parameter, assigned once in the
    // constructor and never reassigned afterward -- see the constructor below for the
    // full explanation of why these exist and what each piece of the constructor's
    // signature means.
    private final DriveTrain drivetrain;
    private final CommandXboxController driverController;

    /**
     * The constructor. Java calls this automatically whenever something writes
     * {@code new TeleopDriveCommand(...)} -- in this project, that happens exactly
     * once, in RobotContainer.java's {@code configureDefaultCommands()}. Several
     * pieces of syntax on this line are worth calling out individually, since they
     * repeat, in different combinations, in every command class in this project:
     *
     * <ul>
     *   <li><b>{@code public}</b> -- an access modifier. It means any other class,
     *       anywhere in this project (or beyond), can call {@code new
     *       TeleopDriveCommand(...)}. Compare this to {@code private} fields like
     *       {@code drivetrain} above: those can only be read by code written inside
     *       this very class. Java requires an explicit access modifier decision like
     *       this on every field and method; Python has no equivalent keyword; it only
     *       has the {@code _leadingUnderscore} naming CONVENTION this whole project
     *       already uses to mean the same thing, which nothing in the language itself
     *       enforces.</li>
     *   <li><b>{@code DriveTrain drivetrain}</b> -- a parameter. Unlike Python, where
     *       a type hint after a colon ({@code drivetrain: DriveTrain}) is optional
     *       and checked only by external tools (never by the Python interpreter
     *       itself), Java requires every parameter to have a declared type, written
     *       BEFORE the name with no colon, and the compiler itself refuses to compile
     *       code that passes the wrong type here -- there is no way to skip this in
     *       Java the way an un-annotated Python parameter skips it.</li>
     *   <li><b>{@code CommandXboxController driverController}</b> -- the physical Xbox
     *       controller plugged into port 0 (see {@code Constants.OperatorConstants
     *       .DRIVER_CONTROLLER_PORT}, and RobotContainer.java, where the real
     *       controller object is actually constructed and passed in here). Storing the
     *       whole controller object -- instead of, say, two numbers read from it once
     *       -- is what lets {@code execute()} below call {@code .getLeftY()}/
     *       {@code .getRightY()} on it fresh every single loop.</li>
     *   <li><b>No return type written before the constructor's name at all</b> -- not
     *       even {@code void}. Every normal Java method needs a return type
     *       ({@code void} for "returns nothing," or a real type for "returns this").
     *       A constructor is the one exception: it implicitly builds and returns the
     *       new object, and Java's grammar does not allow ANY return-type keyword to
     *       be written on this line, {@code void} included -- writing one is a syntax
     *       error, not just bad style. Python's {@code __init__(self, ...) -> None} is
     *       different in exactly this respect: Python DOES allow (and this project's
     *       Python sibling uses) an explicit {@code -> None} annotation on
     *       {@code __init__}, because in Python {@code __init__} is an ordinary method
     *       that happens to conventionally return {@code None} -- Java's constructor
     *       is a distinct kind of member with its own grammar rule forbidding a return
     *       type outright.</li>
     *   <li><b>{@code super(); }-- wait, there is no {@code super()} call written
     *       here.</b> Every command's constructor in this project's Python sibling
     *       starts with an explicit {@code super().__init__()}. In Java, if a
     *       constructor's first line does NOT explicitly call {@code super(...)}, the
     *       compiler automatically inserts a call to the parent class's no-argument
     *       constructor for you, as if it were the first line. {@code Command}'s own
     *       no-argument constructor does the setup this class needs, so nothing
     *       explicit is required here -- unlike Python, where {@code __init__} is
     *       never called automatically and always has to be invoked by name.</li>
     * </ul>
     */
    public TeleopDriveCommand(DriveTrain drivetrain, CommandXboxController driverController) {
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        // `this.drivetrain = drivetrain;` -- the field and the parameter share the
        // same name on purpose (this project's Java convention, mirroring the Python
        // sibling's `self._drivetrain = drivetrain`), which means `this.` in front of
        // the left-hand side is not optional decoration here: without it, `drivetrain
        // = drivetrain;` would just assign the parameter to itself and leave the
        // field permanently unset. `this.` explicitly means "the field belonging to
        // the object being constructed," disambiguating it from the same-named
        // parameter.
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // execute() runs every ~20ms while this command is scheduled -- exactly often
        // enough to keep reading fresh joystick values and keep driving. Xbox
        // joysticks report "pushed forward" as a NEGATIVE Y value, which is backwards
        // from how a driver thinks about "forward" -- the leading minus signs below
        // flip that back.
        double leftY = -driverController.getLeftY();
        double rightY = -driverController.getRightY();
        drivetrain.drive(SPEED_SCALE * leftY, SPEED_SCALE * rightY);
    }

    @Override
    public boolean isFinished() {
        // `boolean` (lowercase) is one of Java's eight built-in "primitive" types --
        // unlike `DriveTrain` or `CommandXboxController` above, it is not a class, has
        // no methods of its own, and can only ever hold `true` or `false`, never
        // `null`. Returning `false` here means "never finish on your own" -- exactly
        // what a default command needs, since it's meant to keep running until some
        // OTHER command that also needs DriveTrain gets scheduled and interrupts this
        // one instead.
        return false;
    }
}
