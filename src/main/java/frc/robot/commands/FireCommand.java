package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trigger;

/**
 * FireCommand is the one genuinely tricky piece of logic in this whole codebase, and
 * it's a good example of why "just read the one sensor" isn't always enough: the cam
 * has exactly one sensor, a limit switch at "home," and reading it once at the start
 * would immediately (and wrongly) report "done," since the cam starts each fire cycle
 * already at home. A full fire cycle actually means: leave home, THEN come back to
 * home. {@code isFinished()} below tracks that as one bit of state,
 * {@code hasLeftHome}, which is reset every time this command starts over in
 * {@code initialize()}.
 *
 * <p>This class does NOT set its own timeout. A jammed cam or a broken switch wire
 * means it would never see "returned home" and would run forever on its own; the
 * safety timeout is applied as a {@code .withTimeout()} decorator at the one place
 * this command is actually bound to a button, in RobotContainer.java. Decorators like
 * {@code .withTimeout()}/{@code .andThen()}/{@code .until()} work on ANY Command -- an
 * explicit class like this one just as well as a {@code Commands.run(...)} one-liner --
 * which is why it doesn't matter that FireCommand and, say, DriveDistanceCommand build
 * their behavior in very different ways internally.
 */
public class FireCommand extends Command {

    private final Trigger trigger;
    // Not `final`: unlike every field seen so far, this one IS reassigned after
    // construction -- once in initialize() every time the command restarts, and again
    // inside isFinished() as the cam leaves home. It's a plain boolean instance field,
    // not a parameter -- nothing external ever passes this in; it's a value this
    // object tracks purely for itself.
    private boolean hasLeftHome = false;

    public FireCommand(Trigger trigger) {
        this.trigger = trigger;
        addRequirements(trigger);
    }

    @Override
    public void initialize() {
        hasLeftHome = false;
    }

    @Override
    public void execute() {
        trigger.runCam();
    }

    @Override
    public boolean isFinished() {
        if (!hasLeftHome) {
            // Still waiting for the cam to leave home for the first time -- once it
            // does, remember that and start watching for it to come back.
            if (!trigger.isAtHome()) {
                hasLeftHome = true;
            }
            return false;
        }
        return trigger.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        trigger.stopCam();
    }
}
