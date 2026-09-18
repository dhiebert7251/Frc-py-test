"""Commands for DriveTrain.

Every command in this project is written as its own explicit class here in
commands/, rather than as a short `cmd.run(...)`-returning method living
directly on the subsystem. See the README's "Where do commands live?"
section for the full reasoning and its tradeoffs -- the short version is:
an explicit class makes the Command lifecycle (initialize -> execute (many
times) -> isFinished? -> end) visible as four separate methods instead of
implicit in how a factory function is composed, which is easier to teach to
someone who hasn't internalized that lifecycle yet.

All four commands below need DriveTrain and nothing else, so they all call
self.addRequirements(drivetrain) in __init__ -- this is what stops, for
example, TeleopDriveCommand (the default command) and DriveDistanceCommand
from both trying to drive the motors at the same time; scheduling one
automatically interrupts the other.

None of this file uses a lambda or an inline `cmd.xyz(...)` helper anywhere
-- every command, including the two one-shot ones, is a full class with
named methods. That costs a little boilerplate (see TeleopDriveCommand vs.
what it would look like with a lambda, in its own docstring below) but
means every piece of behavior has a name and a place to put a comment,
which matters more than brevity for a codebase meant to be read by
beginners.
"""
from __future__ import annotations

from commands2 import Command
from commands2.button import CommandXboxController
from wpimath.controller import PIDController

from constants import DriveTrainConstants, METERS_PER_FOOT
from subsystems.drivetrain import DriveTrain


class TeleopDriveCommand(Command):
    """The default command: tank drive read straight from the driver
    controller's two joystick Y-axes.

    Takes the controller object itself, rather than two "give me the
    current left/right stick value" callables -- a common alternative
    (`Callable[[], float]` arguments filled in with `lambda: ...` at the
    call site) that avoids naming this class but requires understanding
    closures/lambdas to read. Calling `driver_controller.getLeftY()`
    directly, right here, is one idea instead of two.

    This is the simplest possible Command with real behavior. It has no
    state and nothing to set up or clean up, so it only overrides
    execute() and isFinished() -- there's no need to write empty
    initialize()/end() methods just to have them; Command's base class
    already provides do-nothing versions. isFinished() always returns
    False because a default command is meant to run forever, until some
    other command needs DriveTrain and interrupts it.
    """

    def __init__(self, drivetrain: DriveTrain, driver_controller: CommandXboxController) -> None:
        # __init__ is a "constructor" -- Python calls this method
        # automatically whenever something builds a new TeleopDriveCommand,
        # e.g. `TeleopDriveCommand(self.drivetrain, self._driver_controller)`
        # in robotcontainer.py. If commands/gripper_commands.py's
        # IntakeCommand.__init__ is unfamiliar, read its comments first --
        # it walks through this exact same pattern with just one parameter.
        # Applied to the two parameters here:
        #
        #   * `self` is the new TeleopDriveCommand object under
        #     construction. Every method on a class takes it as the first
        #     parameter, and Python supplies it automatically -- it's never
        #     written at the call site (the call above passes two
        #     arguments, not three, even though this signature lists
        #     three parameters counting `self`). Anything saved onto
        #     `self` here is what execute() and isFinished() below can see
        #     later, since they receive that same `self` when the
        #     scheduler calls them.
        #   * `drivetrain: DriveTrain` -- a parameter named `drivetrain`,
        #     TYPE-HINTED as `DriveTrain` (the class defined in
        #     subsystems/drivetrain.py). The `: DriveTrain` part is a hint
        #     for humans and editors/IDEs, not something Python checks
        #     while the code runs -- passing, say, a Shooter here wouldn't
        #     crash on its own, but it WOULD be a bug, and the type hint is
        #     what makes that bug visible (as an editor warning) before the
        #     code is ever run.
        #   * `driver_controller: CommandXboxController` -- the physical
        #     Xbox controller plugged into port 0 (see
        #     OperatorConstants.DRIVER_CONTROLLER_PORT in constants.py,
        #     and robotcontainer.py, where the real controller object is
        #     actually constructed and passed in here). Storing the whole
        #     controller object -- instead of, say, two numbers read from
        #     it once -- is what lets execute() below ask it fresh
        #     questions (`.getLeftY()`, `.getRightY()`) every single loop.
        #   * `-> None` after the closing `)` is this METHOD's own return
        #     type hint: it says __init__ doesn't hand back a value. Python
        #     constructors are never allowed to return anything other than
        #     None anyway, so this is really just making that fact visible
        #     to a reader. Compare isFinished() below, whose `-> bool`
        #     means it DOES hand back a value (True or False).
        #   * `super().__init__()` on the next line calls Command's own
        #     constructor first, before this class's __init__ does
        #     anything else -- every Command subclass in this project
        #     starts with this exact line, since skipping it would leave
        #     the underlying commands2.Command machinery half-set-up.
        super().__init__()
        self._drivetrain = drivetrain
        self._driver_controller = driver_controller
        self.addRequirements(drivetrain)

    def execute(self) -> None:
        # execute() runs every ~20ms while this command is scheduled --
        # exactly often enough to keep reading fresh joystick values and
        # keep driving. Xbox joysticks report "pushed forward" as a
        # NEGATIVE Y value, which is backwards from how a driver thinks
        # about "forward" -- the leading minus signs below flip that back.
        left_y = -self._driver_controller.getLeftY()
        right_y = -self._driver_controller.getRightY()
        self._drivetrain.drive(
            DriveTrainConstants.SPEED_SCALE * left_y,
            DriveTrainConstants.SPEED_SCALE * right_y,
        )

    def isFinished(self) -> bool:
        # `-> bool` means this method hands back True or False. The
        # CommandScheduler checks isFinished() every loop while a command
        # is running and ends it the instant this returns True. Always
        # returning False here means "never finish on your own" -- exactly
        # what a default command needs, since it's meant to keep running
        # until some OTHER command that also needs DriveTrain gets
        # scheduled and interrupts this one instead.
        return False


class ResetGyroCommand(Command):
    """Resets the navX gyro's heading to 0 -- bound to the driver's Back
    button. Do this before every autonomous run, with the robot pointed the
    way it should be for that run's "0 degrees."

    A one-shot action still gets a full class here, on purpose (see the
    README's "Where do commands live?" section): initialize() does the
    actual work, and isFinished() returns True immediately so the command
    scheduler ends it the very next loop after that -- there's no execute()
    at all, since there's nothing to repeat.
    """

    def __init__(self, drivetrain: DriveTrain) -> None:
        # Same __init__ pattern as TeleopDriveCommand above, just with one
        # parameter instead of two -- see its comments for what `self`,
        # `drivetrain: DriveTrain`, and `-> None` each mean.
        super().__init__()
        self._drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        self._drivetrain.reset_gyro()

    def isFinished(self) -> bool:
        return True


class DriveDistanceCommand(Command):
    """Drives straight to a target distance, given in FEET, using a PID
    loop on the average of the two drive encoders.

    Why PID instead of "drive at a fixed speed until the encoder says
    you're far enough" (this command's first version): a fixed speed either
    overshoots -- the motors are still at full speed right up to the exact
    instant the target is crossed, so the robot coasts/slams past it -- or
    forces someone to guess a "stop early to leave room for coasting"
    fudge factor. A PID controller instead recalculates "how hard should I
    push" every loop from how much distance is left, so the commanded speed
    naturally tapers off as the target gets close instead of being all full
    power then all stop.
    """

    def __init__(self, drivetrain: DriveTrain, distance_feet: float) -> None:
        # `distance_feet: float` is the same `name: type` pattern as
        # `drivetrain: DriveTrain` above, just with a built-in Python type
        # (`float`, a decimal number) instead of one of our own classes --
        # the colon-and-type syntax works exactly the same way either way.
        super().__init__()
        self._drivetrain = drivetrain
        # Feet-to-meters conversion happens exactly once, right here, at the
        # boundary where a "human" feet value enters this command -- see
        # constants.METERS_PER_FOOT.
        self._target_distance_meters = distance_feet * METERS_PER_FOOT
        self.addRequirements(drivetrain)

        self._pid = PIDController(
            DriveTrainConstants.DRIVE_DISTANCE_KP,
            DriveTrainConstants.DRIVE_DISTANCE_KI,
            DriveTrainConstants.DRIVE_DISTANCE_KD,
        )
        self._pid.setTolerance(DriveTrainConstants.DRIVE_DISTANCE_TOLERANCE_METERS)

    def initialize(self) -> None:
        # initialize() runs exactly once, the instant this command is
        # scheduled (not when it's constructed in __init__, which for
        # autonomous commands happens once at RobotContainer startup,
        # possibly minutes before the command actually runs). Zeroing the
        # encoders and the PID controller here means "distance driven" is
        # always measured from wherever the robot happens to be right now.
        self._drivetrain.reset_encoders()
        self._pid.reset()
        self._pid.setSetpoint(self._target_distance_meters)

    def execute(self) -> None:
        # PIDController.calculate(measurement) returns "how hard to push"
        # based on the error between `measurement` and the setpoint given
        # in initialize(). It's clamped to +/-DRIVE_DISTANCE_MAX_OUTPUT as a
        # second, independent safety margin on top of tuning KP
        # conservatively -- a large distance error (commanding 10 feet from
        # a dead stop) should never be able to demand more than that
        # fraction of full power.
        output = self._pid.calculate(self._drivetrain.get_average_distance_meters())
        max_output = DriveTrainConstants.DRIVE_DISTANCE_MAX_OUTPUT
        output = max(-max_output, min(max_output, output))
        self._drivetrain.drive(output, output)

    def isFinished(self) -> bool:
        return self._pid.atSetpoint()

    def end(self, interrupted: bool) -> None:
        # `interrupted: bool` is a parameter the CommandScheduler fills in
        # for you -- True if this command got cut off early (the driver
        # grabbed the joystick mid-autonomous, the match ended, the robot
        # got disabled), False if isFinished() returned True on its own.
        # end() runs exactly once either way, which is exactly why
        # stopping the motors belongs here rather than only handling the
        # "finished normally" path -- this command doesn't need to tell
        # the two cases apart, but end() always receives this parameter
        # regardless of whether a command reads it.
        self._drivetrain.stop()


class TurnToAngleCommand(Command):
    """PID-turns to an absolute heading, given in degrees (CCW-positive,
    0 = whatever heading the navX gyro was last reset to). Positive =
    turn left.
    """

    def __init__(self, drivetrain: DriveTrain, target_degrees: float) -> None:
        super().__init__()
        self._drivetrain = drivetrain
        self._target_degrees = target_degrees
        self.addRequirements(drivetrain)

        self._pid = PIDController(
            DriveTrainConstants.TURN_KP, DriveTrainConstants.TURN_KI, DriveTrainConstants.TURN_KD
        )
        # A heading wraps around at +/-180 degrees. Without this next line,
        # turning from 179 degrees to -179 degrees (really just a 2-degree
        # turn) would look to a naive PID controller like a 358-degree turn
        # the "long way around." enableContinuousInput tells it to treat the
        # range as a circle instead of a straight line.
        self._pid.enableContinuousInput(-180, 180)
        self._pid.setTolerance(DriveTrainConstants.TURN_TOLERANCE_DEGREES)

    def initialize(self) -> None:
        self._pid.reset()
        self._pid.setSetpoint(self._target_degrees)

    def execute(self) -> None:
        output = self._pid.calculate(self._drivetrain.get_heading_degrees())
        # Turning in place: equal and opposite power to each side. Positive
        # output steers left, so the left side goes backward while the
        # right side goes forward.
        self._drivetrain.drive(-output, output)

    def isFinished(self) -> bool:
        return self._pid.atSetpoint()

    def end(self, interrupted: bool) -> None:
        self._drivetrain.stop()
