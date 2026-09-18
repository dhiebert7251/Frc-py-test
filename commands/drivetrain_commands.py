"""Commands for DriveTrain.

Every command in this project is written as its own explicit class here in
commands/, rather than as a short `cmd.run(...)`-returning method living
directly on the subsystem. See the README's "Where do commands live?"
section for the full reasoning and its tradeoffs -- the short version is:
an explicit class makes the Command lifecycle (initialize -> execute (many
times) -> isFinished? -> end) visible as four separate methods instead of
implicit in how a factory function is composed, which is easier to teach to
someone who hasn't internalized that lifecycle yet.

All three commands below need DriveTrain and nothing else, so they all call
self.addRequirements(drivetrain) in __init__ -- this is what stops, for
example, TeleopDriveCommand (the default command) and DriveDistanceCommand
from both trying to drive the motors at the same time; scheduling one
automatically interrupts the other.
"""
from __future__ import annotations

from typing import Callable

from commands2 import Command
from wpimath.controller import PIDController

from constants import DriveTrainConstants, METERS_PER_FOOT
from subsystems.drivetrain import DriveTrain


class TeleopDriveCommand(Command):
    """The default command: tank drive from two joystick-Y suppliers.

    This is the simplest possible Command. It has no state and nothing to
    set up or clean up, so it only overrides execute() and isFinished() --
    there's no need to write empty initialize()/end() methods just to have
    them; Command's base class already provides do-nothing versions.
    isFinished() always returns False because a default command is meant to
    run forever, until some other command needs DriveTrain and interrupts it.
    """

    def __init__(self, drivetrain: DriveTrain, left_y: Callable[[], float], right_y: Callable[[], float]) -> None:
        super().__init__()
        self._drivetrain = drivetrain
        self._left_y = left_y
        self._right_y = right_y
        self.addRequirements(drivetrain)

    def execute(self) -> None:
        # execute() runs every ~20ms while this command is scheduled --
        # exactly often enough to keep reading fresh joystick values and
        # keep driving.
        self._drivetrain.drive(
            DriveTrainConstants.SPEED_SCALE * self._left_y(),
            DriveTrainConstants.SPEED_SCALE * self._right_y(),
        )

    def isFinished(self) -> bool:
        return False


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
        # end() runs exactly once, whether isFinished() returned True OR
        # this command got interrupted (a driver grabbing the joystick
        # mid-autonomous, the match ending, disabling the robot). It's the
        # only place guaranteed to run in both cases, which is exactly why
        # stopping the motors belongs here rather than only handling the
        # "finished normally" path.
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
