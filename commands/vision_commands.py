"""Commands that use Vision to navigate relative to an AprilTag.

This is this project's one genuinely cross-subsystem command: it needs
both DriveTrain (to actually move) and Vision (to know where a tag is).
Earlier branches of this project explicitly said no command needed more
than one subsystem -- that was true until this one. Decisions like that
are provisional, not permanent: once the situation changes (adding
vision), the right response is to update the decision, not defend it. See
the README's "Where do commands live?" section for the fuller version of
this note.

ApproachTagCommand only calls `self.addRequirements(drivetrain)`, not
Vision -- it only ever READS from Vision (asking "where is this tag?"),
never commands it, and `addRequirements()` exists to prevent two commands
from fighting over something they both DRIVE, not something they both read.
"""
from __future__ import annotations

import math

from commands2 import Command
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d, Translation2d

from constants import DriveTrainConstants, METERS_PER_FOOT
from subsystems.drivetrain import DriveTrain
from subsystems.vision import Vision


class ApproachTagCommand(Command):
    """Drives to a point a fixed distance in front of an AprilTag, then
    turns to face it -- optionally rotated some number of degrees off of
    "directly facing it."

    Three phases, run one after another inside this single command (the
    same kind of internal state machine as `commands/trigger_commands.py`'s
    `FireCommand`, just with three states instead of two):

      1. TURN_TO_TARGET -- turn in place to face the point we're driving to.
      2. DRIVE_TO_TARGET -- drive straight to that point.
      3. FACE_TAG -- turn in place to the final heading.

    Nothing about *where* to go is known until this command actually
    starts: `initialize()` reads the robot's current estimated pose (from
    `DriveTrain.get_pose()`) and the tag's known field position (from
    `Vision.get_tag_pose()`) to compute the target point and final heading,
    the same loop this command is scheduled -- unlike `DriveDistanceCommand`/
    `TurnToAngleCommand`, whose targets are fixed numbers known when they're
    constructed.

    Why one parameterized class instead of two nearly-identical ones (the
    way `RaiseElevatorCommand`/`LowerElevatorCommand` are two separate
    classes for what's conceptually "the same page, in the other
    direction"): those two commands' bodies are almost entirely different
    numbers, three lines each. This command's three phases -- computing a
    target point, turning to a bearing, driving a distance, turning to a
    final heading -- are the same ~40 lines of state-machine logic for both
    of this project's example uses; the only thing that differs between
    "stop 3 feet away, facing the tag" and "stop 5 feet away, then turn 45
    degrees off of facing it" is one number, `face_offset_degrees`.
    Duplicating that logic to keep two separate classes would risk the two
    copies drifting out of sync the next time one gets a bugfix -- see
    robotcontainer.py for the two named instances this project actually
    binds.
    """

    def __init__(
        self,
        drivetrain: DriveTrain,
        vision: Vision,
        tag_id: int,
        standoff_feet: float,
        face_offset_degrees: float = 0.0,
    ) -> None:
        super().__init__()
        self._drivetrain = drivetrain
        self._vision = vision
        self._tag_id = tag_id
        self._standoff_meters = standoff_feet * METERS_PER_FOOT
        # Positive face_offset_degrees means "turn right (clockwise) from
        # facing the tag directly" -- the opposite sign from this
        # codebase's CCW-positive convention (see
        # DriveTrainConstants.TURN_TOLERANCE_DEGREES's neighboring
        # comments), so it's negated once, right here, at the boundary
        # where a human-facing "turn right" number enters this command.
        self._face_offset_degrees = -face_offset_degrees
        self.addRequirements(drivetrain)

        # Reuses DriveTrain's own turn/drive PID gains rather than
        # introducing a second, separately-tuned set -- this command does
        # the exact same two kinds of motion (turn in place, drive
        # straight) that TurnToAngleCommand/DriveDistanceCommand already
        # tune gains for.
        self._turn_pid = PIDController(
            DriveTrainConstants.TURN_KP, DriveTrainConstants.TURN_KI, DriveTrainConstants.TURN_KD
        )
        self._turn_pid.enableContinuousInput(-180, 180)
        self._turn_pid.setTolerance(DriveTrainConstants.TURN_TOLERANCE_DEGREES)

        self._drive_pid = PIDController(
            DriveTrainConstants.DRIVE_DISTANCE_KP,
            DriveTrainConstants.DRIVE_DISTANCE_KI,
            DriveTrainConstants.DRIVE_DISTANCE_KD,
        )
        self._drive_pid.setTolerance(DriveTrainConstants.DRIVE_DISTANCE_TOLERANCE_METERS)

        self._phase = "DONE"
        self._target_point = Translation2d()
        self._final_heading_degrees = 0.0
        self._drive_start_distance_meters = 0.0

    def initialize(self) -> None:
        tag_pose = self._vision.get_tag_pose(self._tag_id)
        if tag_pose is None:
            # An unknown tag ID -- nothing to approach. Ending immediately
            # (isFinished() checks for this phase) is safer than guessing;
            # see the README's "Assumptions that need bench verification"
            # for why this is worth a dashboard warning in a real robot,
            # not just a silently-do-nothing command.
            self._phase = "FAILED"
            return

        tag_pose_2d = tag_pose.toPose2d()
        # A tag's pose "faces" outward, away from the tag surface -- the
        # standoff point is that far along the tag's own facing direction,
        # and facing the tag from there means pointing the opposite way
        # (180 degrees from how the tag itself faces).
        tag_facing = tag_pose_2d.rotation()
        self._target_point = tag_pose_2d.translation() + Translation2d(self._standoff_meters, 0.0).rotateBy(
            tag_facing
        )
        self._final_heading_degrees = (tag_facing + Rotation2d.fromDegrees(180.0 + self._face_offset_degrees)).degrees()

        self._begin_turn_to_target()

    def _begin_turn_to_target(self) -> None:
        current_translation = self._drivetrain.get_pose().translation()
        delta = self._target_point - current_translation
        bearing_degrees = math.degrees(math.atan2(delta.Y(), delta.X()))

        self._turn_pid.reset()
        self._turn_pid.setSetpoint(bearing_degrees)
        self._phase = "TURN_TO_TARGET"

    def _begin_drive_to_target(self) -> None:
        # Distance-to-go is recomputed here, once, from the pose the robot
        # actually ended the turn at -- not assumed from the turn's own
        # target, since a real turn won't land exactly on its setpoint.
        distance_meters = self._drivetrain.get_pose().translation().distance(self._target_point)
        self._drive_start_distance_meters = self._drivetrain.get_average_distance_meters()

        self._drive_pid.reset()
        self._drive_pid.setSetpoint(distance_meters)
        self._phase = "DRIVE_TO_TARGET"

    def _begin_face_tag(self) -> None:
        self._turn_pid.reset()
        self._turn_pid.setSetpoint(self._final_heading_degrees)
        self._phase = "FACE_TAG"

    def execute(self) -> None:
        if self._phase == "TURN_TO_TARGET":
            output = self._turn_pid.calculate(self._drivetrain.get_heading_degrees())
            self._drivetrain.drive(-output, output)
            if self._turn_pid.atSetpoint():
                self._begin_drive_to_target()

        elif self._phase == "DRIVE_TO_TARGET":
            # Same relative-distance measurement DriveDistanceCommand uses
            # -- see its initialize() docstring for why this can't just
            # reset the encoders to zero instead.
            distance_this_phase = self._drivetrain.get_average_distance_meters() - self._drive_start_distance_meters
            output = self._drive_pid.calculate(distance_this_phase)
            max_output = DriveTrainConstants.DRIVE_DISTANCE_MAX_OUTPUT
            output = max(-max_output, min(max_output, output))
            self._drivetrain.drive(output, output)
            if self._drive_pid.atSetpoint():
                self._begin_face_tag()

        elif self._phase == "FACE_TAG":
            output = self._turn_pid.calculate(self._drivetrain.get_heading_degrees())
            self._drivetrain.drive(-output, output)

        # "FAILED"/"DONE": nothing to do -- isFinished() ends the command.

    def isFinished(self) -> bool:
        if self._phase in ("FAILED", "DONE"):
            return True
        return self._phase == "FACE_TAG" and self._turn_pid.atSetpoint()

    def end(self, interrupted: bool) -> None:
        self._drivetrain.stop()
