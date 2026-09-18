"""RobotContainer for the teaching-bot proof of concept.

Wires the six subsystems together, sets teleop default commands and
button bindings, and builds the autonomous chooser. See README.md for the
full controller-binding table and subsystem/command maps -- this file is
meant to be read start-to-finish as the map of the whole robot.
"""
from __future__ import annotations

from commands2 import Command
from commands2.button import CommandXboxController

import autonomous.chooser
from commands.drivetrain_commands import ResetGyroCommand, TeleopDriveCommand
from commands.elevator_commands import LowerElevatorCommand, RaiseElevatorCommand
from commands.gripper_commands import EjectCommand, IntakeCommand
from commands.shooter_commands import SpinUpShooterCommand
from commands.trigger_commands import FireCommand
from commands.vision_commands import ApproachTagCommand
from constants import OperatorConstants, TriggerConstants, VisionConstants
from subsystems.drivetrain import DriveTrain
from subsystems.elevator import Elevator
from subsystems.gripper import Gripper
from subsystems.shooter import Shooter
from subsystems.trigger import Trigger
from subsystems.vision import Vision


class RobotContainer:
    def __init__(self) -> None:
        # Vision has to exist before DriveTrain -- DriveTrain's pose
        # estimator reads from it every loop (see subsystems/drivetrain.py).
        self.vision = Vision()
        self.drivetrain = DriveTrain(self.vision)
        self.shooter = Shooter()
        self.trigger = Trigger()
        self.elevator = Elevator()
        self.gripper = Gripper()

        # CommandXboxController is the current, non-deprecated way to bind
        # buttons to commands for an Xbox-style controller as of RobotPy
        # 2026 (verified directly against the installed
        # robotpy-commands-v2==2026.2.2 package -- no deprecation warning on
        # construction or use). It is NOT the same thing as the generic
        # CommandGenericHID, which has no Xbox-specific button names
        # (a(), b(), leftBumper(), etc.) -- there is no "CommandGameController"
        # class in either the 2026 or the 2027-alpha bindings.
        #
        # 2027 alpha preview (verified against robotpy==2027.0.0a6 in this
        # session -- a7 has no published native wheels yet, same caveat as
        # this repo's 2027-alpha-tracking branch): the underlying
        # wpilib.XboxController class itself has already been renamed to
        # wpilib.NiDsXboxController (PS4/PS5/Stadia controllers were renamed
        # the same way; the generic wpilib.GenericHID was NOT renamed).
        # commands2 (robotpy-commands-v2) has no 2027 release at all yet, so
        # there is no CommandNiDsXboxController counterpart to switch to --
        # if RobotPy's naming pattern holds once commands2 catches up to
        # 2027, expect this line to eventually become
        # `CommandNiDsXboxController(...)`. Until then, this line stays on
        # the current 2026-stable class, since it's the only one that
        # actually exists and runs today.
        self._driver_controller = CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)
        self._operator_controller = CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT)

        self._configure_default_commands()
        self._configure_bindings()

        self._auto_chooser = autonomous.chooser.build(self.drivetrain)

    def _configure_default_commands(self) -> None:
        # A subsystem's default command runs whenever no other command
        # needs that subsystem -- here, that means "whenever the driver
        # isn't running an autonomous/other DriveTrain command, tank drive
        # from the sticks."
        self.drivetrain.setDefaultCommand(TeleopDriveCommand(self.drivetrain, self._driver_controller))

    def _configure_bindings(self) -> None:
        """Configure button-to-command bindings.

        Driver (port 0) -- drive, plus the two example vision commands:
          Left Y / Right Y = tank drive
          Back              = reset gyro heading to 0 (do this before autonomous!)
          A                 = approach the example tag, stop 3 ft away facing it
          X                 = approach the example tag, stop 5 ft away, then
                              turn 45 degrees right of facing it

        Operator (port 1) -- everything else:
          A            = toggle shooter spin-up
          B            = fire trigger
          X            = gripper intake while held
          Y            = gripper eject while held
          Right Bumper = raise elevator while held
          Left Bumper  = lower elevator while held

        Every binding below schedules a named Command class -- none of them
        build a command inline with a lambda, including the one-shot gyro
        reset (see ResetGyroCommand's docstring for why a one-shot action
        still gets a full class in this project).
        """
        self._driver_controller.back().onTrue(ResetGyroCommand(self.drivetrain))

        self._driver_controller.a().onTrue(
            ApproachTagCommand(
                self.drivetrain, self.vision, VisionConstants.EXAMPLE_TAG_ID, VisionConstants.APPROACH_STANDOFF_FEET
            )
        )
        self._driver_controller.x().onTrue(
            ApproachTagCommand(
                self.drivetrain,
                self.vision,
                VisionConstants.EXAMPLE_TAG_ID,
                VisionConstants.APPROACH_AND_TURN_STANDOFF_FEET,
                VisionConstants.APPROACH_AND_TURN_OFFSET_DEGREES,
            )
        )

        self._operator_controller.a().toggleOnTrue(SpinUpShooterCommand(self.shooter))

        # FireCommand has no timeout of its own -- `.withTimeout()` is a
        # decorator that wraps ANY command (see FireCommand's docstring),
        # applied here at the one place this command is actually bound to a
        # button, using the safety timeout defined in TriggerConstants.
        self._operator_controller.b().onTrue(FireCommand(self.trigger).withTimeout(TriggerConstants.FIRE_TIMEOUT_SECONDS))

        self._operator_controller.x().whileTrue(IntakeCommand(self.gripper))
        self._operator_controller.y().whileTrue(EjectCommand(self.gripper))
        self._operator_controller.rightBumper().whileTrue(RaiseElevatorCommand(self.elevator))
        self._operator_controller.leftBumper().whileTrue(LowerElevatorCommand(self.elevator))

    def get_autonomous_command(self) -> Command:
        return self._auto_chooser.getSelected()
