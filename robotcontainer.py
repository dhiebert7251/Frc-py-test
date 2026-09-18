"""RobotContainer for the teaching-bot proof of concept.

Wires the four subsystems together, sets teleop default commands and button
bindings, and builds the autonomous chooser. See README.md for the full
controller-binding table and subsystem/command maps -- this file is meant to
be read start-to-finish as the map of the whole robot.
"""
from __future__ import annotations

from commands2 import Command, cmd
from commands2.button import CommandXboxController

import autonomous.chooser
import autonomous.routines
from constants import OperatorConstants
from subsystems.drivetrain import DriveTrain
from subsystems.elevator import Elevator
from subsystems.gripper import Gripper
from subsystems.shooter import Shooter
from subsystems.trigger import Trigger


class RobotContainer:
    def __init__(self) -> None:
        self.drivetrain = DriveTrain()
        self.shooter = Shooter()
        self.trigger = Trigger()
        self.elevator = Elevator()
        self.gripper = Gripper()

        self._driver_controller = CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)
        self._operator_controller = CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT)

        self._configure_default_commands()
        self._configure_bindings()

        self._auto_chooser = autonomous.chooser.build(self.drivetrain)

    def _configure_default_commands(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.teleop_drive_command(
                lambda: -self._driver_controller.getLeftY(),
                lambda: -self._driver_controller.getRightY(),
            )
        )

    def _configure_bindings(self) -> None:
        """Configure button-to-command bindings.

        Driver (port 0) -- drive only:
          Left Y / Right Y = tank drive
          Back              = reset gyro heading to 0 (do this before autonomous!)

        Operator (port 1) -- everything else:
          A            = toggle shooter spin-up
          B            = fire trigger
          X            = gripper intake while held
          Y            = gripper eject while held
          Right Bumper = raise elevator while held
          Left Bumper  = lower elevator while held
        """
        self._driver_controller.back().onTrue(cmd.runOnce(self.drivetrain.reset_gyro, self.drivetrain))

        self._operator_controller.a().toggleOnTrue(self.shooter.spin_up_command())
        self._operator_controller.b().onTrue(self.trigger.fire_command())
        self._operator_controller.x().whileTrue(self.gripper.intake_command())
        self._operator_controller.y().whileTrue(self.gripper.eject_command())
        self._operator_controller.rightBumper().whileTrue(self.elevator.raise_command())
        self._operator_controller.leftBumper().whileTrue(self.elevator.lower_command())

    def get_autonomous_command(self) -> Command:
        return self._auto_chooser.getSelected()
