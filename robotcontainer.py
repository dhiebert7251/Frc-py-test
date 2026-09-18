"""This class is where the bulk of the robot should be declared. Since Command-based is
a "declarative" paradigm, very little robot logic should actually be handled in the
Robot periodic methods. Instead, the structure of the robot (including subsystems,
commands, and trigger mappings) should be declared here.

Ported from RobotContainer.java.
"""

from __future__ import annotations

from typing import Optional

import wpilib
from cscore import CameraServer, UsbCamera
from commands2 import Command, cmd
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder, NamedCommands

from constants import OperatorConstants, VisionConstants
from subsystems.climber import Climber
from subsystems.drivetrain import DriveTrain
from subsystems.feeder import Feeder
from subsystems.shooter import Shooter
from subsystems.vision import Vision


class RobotContainer:
    def __init__(self) -> None:
        # The robot's subsystems and commands are defined here...

        # Climber subsystem
        self.climber = Climber()

        # Vision must be constructed before DriveTrain (passed into its constructor).
        # DriveTrain must be constructed before Shooter (both are injected into Shooter).
        self.vision = Vision()
        self.drivetrain = DriveTrain(self.vision)

        # Shooter subsystem (flywheel wheel only -- distance resolution and zone enforcement).
        # Feeder subsystem (intake roller + trigger/hopper -- ball path).
        # Separated so that intake/eject can run concurrently with the shooter wheel spinning.
        self.shooter = Shooter(self.vision, self.drivetrain)
        self.feeder = Feeder()

        # Driver camera (USB webcam) -- may be None if no camera is present at startup.
        self._driver_camera: Optional[UsbCamera] = None

        # Driver controller -- drive motions only (port 0)
        self._driver_controller = CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)

        # Operator controller -- all intake and shooting operations (port 1)
        self._operator_controller = CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT)

        # Autonomous chooser
        self._do_nothing_auto = cmd.waitUntil(wpilib.DriverStation.isTeleopEnabled).withName("Do Nothing")

        # Initialize driver camera. Guard against missing hardware (simulation,
        # camera unplugged) so a missing USB camera does not crash startup.
        try:
            self._driver_camera = CameraServer.startAutomaticCapture(VisionConstants.DRIVER_CAMERA_NAME, 0)
            self._driver_camera.setResolution(320, 240)
            self._driver_camera.setFPS(30)
        except Exception as e:  # noqa: BLE001 -- mirrors the Java catch-all around camera startup
            wpilib.reportWarning(f"Driver camera not found on USB port 0 -- running without driver feed: {e}")

        self._configure_default_commands()

        # Build auto chooser -- must run after DriveTrain's constructor calls AutoBuilder.configure()
        self._auto_chooser = AutoBuilder.buildAutoChooser()
        self._auto_chooser.setDefaultOption("Do Nothing", self._do_nothing_auto)
        wpilib.SmartDashboard.putData("Auto Chooser", self._auto_chooser)

        self._configure_bindings()

    def _configure_default_commands(self) -> None:
        """Set up default commands for subsystems. Each subsystem can have one default
        command that runs whenever no other command is using that subsystem."""
        # Robot-relative tank drive. Right trigger analog-boosts speed from 70% to 100%.
        self.drivetrain.setDefaultCommand(
            self.drivetrain.teleop_drive_command(
                lambda: -self._driver_controller.getLeftY(),
                lambda: -self._driver_controller.getRightY(),
                self._driver_controller.getRightTriggerAxis,
            )
        )

        # ---- Named Commands for PathPlanner Autos ----
        # These must be registered before any auto is run. PathPlanner uses a static map
        # looked up at auto runtime, so as long as these are registered before
        # getAutonomousCommand() is called (auto init), placement here is safe even
        # though AutoBuilder.configure() already ran.

        # Shoot sequence:
        #   Phase 1 -- spin shooter up to distance-resolved RPM, wait for speed (max 2s)
        #   Phase 2 -- run shooter + feeder together for 8 seconds
        #   Cleanup -- stop everything
        def _stop_shoot_sequence(interrupted: bool) -> None:
            self.shooter.stop_shooter()
            self.feeder.stop_all()

        NamedCommands.registerCommand(
            "Shoot5Sec",
            cmd.sequence(
                cmd.run(self.shooter.resolve_distance_and_spin, self.shooter)
                .until(self.shooter.is_at_target_speed)
                .withTimeout(2.0),
                cmd.run(
                    lambda: (self.shooter.resolve_distance_and_spin(), self.feeder.start_feed()),
                    self.shooter,
                    self.feeder,
                ).withTimeout(8.0),
            ).finallyDo(_stop_shoot_sequence),
        )

        # Shoot for 8 seconds regardless of speed
        NamedCommands.registerCommand(
            "Shoot",
            cmd.sequence(
                cmd.run(
                    lambda: (self.shooter.resolve_distance_and_spin(), self.feeder.start_feed()),
                    self.shooter,
                    self.feeder,
                ).withTimeout(8.0),
            ).finallyDo(_stop_shoot_sequence),
        )

        # Spin up only (no feeder) -- use at start of action paths to pre-spin
        NamedCommands.registerCommand(
            "SpinUpShooter", cmd.runOnce(self.shooter.resolve_distance_and_spin, self.shooter)
        )

        # Stop everything
        def _stop_all() -> None:
            self.shooter.stop_shooter()
            self.feeder.stop_all()

        NamedCommands.registerCommand("StopAll", cmd.runOnce(_stop_all, self.shooter, self.feeder))

        # Intake control
        NamedCommands.registerCommand(
            "StartIntake", cmd.runOnce(lambda: self.feeder.intake_command().schedule(), self.feeder)
        )

        NamedCommands.registerCommand("StopIntake", cmd.runOnce(self.feeder.stop_all, self.feeder))

        # 3-second wait (outpost human player reload)
        NamedCommands.registerCommand("Wait3Sec", cmd.waitSeconds(3.0))

    def _feed_command(self) -> Command:
        """Feed only -- runs the feeder in feed direction. Requires only Feeder, so it
        runs concurrently with spinUpCommand (Y button)."""
        return cmd.run(self.feeder.start_feed, self.feeder).finallyDo(lambda interrupted: self.feeder.stop_all())

    def _stop_all_command(self) -> Command:
        """Emergency stop -- immediately halts shooter wheel and feeder motors. Has NO
        subsystem requirements so it is always schedulable regardless of what is running,
        including commands with kCancelIncoming. It directly cancels any active commands
        on both subsystems, then stops the motors immediately."""

        def _run() -> None:
            shooter_command = self.shooter.getCurrentCommand()
            if shooter_command is not None:
                shooter_command.cancel()
            feeder_command = self.feeder.getCurrentCommand()
            if feeder_command is not None:
                feeder_command.cancel()
            self.shooter.stop_shooter()
            self.feeder.stop_all()

        return cmd.runOnce(_run)

    def _configure_bindings(self) -> None:
        """Configure button-to-command bindings.

        Driver (port 0) -- drive motions only:
          Left Y / Right Y = tank drive (robot-relative)
          RT               = speed boost (analog, 70% -> 100%)
          Back             = re-seed pose from vision
          Start            = toggle reverse driving

        Operator (port 1) -- intake and shooting:
          Y        = toggle shooter spin-up to distance-resolved RPM / coast stop
          RT       = feed while held
          B        = stop all
          LB       = toggle intake
          RB       = toggle removal
          POV      = override distance preset while held (5/7.5/10/12.5/15/17.5/18.75 ft)
          LT       = reverse shooter at 50% power while held
        """
        # ---- Driver ----
        self._driver_controller.back().onTrue(
            cmd.runOnce(lambda: self.drivetrain.initialize_pose(None), self.drivetrain)
        )

        self._driver_controller.start().onTrue(
            cmd.runOnce(self.drivetrain.toggle_reverse_driving, self.drivetrain)
        )

        # ---- Operator ----
        self._operator_controller.y().toggleOnTrue(self.shooter.spin_up_command())

        self._operator_controller.rightTrigger().whileTrue(self.feeder.shoot_command())

        self._operator_controller.b().onTrue(self._stop_all_command())

        self._operator_controller.leftBumper().whileTrue(self.feeder.intake_command())

        self._operator_controller.rightBumper().whileTrue(self.feeder.eject_command())

        # POV: override distance preset while held. N=5ft, NE=7.5ft, E=10ft, SE=12.5ft,
        # S=15ft, SW=17.5ft, W=max(18.75ft)
        for angle, distance_ft in (
            (0, 5.0),
            (45, 7.5),
            (90, 10.0),
            (135, 12.5),
            (180, 15.0),
            (225, 17.5),
            (270, 18.75),
        ):
            self._operator_controller.pov(angle).whileTrue(
                cmd.startEnd(
                    lambda d=distance_ft: self.shooter.set_distance_preset(d),
                    self.shooter.clear_distance_preset,
                )
            )

        self._operator_controller.leftTrigger().whileTrue(
            cmd.startEnd(self.shooter.reverse_shooter, self.shooter.stop_shooter, self.shooter)
        )

    def initialize_pose(self) -> None:
        """Seeds the drivetrain pose from vision at teleop init (no auto command available)."""
        self.drivetrain.initialize_pose(None)

    def get_autonomous_command(self) -> Command:
        """Use this to pass the autonomous command to the main Robot class."""
        selected_auto = self._auto_chooser.getSelected()
        if selected_auto is None:
            selected_auto = self._do_nothing_auto
        self.drivetrain.initialize_pose(selected_auto)  # seed pose from vision or auto path before running
        return selected_auto
