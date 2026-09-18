"""PathPlanner NamedCommands registration for autonomous routines.

Must be called before any PathPlannerAuto is constructed -- PathPlanner
looks these up by name at auto-load time. Split out of RobotContainer to
mirror how established RobotPy teams (e.g. FRC 2429) keep autonomous-only
wiring separate from teleop command bindings (commands/). Behavior and
command names are unchanged from the original inline versions in
robotcontainer.py so existing PathPlanner auto files referencing these
names keep working.
"""

from commands2 import cmd
from pathplannerlib.auto import NamedCommands

from subsystems.feeder import Feeder
from subsystems.shooter import Shooter


def register(shooter: Shooter, feeder: Feeder) -> None:
    def _stop_shoot_sequence(interrupted: bool) -> None:
        shooter.stop_shooter()
        feeder.stop_all()

    # Shoot sequence:
    #   Phase 1 -- spin shooter up to distance-resolved RPM, wait for speed (max 2s)
    #   Phase 2 -- run shooter + feeder together for 8 seconds
    #   Cleanup -- stop everything
    NamedCommands.registerCommand(
        "Shoot5Sec",
        cmd.sequence(
            cmd.run(shooter.resolve_distance_and_spin, shooter)
            .until(shooter.is_at_target_speed)
            .withTimeout(2.0),
            cmd.run(
                lambda: (shooter.resolve_distance_and_spin(), feeder.start_feed()),
                shooter,
                feeder,
            ).withTimeout(8.0),
        ).finallyDo(_stop_shoot_sequence),
    )

    # Shoot for 8 seconds regardless of speed
    NamedCommands.registerCommand(
        "Shoot",
        cmd.sequence(
            cmd.run(
                lambda: (shooter.resolve_distance_and_spin(), feeder.start_feed()),
                shooter,
                feeder,
            ).withTimeout(8.0),
        ).finallyDo(_stop_shoot_sequence),
    )

    # Spin up only (no feeder) -- use at start of action paths to pre-spin
    NamedCommands.registerCommand(
        "SpinUpShooter", cmd.runOnce(shooter.resolve_distance_and_spin, shooter)
    )

    # Stop everything
    def _stop_all() -> None:
        shooter.stop_shooter()
        feeder.stop_all()

    NamedCommands.registerCommand("StopAll", cmd.runOnce(_stop_all, shooter, feeder))

    # Intake control
    NamedCommands.registerCommand(
        "StartIntake", cmd.runOnce(lambda: feeder.intake_command().schedule(), feeder)
    )

    NamedCommands.registerCommand("StopIntake", cmd.runOnce(feeder.stop_all, feeder))

    # 3-second wait (outpost human player reload)
    NamedCommands.registerCommand("Wait3Sec", cmd.waitSeconds(3.0))
