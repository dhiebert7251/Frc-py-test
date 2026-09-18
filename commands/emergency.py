"""Cross-subsystem composite commands that aren't tied to autonomous.

Split out of RobotContainer to mirror how established RobotPy teams (e.g.
FRC 2429's comp_bot/commands) separate ad-hoc teleop composite commands from
subsystem-owned single-purpose commands (subsystems/*.py) and PathPlanner
named commands (autonomous/named_commands.py). Behavior is unchanged from
the original inline versions in robotcontainer.py.
"""

from commands2 import Command, cmd

from subsystems.feeder import Feeder
from subsystems.shooter import Shooter


def stop_all_command(shooter: Shooter, feeder: Feeder) -> Command:
    """Emergency stop -- immediately halts shooter wheel and feeder motors. Has NO
    subsystem requirements so it is always schedulable regardless of what is running,
    including commands with kCancelIncoming. It directly cancels any active commands
    on both subsystems, then stops the motors immediately."""

    def _run() -> None:
        shooter_command = shooter.getCurrentCommand()
        if shooter_command is not None:
            shooter_command.cancel()
        feeder_command = feeder.getCurrentCommand()
        if feeder_command is not None:
            feeder_command.cancel()
        shooter.stop_shooter()
        feeder.stop_all()

    return cmd.runOnce(_run)


def feed_command(feeder: Feeder) -> Command:
    """Feed only -- runs the feeder in feed direction. Requires only Feeder, so it
    runs concurrently with Shooter.spin_up_command() (Y button)."""
    return cmd.run(feeder.start_feed, feeder).finallyDo(lambda interrupted: feeder.stop_all())
