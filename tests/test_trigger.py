"""Unit tests for Trigger's FireCommand edge-detection state machine.

The cam has only one sensor (a limit switch at "home"), so "one fire" is
defined as: leave home, then come back to home. These tests exercise that
logic directly against the DigitalInput simulation, without needing a real
cam mechanism. FireCommand itself has no built-in timeout (see its
docstring) -- the second test below applies the same `.withTimeout()`
decorator robotcontainer.py binds it with, to prove the safety timeout
actually works.
"""

import wpilib.simulation

from commands.trigger_commands import FireCommand
from constants import TriggerConstants


def test_fire_command_finishes_when_cam_returns_home(control, robot):
    with control.run_robot():
        trigger = robot.robot_container.trigger
        limit_switch_sim = wpilib.simulation.DIOSim(TriggerConstants.LIMIT_SWITCH_DIO_PORT)

        # Start "at home" (limit switch reads True, not inverted).
        limit_switch_sim.setValue(True)
        assert trigger.is_at_home()

        # A command can only be scheduled while the robot is enabled (the
        # default runsWhenDisabled() is False), so enable it first.
        control.step_timing(seconds=0.02, autonomous=False, enabled=True)
        command = FireCommand(trigger)
        command.schedule()
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)
        # Still "at home" on the very first tick -- command must not report
        # finished until it has actually left home at least once.
        assert command.isScheduled()

        # Simulate the cam leaving home.
        limit_switch_sim.setValue(False)
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)
        assert command.isScheduled()

        # Simulate the cam returning home -- command should finish now.
        limit_switch_sim.setValue(True)
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)
        assert not command.isScheduled()


def test_fire_command_times_out_if_never_returns_home(control, robot):
    with control.run_robot():
        trigger = robot.robot_container.trigger
        limit_switch_sim = wpilib.simulation.DIOSim(TriggerConstants.LIMIT_SWITCH_DIO_PORT)

        limit_switch_sim.setValue(False)  # never at home -- simulates a jam
        control.step_timing(seconds=0.02, autonomous=False, enabled=True)
        # .withTimeout() is the decorator robotcontainer.py actually binds
        # FireCommand with -- applying it here too is what proves the timeout
        # (not just the edge-detection logic) really stops a jammed cam.
        command = FireCommand(trigger).withTimeout(TriggerConstants.FIRE_TIMEOUT_SECONDS)
        command.schedule()
        control.step_timing(
            seconds=TriggerConstants.FIRE_TIMEOUT_SECONDS + 0.5, autonomous=False, enabled=True
        )
        assert not command.isScheduled()
