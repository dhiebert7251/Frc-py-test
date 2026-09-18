"""Unit tests for Elevator's limit-switch-gated raise/lower commands."""

import wpilib.simulation

from constants import ElevatorConstants


def test_raise_command_stops_at_top(control, robot):
    with control.run_robot():
        elevator = robot.robot_container.elevator
        top_sim = wpilib.simulation.DIOSim(ElevatorConstants.TOP_LIMIT_SWITCH_DIO_PORT)

        top_sim.setValue(False)  # not at top
        assert not elevator.is_at_top()

        # A command can only be scheduled while the robot is enabled (the
        # default runsWhenDisabled() is False), so enable it first.
        control.step_timing(seconds=0.02, autonomous=False, enabled=True)
        command = elevator.raise_command()
        command.schedule()
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)
        assert command.isScheduled()  # whileTrue-style: keeps running while held

        top_sim.setValue(True)  # reached the top
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)
        assert elevator.is_at_top()


def test_lower_command_stops_at_bottom(control, robot):
    with control.run_robot():
        elevator = robot.robot_container.elevator
        bottom_sim = wpilib.simulation.DIOSim(ElevatorConstants.BOTTOM_LIMIT_SWITCH_DIO_PORT)

        bottom_sim.setValue(False)  # not at bottom
        assert not elevator.is_at_bottom()

        control.step_timing(seconds=0.02, autonomous=False, enabled=True)
        command = elevator.lower_command()
        command.schedule()
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)
        assert command.isScheduled()

        bottom_sim.setValue(True)  # reached the bottom
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)
        assert elevator.is_at_bottom()
