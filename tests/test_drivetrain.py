"""Unit tests for DriveTrain's encoder-distance bookkeeping and its two PID
autonomous commands.

Uses the `robot`/`control` fixtures provided by `python -m robotpy test`
(pyfrc's pytest plugin, registered automatically by that command). There's
no physics engine running under `robotpy test` (that only happens under
`robotpy sim`, via physics.py) -- commanding a motor here does NOT move a
simulated encoder on its own. So instead of driving the PID loop to
convergence for real, these tests poke the simulated encoder/gyro readings
directly (the same trick test_trigger.py and test_elevator.py use on their
DigitalInput sensors) to simulate "the robot got there," and check that the
command notices and stops.
"""

import wpilib.simulation

from commands.drivetrain_commands import DriveDistanceCommand, TurnToAngleCommand
from constants import Auto, METERS_PER_FOOT


def test_average_distance_starts_at_zero(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        assert drivetrain.get_average_distance_meters() == 0.0


def test_reset_encoders_zeroes_distance(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        drivetrain.reset_encoders()
        assert drivetrain.get_left_distance_meters() == 0.0
        assert drivetrain.get_right_distance_meters() == 0.0


def test_drive_distance_command_finishes_once_target_reached(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        # A command can only be scheduled while the robot is enabled (the
        # default runsWhenDisabled() is False), so enable it first.
        control.step_timing(seconds=0.02, autonomous=True, enabled=True)

        command = DriveDistanceCommand(drivetrain, 1.0)  # 1 foot
        command.schedule()
        control.step_timing(seconds=0.1, autonomous=True, enabled=True)
        assert command.isScheduled()  # nowhere near the target yet

        # Simulate the robot having driven all the way there by writing the
        # target distance straight onto both encoders.
        target_meters = 1.0 * METERS_PER_FOOT
        drivetrain._left_encoder.setPosition(target_meters)
        drivetrain._right_encoder.setPosition(target_meters)
        control.step_timing(seconds=0.1, autonomous=True, enabled=True)
        assert not command.isScheduled()


def test_turn_to_angle_command_finishes_once_heading_reached(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        control.step_timing(seconds=0.02, autonomous=True, enabled=True)

        # Same SimDevice mechanism physics.py uses to drive the navX gyro in
        # `robotpy sim` -- poked directly here since there's no physics
        # engine running under `robotpy test`.
        navx_sim = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        yaw_sim = navx_sim.getDouble("Yaw")

        command = TurnToAngleCommand(drivetrain, 90.0)
        command.schedule()
        control.step_timing(seconds=0.1, autonomous=True, enabled=True)
        assert command.isScheduled()

        # get_heading_degrees() negates the raw navX yaw (see DriveTrain),
        # so -90 raw yaw simulates having reached +90 degrees heading.
        yaw_sim.set(-90.0)
        control.step_timing(seconds=0.1, autonomous=True, enabled=True)
        assert not command.isScheduled()


def test_auto_routines_build_without_error(control, robot):
    with control.run_robot():
        import autonomous.routines as routines

        drivetrain = robot.robot_container.drivetrain
        assert routines.drive_forward_only(drivetrain) is not None
        assert routines.drive_turn_drive(drivetrain) is not None
        assert Auto.DRIVE_FORWARD_ONLY_FEET > 0
