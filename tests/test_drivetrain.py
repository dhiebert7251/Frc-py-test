"""Unit tests for DriveTrain's encoder-distance bookkeeping and its two PID
autonomous commands.

Uses the `robot`/`control` fixtures provided by `python -m robotpy test`
(pyfrc's pytest plugin, registered automatically by that command).

A correction worth flagging: `physics.py`'s `PhysicsEngine` actually DOES
run under `python -m robotpy test`, not only under `python -m robotpy sim`
as an earlier version of this comment claimed. Every loop, it recomputes
each simulated motor's encoder reading (and the navX's simulated yaw) from
real motor physics and writes that over whatever was there before --
including a value a test just poked in directly. That's harmless for the
two tests below: each pokes a sensor, then immediately checks that SAME
loop's `isFinished()`-driven `isScheduled()` result, and the scheduled
command reads the poked value before physics.py's own recomputation
overwrites it a moment later. It would NOT be harmless for an assertion
that reads a sensor value back out after the fact -- that needs a
different approach (see the `teaching-bot-odometry` branch's
`tests/test_drivetrain.py`, which calls `drivetrain.periodic()` directly
to sidestep the scheduler, and therefore physics.py's hook into it,
entirely).
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
        # target distance straight onto both encoders. physics.py will
        # overwrite this a moment later with its own computed value (see
        # this file's module docstring), but DriveDistanceCommand's
        # isFinished() reads it first, in this same loop.
        target_meters = 1.0 * METERS_PER_FOOT
        drivetrain._left_encoder.setPosition(target_meters)
        drivetrain._right_encoder.setPosition(target_meters)
        control.step_timing(seconds=0.1, autonomous=True, enabled=True)
        assert not command.isScheduled()


def test_turn_to_angle_command_finishes_once_heading_reached(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        control.step_timing(seconds=0.02, autonomous=True, enabled=True)

        # Same SimDevice mechanism physics.py itself uses to drive the navX
        # gyro -- poked directly here to fake "the robot finished turning."
        # See this file's module docstring for why this still works despite
        # physics.py overwriting it again a moment later.
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
