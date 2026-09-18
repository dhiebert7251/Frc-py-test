"""Unit tests for DriveTrain's encoder-distance bookkeeping, odometry, and
its PID autonomous commands.

Uses the `robot`/`control` fixtures provided by `python -m robotpy test`
(pyfrc's pytest plugin, registered automatically by that command).

A correction worth flagging, found while writing this file's odometry
tests: `physics.py`'s `PhysicsEngine` DOES run under `python -m robotpy
test`, not only under `python -m robotpy sim` as an earlier version of this
comment claimed. Every loop, it recomputes each simulated motor's encoder
reading (and the navX's simulated yaw) from real motor physics and writes
that over whatever was there before -- including a value a test just poked
in directly. That's harmless for the two PID-command tests below: they poke
a sensor, then immediately check the SAME loop's `isFinished()`-driven
`isScheduled()` result, and the poked value is read by the command before
physics.py's own recomputation overwrites it a moment later. It is NOT
harmless for testing odometry's actual output value after the fact, since
by the time a later assertion reads `get_pose()`, physics.py has already
written the sensors back to whatever real (near-zero, since nothing
commanded real motion) motion it computed instead. The odometry tests below
work around this by poking the encoders/gyro and then calling
`drivetrain.periodic()` directly -- bypassing the scheduler (and therefore
physics.py's hook into it) entirely -- to get one deterministic odometry
update from a known sensor state.
"""

import pytest
import wpilib.simulation
from wpimath.geometry import Pose2d, Rotation2d

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

        # Same SimDevice mechanism physics.py itself uses to drive the navX
        # gyro -- poked directly here to fake "the robot finished turning."
        # physics.py will overwrite this a moment later with its own
        # (near-zero, since nothing is really commanding a fast turn)
        # computed yaw, but TurnToAngleCommand's isFinished() reads this
        # poked value first, in the same loop -- see this file's module
        # docstring.
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


def test_pose_starts_at_origin(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        pose = drivetrain.get_pose()
        assert pose.X() == 0.0
        assert pose.Y() == 0.0
        assert pose.rotation().degrees() == 0.0


def test_odometry_tracks_straight_line_driving(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        # Poke both encoders to a known distance, then call periodic()
        # directly instead of control.step_timing() -- calling it directly
        # runs exactly one odometry update from this exact sensor state
        # without going through the CommandScheduler, which is what would
        # otherwise let physics.py's own simulated motor model immediately
        # overwrite this poke (see this file's module docstring). Heading
        # is left at 0, so odometry should report having moved straight
        # down the field's X axis by exactly this distance.
        drivetrain._left_encoder.setPosition(2.0)
        drivetrain._right_encoder.setPosition(2.0)
        drivetrain.periodic()

        pose = drivetrain.get_pose()
        assert pose.X() == pytest.approx(2.0, abs=0.01)
        assert pose.Y() == pytest.approx(0.0, abs=0.01)


def test_reset_pose_seeds_odometry_and_zeroes_encoders(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        seeded_pose = Pose2d(5.0, 1.0, Rotation2d.fromDegrees(90))
        drivetrain.reset_pose(seeded_pose)

        assert drivetrain.get_left_distance_meters() == 0.0
        assert drivetrain.get_right_distance_meters() == 0.0
        pose = drivetrain.get_pose()
        assert pose.X() == pytest.approx(5.0, abs=0.01)
        assert pose.Y() == pytest.approx(1.0, abs=0.01)
        assert pose.rotation().degrees() == pytest.approx(90.0, abs=0.5)


def test_chassis_speeds_zero_when_stopped(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        speeds = drivetrain.get_chassis_speeds()
        assert speeds.vx == pytest.approx(0.0, abs=1e-6)
        assert speeds.omega == pytest.approx(0.0, abs=1e-6)
