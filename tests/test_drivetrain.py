"""Unit tests for DriveTrain's encoder-distance bookkeeping and autonomous
command construction.

Uses the `robot`/`control` fixtures provided by `python -m robotpy test`
(pyfrc's pytest plugin, registered automatically by that command). These are
mostly "does this build/run without exploding" checks appropriate for a
teaching codebase -- see test_robot_lifecycle.py for the full mode-cycle
smoke test.
"""

from constants import Auto


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


def test_drive_distance_command_finishes_immediately_past_target(control, robot):
    with control.run_robot():
        drivetrain = robot.robot_container.drivetrain
        # A negative target distance is already "reached" the moment the
        # command starts (average distance 0.0 >= a negative number), so the
        # command should finish on its own without needing to be canceled.
        command = drivetrain.drive_distance_command(-1.0)
        command.schedule()
        control.step_timing(seconds=0.5, autonomous=True, enabled=True)
        assert not command.isScheduled()


def test_auto_routines_build_without_error(control, robot):
    with control.run_robot():
        import autonomous.routines as routines

        drivetrain = robot.robot_container.drivetrain
        assert routines.drive_forward_only(drivetrain) is not None
        assert routines.drive_turn_drive(drivetrain) is not None
        assert Auto.DRIVE_FORWARD_ONLY_FEET > 0
