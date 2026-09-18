"""Integration test for Shooter's distance-to-RPM interpolation table.

Uses the `robot`/`control` fixtures provided by `python -m robotpy test`
(pyfrc's pytest plugin, registered automatically by that command -- these
fixtures are not available under a bare `pytest` invocation). robotInit()
has already run once the `with control.run_robot():` block starts, so
robot.robot_container's subsystems exist.
"""

from constants import ShooterConstants


def test_rpm_matches_table_endpoints(control, robot):
    with control.run_robot():
        shooter = robot.robot_container.shooter

        # Below/above the table's range clamp to the nearest endpoint.
        assert shooter.get_rpm_from_distance(0.0) == ShooterConstants.DISTANCE_RPM_MAP[0]
        assert shooter.get_rpm_from_distance(100.0) == ShooterConstants.DISTANCE_RPM_MAP[-1]

        # Exact table entries.
        for distance_ft, rpm in zip(ShooterConstants.DISTANCES_FEET, ShooterConstants.DISTANCE_RPM_MAP):
            assert shooter.get_rpm_from_distance(distance_ft) == rpm


def test_rpm_interpolates_between_points(control, robot):
    with control.run_robot():
        shooter = robot.robot_container.shooter

        # Halfway between 5 ft (2150 rpm) and 7.5 ft (2400 rpm) -> 6 ft is 40% of
        # the way there: 2150 + 0.4 * (2400 - 2150) = 2250.
        assert shooter.get_rpm_from_distance(6.0) == 2250.0
