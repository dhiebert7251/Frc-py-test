"""Simulation physics model for `python -m robotpy sim`.

RobotPy loads this file automatically (by convention, a `physics.py` at the
project root next to robot.py) and constructs `PhysicsEngine` once the real
robot's hardware objects exist, so it can read what they were commanded to
do and feed a simulated physical response back.

Only the drivetrain is modeled here: the two REV SparkMax drive motors are
read via rev.SparkMaxSim and converted to chassis speeds with the same
DifferentialDriveKinematics the real DriveTrain uses, then fed to the field
simulator so Field2d/PathPlanner-in-sim has a real pose to work with. The
navX gyro is driven the same way real hardware reports it: through its
SimDevice ("navX-Sensor[4]" for the MXP SPI port), not by writing to the AHRS
object directly.

Not modeled: Shooter flywheel and Feeder motor dynamics -- those will show
their commanded setpoints in sim rather than a simulated physical response
(e.g. Shooter.get_current_rpm() will read back whatever RPM the Phoenix 6
simulated velocity signal defaults to, not a real flywheel spin-up curve).

This file could not be exercised through the full `robotpy sim` harness in
the environment this port was written in (no display/GUI backend available)
-- the underlying pieces (rev.SparkMaxSim, PhysicsInterface.drive(), the
navX SimDevice mechanism) were each verified individually against the
installed packages, but the assembled whole should be smoke-tested with
`python -m robotpy sim` before being trusted.
"""

from __future__ import annotations

import math

import rev
import wpilib.simulation
from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveKinematics, DifferentialDriveWheelSpeeds
from wpimath.system.plant import DCMotor

from constants import DriveTrainConstants


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot) -> None:
        self.physics_controller = physics_controller

        drivetrain = robot.robot_container.drivetrain
        neo = DCMotor.NEO(1)

        # Mirrors what the real SparkMax controllers report in hardware mode.
        self._left_sim = rev.SparkMaxSim(drivetrain._left_motor_lead, neo)
        self._right_sim = rev.SparkMaxSim(drivetrain._right_motor_lead, neo)

        self._kinematics = DifferentialDriveKinematics(DriveTrainConstants.TRACK_WIDTH_METERS)

        # Wheel surface speed (m/s) at 100% duty cycle: free speed (rad/s) geared
        # down by GEAR_RATIO, times wheel radius.
        self._max_wheel_speed_mps = (
            neo.freeSpeed
            / DriveTrainConstants.GEAR_RATIO
            * (DriveTrainConstants.WHEEL_DIAMETER_METERS / 2.0)
        )

        # navX yaw is driven through its SimDevice -- the same mechanism the real
        # sensor uses to report Yaw over NetworkTables. NavXComType.kMXP_SPI
        # registers as SPI port 4, hence "navX-Sensor[4]".
        navx_sim = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        navx_sim.getBoolean("Connected").set(True)
        self._navx_yaw_sim = navx_sim.getDouble("Yaw")
        self._yaw_degrees = 0.0

    def update_sim(self, now: float, tm_diff: float) -> None:
        # DriveTrain._configure_motors() inverts the right lead motor so positive
        # output drives forward on both sides; undo that here so the kinematics
        # below sees true left/right wheel speeds.
        left_speed = self._left_sim.getAppliedOutput() * self._max_wheel_speed_mps
        right_speed = -self._right_sim.getAppliedOutput() * self._max_wheel_speed_mps

        chassis_speeds: ChassisSpeeds = self._kinematics.toChassisSpeeds(
            DifferentialDriveWheelSpeeds(left_speed, right_speed)
        )

        self.physics_controller.drive(chassis_speeds, tm_diff)

        # DriveTrain.get_heading() negates the raw navX angle (see the comment
        # there), so drive the sim yaw with the negated chassis rotation to match.
        self._yaw_degrees -= math.degrees(chassis_speeds.omega) * tm_diff
        self._navx_yaw_sim.set(self._yaw_degrees)

        # Advance each SparkMaxSim's own position/velocity bookkeeping so
        # DriveTrain.get_left_distance_meters()/get_wheel_speeds() (which read
        # back through the encoder sim) reflect this same motion.
        self._left_sim.iterate(left_speed, 12.0, tm_diff)
        self._right_sim.iterate(-right_speed, 12.0, tm_diff)
