# FRC Team 2408 -- RobotPy Port (Feasibility Test)

This is an AI-assisted RobotPy (Python) port of
[2026_Competition_Code](https://github.com/ShrapnelSergeants2408/2026_Competition_Code),
Team 2408's Java competition codebase for the 2026 season. It exists to
test the current state of WPILib's Python support as a candidate control
system, ahead of any decision to actually switch languages.

**This is not the competition codebase.** The Java repository's policy against
directly AI-generated code applies to the team's actual competition robot
code; this port is explicitly AI-generated as a starting point for a
feasibility evaluation, at the request of a team member, and should be
reviewed by a human before being trusted, extended, or run on a real robot.

## What was ported

Everything in the Java source as of the port date, 1:1 in behavior:

- `constants.py` -- `Constants.java`
- `vision_measurement.py` -- `VisionMeasurement.java`
- `result.py` -- `Result.java` (test-result helper for the future component test mode)
- `subsystems/drivetrain.py` -- `DriveTrain.java` (differential/tank drive, REV SparkMax
  + NavX gyro, vision-fused `DifferentialDrivePoseEstimator`, PathPlanner autonomous
  with a tunable `PPLTVController`)
- `subsystems/vision.py` -- `Vision.java` (dual PhotonVision cameras, AprilTag pose
  estimation, multi-tag quality gating, hub/zone distance helpers)
- `subsystems/shooter.py` -- `Shooter.java` (CTRE Phoenix 6 TalonFX flywheel, velocity
  PID, distance-to-RPM interpolation table)
- `subsystems/feeder.py` -- `Feeder.java` (REV SparkMax intake + trigger/hopper, state
  machine; jam detection ported but left disabled, matching the Java source)
- `subsystems/climber.py`, `subsystems/intake.py` -- stubs, same as the Java source
  (no hardware/logic implemented yet)
- `robotcontainer.py` -- `RobotContainer.java` (subsystems, controller bindings,
  PathPlanner named commands, autonomous chooser)
- `robot.py` -- `Robot.java`

Known in-progress state from the Java source was kept as-is: unverified motor
inversions and camera-mount measurements (marked `TODO` in `constants.py`),
the bypassed `Shooter.can_shoot()` zone/sensor check (`return True`), and
disabled jam-clearing in `Feeder`. Genuinely dead/commented-out code from the
Java version was cleaned up rather than transliterated.

## Where this port differs from the Java version

- **AdvantageKit is dropped.** It has no RobotPy port. This version uses a plain
  `commands2.TimedCommandRobot` (which schedules `CommandScheduler` automatically)
  plus `wpilib.DataLogManager` for on-disk + NetworkTables logging. There is no
  AdvantageScope-style replay-log-swap in simulation.
- **Phoenix5 vendordep is dropped.** It was present in the Java project's
  `vendordeps/` but never actually imported/used in code (only Phoenix 6 is used,
  for the shooter TalonFX).
- **PhotonVision pose strategy.** photonlibpy 2026 removed `PoseStrategy` from
  `PhotonPoseEstimator`'s constructor; instead of selecting a strategy once, you
  call the estimate method for the strategy you want, per result.
  `MULTI_TAG_PNP_ON_COPROCESSOR` (which internally falls back to a
  lowest-ambiguity single-tag estimate in Java) is reproduced explicitly in
  `Vision.py` as `estimateCoprocMultiTagPose()` falling back to
  `estimateLowestAmbiguityPose()`.
- **PathPlanner auto starting pose.** `pathplannerlib-python` 2026.1.2 has no public
  `PathPlannerAuto.getStartingPose()` (it's computed into a private attribute
  internally). `DriveTrain.py` reconstructs the same value from
  `PathPlannerAuto.getPathGroupFromAutoFile(name)` +
  `PathPlannerPath.getStartingDifferentialPose()`.
- **`AutoBuilder.configure()`'s `output` callback** takes
  `(ChassisSpeeds, DriveFeedforwards)` in this binding, vs. a single
  `ChassisSpeeds` in Java. `drive_robot_relative()` accepts an optional,
  currently-unused `feedforwards` argument to satisfy that.
- **Driver camera.** Uses `cscore.CameraServer.startAutomaticCapture(...)` directly
  (RobotPy's `wpilib.CameraServer` is a separate out-of-process launcher, not a
  drop-in for Java's in-process `edu.wpi.first.cameraserver.CameraServer`).
- **`Result.pass(...)`/`fail(...)`** are renamed `passed(...)`/`failed(...)` since
  `pass` is a reserved word in Python.
- Vision standard deviations are plain `(x_m, y_m, heading_rad)` tuples rather
  than a `Matrix<N3, N1>` -- that's the type RobotPy's
  `DifferentialDrivePoseEstimator.addVisionMeasurement()` actually takes.

## Setup

This project has **no PathPlanner GUI settings or autonomous paths configured
yet** (the Java project's `deploy/pathplanner/` is likewise empty). Until you
open this project in the PathPlanner GUI and save a robot config + at least one
path/auto, `RobotConfig.fromGUISettings()` will fail and get caught (matching
the Java behavior), and the autonomous chooser will only offer "Do Nothing".

1. Install [RobotPy](https://docs.wpilib.org/en/stable/docs/software/python/index.html)
   and sync dependencies for this project (`py -m robotpy sync` /
   `python -m robotpy sync`, run from this directory) -- this reads
   `pyproject.toml` and installs the pinned versions of `robotpy`,
   `robotpy-rev`, `robotpy-navx`, `robotpy-pathplannerlib`, `photonlibpy`, and
   `phoenix6` used here.
2. Simulate: `python -m robotpy sim`
3. Deploy to a roboRIO: `python -m robotpy deploy`

Camera names (`Front_Camera`, `Rear_Camera`, `Driver_Camera`) must match your
PhotonVision coprocessor configuration, same as the Java version.

## Verification

Every RobotPy/vendor-library API used here (REVLib's config API, Phoenix 6's
Python bindings, photonlibpy's 2026 pose-estimator API, pathplannerlib-python,
robotpy-navx) was checked against the actual installed packages -- not just
assumed from the Java API -- and `RobotContainer`, `Robot`, and every
subsystem's `periodic()` were exercised end-to-end under a simulated HAL
before this was pushed. That confirms the code constructs and runs without
crashing; it does **not** confirm correct behavior on a real robot (motor
directions, camera mounting transforms, and PID gains all still carry the
same "TODO: verify on bench" caveats the Java version has).

**Recommended before trusting this on a real robot:** an independent
read-through by someone on the team who knows both the Java original and
Python, in a fresh review pass -- particularly `subsystems/drivetrain.py`'s
PathPlanner integration and `subsystems/vision.py`'s pose-estimator fallback
logic, which involved real API differences rather than 1:1 syntax translation.
