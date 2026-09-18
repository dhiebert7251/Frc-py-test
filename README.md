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
- `robotcontainer.py` -- `RobotContainer.java` (subsystem wiring, controller bindings)
- `robot.py` -- `Robot.java`
- `autonomous/named_commands.py`, `autonomous/chooser.py` -- PathPlanner named-command
  registration and autonomous chooser construction, both originally inline in
  `RobotContainer.java`
- `commands/emergency.py` -- the cross-subsystem stop-all/feed-only composite commands,
  also originally inline in `RobotContainer.java`

Known in-progress state from the Java source was kept as-is: unverified motor
inversions and camera-mount measurements (marked `TODO` in `constants.py`),
the bypassed `Shooter.can_shoot()` zone/sensor check (`return True`), and
disabled jam-clearing in `Feeder`. Genuinely dead/commented-out code from the
Java version was cleaned up rather than transliterated.

## Project structure

Restructured from a flat `subsystems.py` + inline-everything layout to match
how established RobotPy teams organize larger projects (see
[`comp_bot/`](https://github.com/aesatchien/FRC2429_2026/tree/main/comp_bot) in
FRC 2429's 2026 codebase, one of the few active teams running RobotPy in
competition):

```
constants.py, vision_measurement.py, result.py   # plain data/config, no hardware
robot.py, robotcontainer.py                       # entry point + subsystem/binding wiring
physics.py                                        # `robotpy sim` drivetrain model
subsystems/    # one hardware subsystem per file (unchanged from the original port)
commands/      # cross-subsystem teleop composite commands (currently: emergency stop)
autonomous/    # PathPlanner named-command registration + autonomous chooser
tests/         # pytest, run via `python -m robotpy test`
deploy/        # files copied to the RoboRIO's deploy directory at deploy time
```

Single-subsystem commands (e.g. `Shooter.spin_up_command()`,
`Feeder.intake_command()`) stay as subsystem methods rather than moving into
`commands/` -- that's also a common, valid pattern in RobotPy projects, and
moving them would mean rewriting them as full `Command` subclasses (they're
currently closures wrapped by `commands2.cmd.run(...).finallyDo(...)`), which
is a larger, riskier change than the structural split this pass was scoped to.

### A pre-existing gap this restructuring surfaced (and fixed)

`RobotContainer`'s constructor calls `AutoBuilder.buildAutoChooser()`
unguarded right after `DriveTrain` already catches a failed
`AutoBuilder.configure()` -- both the Java source and the first version of
this port would crash the whole robot on startup whenever PathPlanner isn't
configured yet (no `deploy/pathplanner/settings.json`), which is this
project's actual current state. `autonomous/chooser.py` now checks
`AutoBuilder.isConfigured()` first and falls back to a chooser offering only
"Do Nothing" instead of raising. This is the one place this pass changed
behavior rather than just moving code -- called out here because it's a
deliberate, small deviation from a strict 1:1 port, made because the new
`tests/` suite (see below) needs the robot to actually come up.

## Running tests

```
python -m robotpy test
```

This uses `pyfrc`'s pytest plugin (bundled with `robotpy`), which provides
`robot` and `control` fixtures that construct the real `Robot`/`RobotContainer`
under a simulated HAL and step them through disabled/autonomous/teleop --
these fixtures aren't available under a bare `pytest` invocation.
`tests/pyfrc_test.py` holds pyfrc's generic mode-transition tests
(`python -m robotpy add-tests` regenerates it); `test_shooter.py` checks the
distance-to-RPM table against the same values as `Shooter.java`;
`test_robot_lifecycle.py` is an automated version of the manual
disabled→auto→teleop check this port was verified with before its first push;
`test_result.py` is a plain unit test with no hardware involved. All 10 tests
pass as of this restructuring.

## Simulating (`physics.py`)

`python -m robotpy sim` now has a real drivetrain model to run against
instead of a frozen field widget: `physics.py` reads the two simulated
SparkMax drive motors, converts their output to chassis speeds with the same
kinematics `DriveTrain` uses, and feeds that back to move the simulated robot
(and drives the navX's `SimDevice` so simulated heading matches). Shooter and
Feeder motor dynamics are **not** modeled -- their telemetry in sim reflects
commanded setpoints, not a simulated physical response. This file's
individual pieces (`rev.SparkMaxSim`, `PhysicsInterface.drive()`, the navX
`SimDevice` mechanism) were each verified against the installed packages, and
it ran without raising across a full disabled→auto→teleop cycle in
`test_robot_lifecycle.py`, but the environment this port was written in had
no GUI backend to actually run `python -m robotpy sim` interactively -- do
that before trusting the simulated motion looks physically reasonable.

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

## 2027 season status

This branch stays on the pinned 2026.x package versions above. A separate
`2027-alpha-tracking` branch documents what was checked against the current
2027 alpha (`robotpy==2027.0.0a6`/`a7`): as of that check, **none** of the
vendor libraries this robot depends on (REV, navX, PathPlanner, PhotonVision,
Phoenix 6) have a 2027 release, and neither does `robotpy-commands-v2`
(the `commands2` package) -- so the Command-based architecture this whole
port relies on isn't available for 2027 yet either. WPILib's own math/geometry
API is also being restructured for 2027 (flattened namespaces, some classes
renamed), so this won't be a drop-in version bump once those catch up. See
that branch's README for the specifics and links to the upstream tracking
issues.

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
robotpy-navx, pyfrc's simulation/test APIs) was checked against the actual
installed packages -- not just assumed from the Java API. `RobotContainer`,
`Robot`, and every subsystem's `periodic()` were exercised end-to-end under a
simulated HAL, and `python -m robotpy test` (10 tests, see above) passes.
That confirms the code constructs and runs without crashing; it does **not**
confirm correct behavior on a real robot (motor directions, camera mounting
transforms, and PID gains all still carry the same "TODO: verify on bench"
caveats the Java version has), and `python -m robotpy sim`'s GUI was never
actually run interactively in the environment this was written in.

**Recommended before trusting this on a real robot:** an independent
read-through by someone on the team who knows both the Java original and
Python, in a fresh review pass -- particularly `subsystems/drivetrain.py`'s
PathPlanner integration and `subsystems/vision.py`'s pose-estimator fallback
logic, which involved real API differences rather than 1:1 syntax translation.
