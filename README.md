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

## Contents

- [What was ported](#what-was-ported)
- [File structure](#file-structure)
- [Naming and numbering conventions](#naming-and-numbering-conventions)
- [Subsystems](#subsystems)
- [Commands](#commands)
- [Controller bindings](#controller-bindings)
- [Running tests](#running-tests)
- [Simulating (`physics.py`)](#simulating-physicspy)
- [Where this port differs from the Java version](#where-this-port-differs-from-the-java-version)
- [2027 season status](#2027-season-status)
- [Setup](#setup)
- [Verification](#verification)
- [If your team transitions to Python](#if-your-team-transitions-to-python)

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

## File structure

```
frc-py-test/                          repo root == the RobotPy project root
├── pyproject.toml                    pinned package versions (see Setup)
├── README.md                         this file
├── .wpilib/wpilib_preferences.json   team number + project language, read by the
│                                      VS Code RobotPy extension
├── .gitignore
├── robot.py                          entry point: the Robot class, mode-transition
│                                      callbacks (autonomousInit, teleopInit, ...)
├── robotcontainer.py                 wires subsystems + controllers + button
│                                      bindings together -- read this file first
├── constants.py                      every tunable number/ID in the project,
│                                      grouped into one namespace class per subsystem
├── vision_measurement.py             small data class passed from Vision to
│                                      DriveTrain's pose estimator
├── result.py                         helper for the not-yet-built component
│                                      test mode
├── physics.py                        drivetrain model used only by `robotpy sim`;
│                                      never runs on a real robot
├── subsystems/                       one file per hardware subsystem
│   ├── drivetrain.py                 DriveTrain: tank drive, odometry, PathPlanner
│   ├── vision.py                     Vision: PhotonVision cameras, AprilTag pose
│   ├── shooter.py                    Shooter: flywheel motor, distance-to-RPM table
│   ├── feeder.py                     Feeder: intake + trigger/hopper motors
│   ├── climber.py                    Climber: stub, no hardware yet
│   └── intake.py                     Intake: stub, no hardware yet
├── commands/                         cross-subsystem teleop composite commands
│   └── emergency.py                  stop-all / feed-only
├── autonomous/                       everything PathPlanner-autonomous-specific
│   ├── named_commands.py             commands PathPlanner auto files can call by name
│   └── chooser.py                    builds the SmartDashboard auto-routine picker
├── tests/                            pytest, run via `python -m robotpy test`
│   ├── pyfrc_test.py                 generic disabled/auto/teleop/practice tests
│   │                                 (from pyfrc; regenerate with `robotpy add-tests`)
│   ├── test_robot_lifecycle.py       full disabled->auto->teleop smoke test
│   ├── test_shooter.py               checks the distance->RPM table
│   └── test_result.py                unit test for result.py, no hardware involved
└── deploy/                           files copied onto the robot at deploy time
    └── pathplanner/                  PathPlanner project files (currently empty --
                                       see Setup)
```

**Where to start reading:** `robotcontainer.py` first (it's the map of
everything else), then whichever `subsystems/*.py` file matches what you're
trying to understand, then `constants.py` for the numbers that file uses.

## Naming and numbering conventions

None of this was formally written down in the Java source (its own
`robot.md`/`subsystems.md` docs are still work-in-progress notes, not a style
guide) -- everything under "Team 2408" below is reverse-engineered from what
the code actually does, not a policy anyone wrote. Worth turning into an
actual `CONTRIBUTING.md` if the team keeps this codebase going (see
[If your team transitions to Python](#if-your-team-transitions-to-python)).

### FRC-wide conventions this code follows

- **Field coordinate system.** WPILib's standard since 2023: the origin is
  always at the blue alliance's near corner, X increases downfield (away
  from the blue driver station), Y increases to the left, and rotation is
  counterclockwise-positive with 0 degrees facing downfield. Auto paths are
  authored once for blue and mirrored for red at runtime -- that's what
  `DriveTrain._flip_for_red_alliance()` and PathPlanner's
  `should_flip_path` callback do.
- **SI units internally, imperial at the boundary.** WPILib math (`wpimath`)
  works in meters/radians/seconds even though FRC parts are specified in
  inches and degrees. Conversions happen right where a human-facing constant
  becomes a WPILib call -- see `WHEEL_DIAMETER_METERS` (measured in inches,
  converted once) and the `math.radians(...)` calls in `constants.py`.
- **Controller ports 0 and 1 = driver and operator.** Not a WPILib
  requirement, but close to universal across FRC teams: the Driver Station
  assigns each plugged-in controller a port number, and port 0 is
  conventionally the drive controller.

### Team 2408 conventions (as implemented, not formally documented)

- **CAN IDs are grouped by subsystem, by tens.** `20`-`23` = DriveTrain
  (`20`/`22` = left/right lead motors, `21`/`23` = left/right follower
  motors -- lead and follower alternate rather than being grouped together).
  `30`-`32` = Shooter/Feeder (`30` = shooter flywheel TalonFX, `31` = intake
  roller SparkMax, `32` = trigger/hopper SparkMax). A new subsystem should
  probably claim its own decade (`40`-`49`, etc.) rather than reusing one.
- **DIO port 1** is the (currently disabled) Feeder photo sensor --
  `SensorConstants.PHOTO_SENSOR_DIO_PORT`.
- **Constants live in one file, grouped into a namespace class per
  subsystem.** `constants.py` has `DriveTrainConstants`, `ShooterConstants`,
  `VisionConstants`, `SensorConstants`, `Auto`, `OperatorConstants` --
  matching the Java source's nested static classes inside `Constants.java`.
  A new subsystem's constants belong in a new class here, not scattered
  across its subsystem file.
- **One file per subsystem, filename matches the class it defines,** in
  lowercase: `DriveTrain` -> `drivetrain.py`, `Shooter` -> `shooter.py`.
  Where the name is clearly two words rather than one compound word, the
  file uses an underscore instead of squashing them together --
  `VisionMeasurement` -> `vision_measurement.py`.
- **Command-returning methods on a subsystem end in `_command`:**
  `spin_up_command()`, `intake_command()`, `eject_command()`,
  `shoot_command()`, `teleop_drive_command()`. Grepping for `_command` finds
  every command a subsystem offers.
- **Private/internal state is `_`-prefixed:** `_reverse_driving`,
  `_pov_preset_set`, `_cached_measurement`, etc. -- signals "don't touch this
  from outside the class," though Python doesn't enforce it the way a
  Java `private` keyword does.

### Python (this port) vs. Java (the source) naming

| | Java (source) | Python (this port, [PEP 8](https://peps.python.org/pep-0008/)) |
|---|---|---|
| Functions/methods | `camelCase` -- `getLeftDistanceMeters()` | `snake_case` -- `get_left_distance_meters()` |
| Local variables | `camelCase` -- `leftDistanceMeters` | `snake_case` -- `left_distance_meters` |
| Classes | `PascalCase` -- `DriveTrain` | `PascalCase` -- `DriveTrain` (unchanged) |
| Constants | `ALL_CAPS_WITH_UNDERSCORES` -- `SHOOTER_MOTOR_ID` | `ALL_CAPS_WITH_UNDERSCORES` (unchanged) |
| Files | one class per file, `PascalCase.java` matching the class | one module per file, `snake_case.py` (see above) |

**The one thing that trips people up:** our own code follows the Python
column above, but every call *into* WPILib/RobotPy or a vendor library
(`wpilib`, `commands2`, `rev`, `phoenix6`, `pathplannerlib`, `photonlibpy`)
still uses that library's own naming -- which for everything except
`phoenix6` is still Java-style `camelCase` as of the 2026 season
(`wpilib.SmartDashboard.putNumber(...)`, `self.drivetrain.setDefaultCommand(...)`).
So a single line of this code can legitimately mix both:
`self._left_encoder.getPosition()` -- our snake_case variable, calling a
camelCase method that belongs to REV's library. This isn't inconsistency to
"fix" -- it's a real seam between our code and code we don't own, and it's
worth teaching explicitly rather than letting rookies think someone just
forgot to rename something. (`phoenix6` alone already uses
Python-native `snake_case`/`ALL_CAPS` -- see the
[2027 season status](#2027-season-status) section for why the rest of
WPILib is expected to follow eventually.)

## Subsystems

| Subsystem | File | Purpose | Hardware | Key methods |
|---|---|---|---|---|
| **DriveTrain** | `subsystems/drivetrain.py` | Moves the robot (tank/differential drive) and tracks where it is on the field | 4x REV SparkMax NEO (CAN 20-23), navX2 gyro (SPI/MXP) | `drive()`, `teleop_drive_command()`, `turn_to_angle()`, `get_pose()` / `reset_pose()` / `initialize_pose()`, `get_heading()`, `drive_robot_relative()` / `get_robot_relative_speeds()` (used by PathPlanner) |
| **Vision** | `subsystems/vision.py` | Estimates the robot's field position from AprilTags and feeds it to DriveTrain | 2x PhotonVision coprocessor cameras (front/rear) | `get_best_vision_measurement()` / `get_best_vision_measurement_if_fresh()`, `get_alliance_hub_pose()`, `get_distance_to_*()` / `is_aligned_with_*()` for hub/HP-station/trench/depot/outpost/tower, `is_any_vision_available()` |
| **Shooter** | `subsystems/shooter.py` | Spins the flywheel that launches fuel, at a speed resolved from target distance | 1x CTRE TalonFX (CAN 30) | `set_target_rpm()`, `stop_shooter()`, `reverse_shooter()`, `get_current_rpm()`, `is_at_target_speed()`, `get_rpm_from_distance()`, `set_distance_preset()` / `clear_distance_preset()`, `resolve_distance_and_spin()`, `spin_up_command()` |
| **Feeder** | `subsystems/feeder.py` | Moves fuel from the intake through to the shooter, or ejects it | 2x REV SparkMax NEO (CAN 31 intake roller, CAN 32 trigger/hopper), 1x photo sensor (DIO 1, disabled until installed) | `start_feed()`, `stop_all()`, `has_ball()`, `intake_command()`, `eject_command()`, `shoot_command()` (jam-clear state machine exists but is disabled, matching the Java source) |
| **Climber** | `subsystems/climber.py` | Will let the robot climb/hang at end of match | none yet | stub -- no hardware or logic |
| **Intake** | `subsystems/intake.py` | Placeholder; overlaps in name with Feeder's intake roller, currently unused | none yet | stub -- no hardware or logic |

`Vision` is constructed before `DriveTrain` (passed into its constructor);
`DriveTrain` is constructed before `Shooter` (both `Vision` and `DriveTrain`
are injected into `Shooter`) -- see the comments in `robotcontainer.py` if
you ever need to reorder subsystem construction.

## Commands

**Subsystem-owned** (methods on the subsystem itself -- the most common
pattern in this codebase; see
[Naming and numbering conventions](#naming-and-numbering-conventions)):

| Command | Subsystem(s) | What it does |
|---|---|---|
| `teleop_drive_command()` | DriveTrain | Default command; tank drive from two joystick axes plus an analog boost trigger |
| `turn_to_angle(degrees)` | DriveTrain | PID-turns to a heading; not currently bound to any button, available for future auto/teleop use |
| `spin_up_command()` | Shooter | Toggle: spins the flywheel to the distance-resolved RPM, or coasts to a stop |
| `intake_command()` | Feeder | Runs intake roller + trigger inward while held |
| `eject_command()` | Feeder | Runs intake roller + trigger outward (reverse) while held |
| `shoot_command()` | Feeder | Feeds fuel toward the shooter while held |

**Cross-subsystem composite** (`commands/emergency.py` -- see that module's
docstring for why these live apart from subsystem-owned commands):

| Command | Subsystem(s) | What it does |
|---|---|---|
| `stop_all_command()` | Shooter + Feeder | Emergency stop -- cancels whatever's running on either subsystem and kills both motors immediately; has no subsystem requirement, so it's always schedulable |
| `feed_command()` | Feeder | Feed-only helper carried over from the Java source; not currently bound to any button in either version |

**PathPlanner named commands** (`autonomous/named_commands.py` -- referenced
by name *from PathPlanner auto files*, not from controller bindings; a
human editing an auto in the PathPlanner GUI picks these by name):

| Name | Subsystem(s) | What it does |
|---|---|---|
| `Shoot5Sec` | Shooter + Feeder | Spin up (wait up to 2s for target speed), then run shooter + feeder together for 8s, then stop |
| `Shoot` | Shooter + Feeder | Run shooter + feeder together for 8s regardless of speed, then stop |
| `SpinUpShooter` | Shooter | Spin up once, no feeding -- pre-spin before a `Shoot`/`Shoot5Sec` |
| `StopAll` | Shooter + Feeder | Stop both, once |
| `StartIntake` | Feeder | Schedule `intake_command()` |
| `StopIntake` | Feeder | Stop, once |
| `Wait3Sec` | none | 3-second pause (e.g. waiting on a human player reload) |

## Controller bindings

**Driver (port 0) -- drive motions only:**

| Input | Action |
|---|---|
| Left Y / Right Y | Tank drive (robot-relative) |
| Right Trigger | Speed boost, analog: 70% -> 100% |
| Back | Re-seed field position from vision |
| Start | Toggle reverse driving (robot's rear becomes the front) |

**Operator (port 1) -- intake and shooting:**

| Input | Action |
|---|---|
| Y | Toggle: spin shooter up to distance-resolved RPM / coast to a stop |
| Right Trigger | Feed toward shooter while held |
| B | Stop all -- immediately halts shooter + feeder |
| Left Bumper | Intake while held |
| Right Bumper | Eject (reverse intake) while held |
| Left Trigger | Reverse shooter at 50% power while held (unjam) |
| POV (D-pad) 0° | Distance preset: 5.0 ft, while held |
| POV 45° | Distance preset: 7.5 ft, while held |
| POV 90° | Distance preset: 10.0 ft, while held |
| POV 135° | Distance preset: 12.5 ft, while held |
| POV 180° | Distance preset: 15.0 ft, while held |
| POV 225° | Distance preset: 17.5 ft, while held |
| POV 270° | Distance preset: 18.75 ft, while held |

Distance presets override the automatic vision/odometry-based distance
resolution for as long as they're held; releasing the POV returns to
automatic resolution. See `Shooter._resolve_shooter_distance()` for the full
priority order (POV preset > vision > odometry > default 10 ft in teleop;
always a fixed 10 ft in autonomous).

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
`2027-alpha-tracking` branch reviews the official
[WPILib 2027 yearly changelog](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html)
and cross-checks its claims against the actual installable
`robotpy==2027.0.0a6` package (`a7` cannot be installed on any platform yet --
its native HAL wheel has no `a7` release at all). Headline findings: **none**
of the vendor libraries this robot depends on (REV, navX, PathPlanner,
PhotonVision, Phoenix 6) have a 2027 release, and neither does
`robotpy-commands-v2` (the `commands2` package this whole port's architecture
depends on). Beyond that, 2027 is a bigger jump than a version bump: the
RoboRIO is being replaced by new "SystemCore" hardware (with peripheral
support changes -- notably SPI/SPI-IMU removal, which is how this robot's
navX is wired), a large fraction of the Python API is being renamed
(`robotInit()` removed, `DriverStation` split into `MatchState`/`RobotState`,
`ChassisSpeeds` split into velocity/acceleration classes, and -- the
naming-convention specifics -- the changelog claims Python moves to
snake_case at alpha 6, which direct testing found **not yet true** in the
actual `a6` package, and all constants/enums move to `ALL_CAPS` at alpha 7,
unverifiable since `a7` won't install), and the command framework itself is
being replaced by a structurally different **Commands v3** (coroutine-based,
Java-only so far -- no Python binding exists for v2 or v3 in 2027). See that
branch's README for the full findings with sources.

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

## If your team transitions to Python

This section is specifically about what would help beyond just working
code, aimed at a programming subteam that's mostly rookies who will need to
be taught the whole stack -- FRC concepts and Python itself, at the same
time.

### Where Python actually helps rookies, and where it doesn't

The real advantage isn't "Python is easier than Java" in the abstract --
it's the **iteration loop**. `python -m robotpy sim` starts in a couple of
seconds; a Java Gradle build+deploy cycle is much slower. That difference
compounds hugely for someone still learning what a `Command` even is: they
can change one line, resimulate, and see the result almost immediately,
instead of waiting through a build every time. Lean into this deliberately
-- have new members learn command-based concepts *in the simulator*, on
this codebase or a stripped-down copy of it, before they ever touch the real
robot.

Where it doesn't help: Python's dynamic typing removes a safety net Java
gives you for free. A typo'd attribute name or a wrong argument type is a
compile error in Java; in Python it's a `RuntimeError` at whatever line
actually gets hit, potentially only in a rarely-exercised code path (an
error handler, a specific button combo). This codebase leans on type hints
(`def foo(self, x: float) -> bool:`) and the `tests/` suite specifically to
put some of that safety net back -- both are worth treating as required
practice for rookies, not optional polish, precisely because they don't
have Java's compiler catching mistakes for them.

### Suggested tooling

None of this is set up in this port yet -- worth adding before rookies start
contributing regularly, since it removes entire categories of code-review
comments that don't teach anything ("put a space after that comma") and
lets mentors spend review time on logic instead:

- **An auto-formatter** (e.g. `black` or `ruff format`) run automatically
  before commit. Removes style bikeshedding entirely -- nobody has an
  opinion on formatting because nobody hand-formats.
- **A linter** (`ruff` covers both formatting and linting in one tool) to
  catch unused imports, undefined names, and similar mistakes before they
  reach a mentor's review, let alone the robot.
- **A pre-commit hook** running both, so the feedback happens the moment
  someone tries to commit, not in a review comment an hour later.
- **`mypy` or `pyright`** (optional, but worth considering given the point
  above about losing Java's compile-time checking) to catch type mismatches
  before runtime, since this codebase already writes type hints everywhere.

### Suggested first tasks for rookies

Contained, low-risk-if-wrong, and each teaches something different about
the codebase:

- Implement `Climber` or `Intake` (both are empty stubs today) -- forces
  learning the SparkMax/TalonFX configuration pattern already established
  in `Feeder`/`Shooter` by reading and adapting it, with a mentor reviewing.
- Add a test to `tests/` for an existing subsystem method that doesn't have
  one yet -- learns the codebase by reading it closely enough to predict its
  behavior, without touching hardware or robot behavior at all.
- Set up the actual PathPlanner GUI project and author a first real
  autonomous path (see [Setup](#setup) -- there isn't one yet) -- learns
  the auto tooling without writing any Python.
- Add a docstring to a private helper method that doesn't have one --
  smallest possible contribution, still requires actually understanding
  what the method does.

### A few Python-for-Java-programmers gotchas worth teaching explicitly

- **Indentation is syntax**, not style -- a wrong indent changes what block
  of code a line belongs to instead of just looking wrong.
- **`self` is an explicit first parameter** on every method, not an implicit
  keyword (`self.get_pose()`, defined as `def get_pose(self):`).
- **Mutable default arguments are a classic bug**: `def f(items=[]):` reuses
  the *same* list across every call that doesn't pass one explicitly. This
  codebase avoids it (see `result.py`'s use of `field(default_factory=list)`
  in a `dataclass`) -- worth explaining why, not just doing it.
- **Reading a Python traceback**: the *last* line is usually the actual
  error and the most useful one to read first; the lines above it are the
  call stack that led there, read top-to-bottom in the order calls
  happened. This is the opposite instinct from skimming a huge Java stack
  trace for the first `Caused by:`.
- **`==` vs. `is`**: `==` compares values, `is` compares identity. Almost
  everything in this codebase should use `==`.

### General FRC safety, worth a reminder for a rookie-heavy roster regardless of language

Not Python-specific, but worth stating for a subteam that's mostly new:
never reach into a mechanism while it's powered, always know where the
E-stop is, and have a dedicated spotter whenever code is being tested on
the real robot for the first time (or after any change to something that
moves) rather than only the person driving.

### Worth writing down, not just doing

The [Naming and numbering conventions](#naming-and-numbering-conventions)
section above is reverse-engineered from this code, not a document anyone
on the team wrote. If the team commits to Python, turning that section (plus
the tooling choices above, once decided) into an actual `CONTRIBUTING.md` in
the repo means new members can look it up themselves instead of every
convention being tribal knowledge a mentor has to repeat to each rookie.
