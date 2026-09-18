# 2027 Alpha Tracking (NOT a working robot)

This branch exists only to track how close RobotPy's 2027 season support is
to being usable for Team 2408's actual robot -- differential drive on REV
SparkMax + navX, a CTRE Phoenix 6 shooter motor, dual PhotonVision cameras,
PathPlanner autonomous, all wired together with the `commands2` command-based
framework. It is deliberately **not** a continuation of the working port on
[`claude/frc-java-robotpy-port-i012ns`](../../tree/claude/frc-java-robotpy-port-i012ns) --
almost none of that code can run yet, for reasons below, so carrying it over
unmodified would just be broken code that happens to sit in a repo.

**Bottom line as of this check:** wait, and expect more than a version bump
when the time comes. This isn't just vendor libraries catching up -- the
2027 season replaces the RoboRIO with new "SystemCore" hardware, renames a
large fraction of the API surface, and is introducing a structurally
different command framework (Commands v3) rather than extending the one
this port uses. Re-run the checks below periodically; the moment
`robotpy-commands-v2` and `robotpy-rev` both publish a 2027.x release is the
real go/no-go signal for starting a real 2027 port, not the `robotpy` core
version number by itself.

**Sources for everything below:** the official
[WPILib 2027 yearly changelog](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html)
(quoted directly, not summarized from memory), cross-checked wherever
possible by installing the actual packages from PyPI and introspecting them
directly. Each claim below says which of those two it's based on.

## Naming conventions

This was checked specifically because it changes how *every* line of ported
code reads, not just which packages are pinned.

- **snake_case.** The changelog states: *"Alpha 6: Python functions and
  variables have been renamed to use snake_case instead of camelCase for
  consistency with Python naming conventions."* **This could not be
  confirmed in the actual installable `robotpy==2027.0.0a6` wheel** -- direct
  testing found `wpilib.SmartDashboard.putString` (camelCase) still present
  and callable, with no `put_string` snake_case alias, and this held for
  every other class checked (`Timer`, `TimedRobot`, `Pose2d`). Either the
  published `a6` wheel lags behind the change described in the changelog, or
  the changelog bullet refers to something narrower (pure-Python packages
  like `robotpy_ext`/`pyfrc`, not the compiled `wpilib`/`wpimath` bindings).
  Re-check this directly against whatever alpha is current when a 2027 port
  starts -- don't assume snake_case has landed just because a changelog
  entry mentions "Alpha 6."
- **ALL_CAPS constants and enum values.** The changelog states: *"Alpha 7:
  All constants (including enumerated values) have been changed to ALL_CAPS
  style."* This could **not** be verified directly -- `2027.0.0a7`'s native
  HAL wheel (`robotpy-native-wpihal`) has no 2027 release on PyPI at all yet
  (not a platform issue: every other `a7` component installed fine except
  this one), so `robotpy==2027.0.0a7` cannot currently be installed on any
  platform, not just the one this check ran on. Expect enums like
  `SparkLowLevel.MotorType.kBrushless` or
  `DriverStation.Alliance.kRed` (2026-style, `k`-prefixed) to become
  something like `MotorType.BRUSHLESS` / `Alliance.RED` once this lands --
  CTRE's Phoenix 6 Python bindings already use exactly this style today
  (`InvertedValue.CLOCKWISE_POSITIVE`, `NeutralModeValue.COAST`), so this is
  WPILib converging on a convention Phoenix 6 got to first, not a novel one.
- **Verified rewrites that aren't casing changes, but will still break every
  reference to them:** `testInit`/`testPeriodic` are gone --
  `wpilib.TimedRobot` at `a6` has `utilityInit`/`utilityPeriodic` instead
  (changelog: *"Rename 'Test' robot mode to 'Utility'"*, confirmed present by
  direct `hasattr` check). `Timer.getFPGATimestamp()` no longer exists at all
  (changelog: *"Rename FPGA clock to monotonic clock"*) -- `Timer` at `a6`
  has `getTimestamp()` and `getMonotonicTimestamp()` instead, confirmed
  directly.

## SystemCore: this is bigger than a version bump

The changelog opens with: *"The change from the roboRIO to Systemcore is the
biggest control system update since the introduction of the cRIO."* This
matters for this specific robot, not just in the abstract:

- CAN device classes now take a `CANPort` enum instead of a plain integer
  (*"to disambiguate Systemcore ports from Motioncore ports"*, Alpha 7) --
  every `CAN_ID` constant in `constants.py` (`LEFT_LEAD_CAN_ID = 20`, etc.)
  would need to change type, not just get renamed.
- SystemCore *removes* several RoboRIO peripheral categories outright,
  including **SPI and SPI IMUs** (the changelog names ADIS16448, ADIS16470,
  ADXL345, ADXRS450 specifically) and analog gyro. This robot's navX is
  wired over SPI (`navx.AHRS.NavXComType.kMXP_SPI` in
  `subsystems/drivetrain.py` on the 2026 port) -- whether navX support
  survives on SystemCore, and in what form, is an open question this
  changelog doesn't answer for that specific sensor family.
- Relay, analog output, DMA, built-in accelerometer, digital glitch filter,
  interrupts, counter, ultrasonic, analog trigger, Nidec Brushless, Servo,
  and Jaguar support are also removed. None of those are used by this robot
  today, so they're listed here only as evidence of how much of the
  RoboRIO-era hardware API doesn't carry forward at all, versus just being
  renamed.

## No vendor library this robot depends on has a 2027 release

Checked via `pip index versions <pkg>` for each -- latest version of every
one is still 2026.x:

| Package | Latest seen | Provides |
|---|---|---|
| `robotpy-commands-v2` | 2026.2.2 | `commands2` -- the Subsystem/Command framework this whole port's architecture depends on |
| `robotpy-rev` | 2026.0.4 | REV SparkMax (drivetrain + feeder motors) |
| `robotpy-navx` | 2026.0.1.1 | navX gyro |
| `robotpy-pathplannerlib` | 2026.1.2 | PathPlanner autonomous |
| `photonlibpy` | 2026.3.4 | PhotonVision AprilTag pose estimation |
| `phoenix6` | 26.3.0 | CTRE TalonFX (shooter motor) |
| `robotpy-apriltag` | 2026.2.2 | AprilTag field layouts |
| `robotpy-cscore` | 2026.2.2 | Camera streaming |

The changelog itself explains why for two of these: *"The WPILib AprilTag
and CameraServer libraries have been moved to vendordeps"* -- so
`robotpy-apriltag`/`robotpy-cscore` aren't just lagging, they're being
restructured to ship the same way REV/CTRE/PathPlanner already do, as
separate vendor packages outside core WPILib.

`robotpy-rev`'s own issue tracker confirms the rest isn't an oversight
either: [robotpy-rev#91 "snake_case upgrade"](https://github.com/robotpy/robotpy-rev/issues/91)
says a 2027 branch exists locally but is blocked on WPILib's alpha-7
stabilizing first, because of breaking changes. There's no equivalent public
tracking issue for the other libraries, but the same blocker likely applies
to all of them -- they all bind against WPILib's C++ core, which the
changelog shows is still moving significantly release to release.

## The command framework itself is being replaced, not upgraded

The changelog introduces **Commands v3** at Alpha 5 for Java: *"Add Commands
v3 framework... Design Document... port of the Hatchbot example to Commands
v3."* Later Commands v3 entries describe a coroutine-based design (*"Add
compile-time checks for unsafe or incorrect coroutine usage,"*
*"Coroutine.waitUntil overload with timeout,"* *"declarative state machine
API on top of commands v3"*) -- structurally different from `commands2`'s
Subsystem/Command/CommandScheduler model, not an incremental version bump.
The VS Code extension changelog adds *"Support Commandsv3 vendordep"* at
Alpha 5, which is Java/Gradle-ecosystem tooling. **There is no mention
anywhere in this changelog, or on PyPI, of a Python binding for either
Commands v2 or v3 for 2027** -- when Python command-based support does
arrive for 2027, there's no indication yet it will be a straightforward
"port commands2 forward"; it may target v3's different model instead.

## Other confirmed changes worth knowing about before porting

Quoted or paraphrased directly from the changelog, grouped by how much they'd
touch this specific codebase:

- **`robotInit()` is removed.** *"Alpha 5: Remove `robotInit()`. Use the
  `Robot()` constructor instead."* Every subsystem's construction in this
  port's `robot.py`/`robotcontainer.py` happens inside `robotInit()`.
- **`Rotation2d` angles are now wrapped.** *"Silent Breaking:
  `Rotation2d`'s `getRadians()`, `getDegrees()`, and `getRotations()`
  methods now return a wrapped angle... Those who don't want wrapping should
  use `double` or `Angle` instead of `Rotation2d`."* This port's
  `DriveTrain.get_heading()` and the `physics.py` yaw-accumulator both treat
  `Rotation2d`/heading degrees as continuous values today.
- **`ChassisSpeeds` split into `ChassisVelocities` and
  `ChassisAccelerations`.** *"Use immutable member functions in
  `ChassisSpeeds`"* and *"Add `ChassisAccelerations` and drivetrain
  accelerations classes."* Matches what direct testing against `a6` already
  showed (no `wpimath.ChassisSpeeds`, both replacement classes present).
- **`DriverStation` is split into `MatchState` and `RobotState`.**
  Confirmed directly: at `a6`, `wpilib.DriverStation` has only 3 methods
  left (`startDataLog` and two event-handle methods). `getAlliance()`,
  `getMatchTime()` etc. moved to `MatchState`; `isAutonomous()`,
  `isEnabled()`, `isUtility()` (renamed from `isTest()`) etc. moved to
  `RobotState`. Every subsystem in this port calls
  `wpilib.DriverStation.getAlliance()`/`.isTest()`/`.isAutonomous()`
  directly.
- **Motor control API changes.** *"Rename `MotorController` `set()` to
  `setThrottle()`"* and *"Remove `MotorController::StopMotor()`. Use
  `MotorController::Disable()` instead."* Every subsystem's `.set(...)`
  calls on SparkMax/TalonFX-style motor objects would need updating.
- **Gamepad classes are in flux even within the alpha cycle.** *"Alpha 5:
  Replace individual gamepad classes (e.g. `XboxController`, `PS4Controller`
  ...) with a single `Gamepad` class. Alpha 7 adds back support for
  individual gamepad classes."* `CommandXboxController`'s 2027 equivalent
  isn't settled yet even in Java/C++, let alone in RobotPy.
- **SmartDashboard is being fully removed, not just deprecated.** Beyond the
  Telemetry/Tunables replacement noted above, there's a separate top-level
  warning: *"SmartDashboard has been removed for 2027 due to its usage of
  Network Tables v3."* Every subsystem in this port calls
  `wpilib.SmartDashboard.put*` for telemetry.
- **Shuffleboard, PathWeaver, and RobotBuilder are all removed outright**
  (lack of a maintainer, NT3 usage, and lack of swerve support /
  maintenance burden, respectively) -- not relevant to this port directly
  since none of those are used, but worth knowing if anyone on the team
  uses them for dashboarding today.

## How to re-check

From a scratch virtualenv:

```
pip install robotpy  # see what the latest stable/alpha actually is now
pip index versions robotpy-commands-v2 robotpy-rev robotpy-navx \
    robotpy-pathplannerlib photonlibpy phoenix6 robotpy-apriltag robotpy-cscore
```

The moment `robotpy-commands-v2` and `robotpy-rev` (the two most
load-bearing for this robot) show a `2027.x` version, that's the real
signal to start a proper 2027 port. At that point, re-read the current
[WPILib yearly changelog](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html)
in full (a lot will have changed since this check -- alphas 2 through 7
alone added hundreds of entries), and re-verify every claim above directly
against whatever alpha is current the same way this check did, the same way
the 2026 port on
[`claude/frc-java-robotpy-port-i012ns`](../../tree/claude/frc-java-robotpy-port-i012ns)
verified its APIs (see that branch's README "Verification" section) --
don't assume a changelog bullet has actually shipped in the installable
package just because it's listed under a given alpha number.

## What's on this branch

Just enough to prove *something* runs under the 2027 alpha:

- `pyproject.toml` -- pinned to `robotpy==2027.0.0a7` (no vendor `requires`,
  no `commands2`/`apriltag` components -- none have a 2027 release; note
  `a7` currently cannot actually be installed on any platform because
  `robotpy-native-wpihal` has no `a7` release -- see above).
- `robot.py` -- a bare `wpilib.TimedRobot` subclass with no subsystems, no
  `commands2` (it has no 2027 release either). This was constructed and
  had `robotInit()` called successfully under a simulated HAL running
  `robotpy==2027.0.0a6`, the newest version actually installable.

Everything else from the 2026 port (`subsystems/`, `commands/`,
`autonomous/`, `tests/`, `physics.py`, `robotcontainer.py`, `constants.py`,
`vision_measurement.py`, `result.py`) was removed from this branch rather
than left in a broken, unimportable state -- they all depend on packages
that don't exist for 2027 yet, and several of the WPILib APIs they call
directly (`DriverStation.getAlliance()`/`.isTest()`, `SmartDashboard.put*`,
`robotInit()`, `ChassisSpeeds`) are confirmed gone or renamed in the alpha
already. They're intact on
[`claude/frc-java-robotpy-port-i012ns`](../../tree/claude/frc-java-robotpy-port-i012ns).
