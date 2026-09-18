# 2027 Alpha Tracking (NOT a working robot)

This branch exists only to track how close RobotPy's 2027 season support is
to being usable for Team 2408's actual robot -- differential drive on REV
SparkMax + navX, a CTRE Phoenix 6 shooter motor, dual PhotonVision cameras,
PathPlanner autonomous, all wired together with the `commands2` command-based
framework. It is deliberately **not** a continuation of the working port on
[`claude/frc-java-robotpy-port-i012ns`](../../tree/claude/frc-java-robotpy-port-i012ns) --
almost none of that code can run yet, for reasons below, so carrying it over
unmodified would just be broken code that happens to sit in a repo.

**Bottom line as of this check:** wait. Re-run the check in "How to re-check"
below every so often; the moment `robotpy-commands-v2` and `robotpy-rev` both
publish a 2027.x release is the real go/no-go signal for starting a real
2027 port, not the `robotpy` core version number by itself.

## What was actually checked

Everything below was checked by installing the real packages from PyPI into
a scratch virtualenv and introspecting them directly -- not inferred from
changelogs or assumed to follow 2026's API.

### Core `robotpy` package: alpha releases exist

`pip index versions robotpy` lists (newest first as of this check):
`2027.0.0a7`, `2027.0.0a6.post1`, `2027.0.0a6`, `2027.0.0a5.post1`,
`2027.0.0a5`, `2027.0.0a4`, `2027.0.0a2`. `2027.0.0a7`'s native wheels
(`robotpy-native-wpihal` etc.) weren't published yet for the platform this
check ran on (linux x86_64) -- `pip install robotpy==2027.0.0a7` failed for
that reason alone. `2027.0.0a6` installed and ran cleanly, so everything
below was actually verified against **a6**, not a7. `pyproject.toml` on this
branch still pins a7 (the newest) per the request that led to this branch;
if `robotpy sync` can't find a7 wheels for your platform, drop back to a6.

### None of the vendor libraries this robot needs have a 2027 release

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
| `robotpy-apriltag` | 2026.2.2 | AprilTag field layouts (used even without PhotonVision, for `Vision.py`'s field bounds check) |
| `robotpy-cscore` | 2026.2.2 | Camera streaming |

`robotpy-rev`'s own issue tracker confirms this isn't an oversight:
[robotpy-rev#91 "snake_case upgrade"](https://github.com/robotpy/robotpy-rev/issues/91)
says a 2027 branch exists locally but is blocked on WPILib's alpha-7
stabilizing first, because of breaking changes. There's no equivalent
public tracking issue for the other libraries, but the same blocker likely
applies to all of them -- they all bind against WPILib's C++ core, which is
what's still moving.

### WPILib's own API is being restructured, not just bumped

Confirmed directly against the installed `2027.0.0a6` packages:

- **`wpimath` lost its submodules.** In 2026, geometry/kinematics/estimator/
  controller classes live in `wpimath.geometry`, `wpimath.kinematics`, etc.
  In 2027 alpha, they're all flattened directly onto `wpimath` --
  `wpimath.Pose2d` instead of `wpimath.geometry.Pose2d`, and there is no
  `wpimath.geometry` module at all anymore.
- **Renamed classes.** `ChassisSpeeds` is now split into `ChassisVelocities`
  and `ChassisAccelerations`; `DifferentialDriveWheelSpeeds` is now
  `DifferentialDriveWheelVelocities`.
- **`wpilib.drive` is gone.** No `DifferentialDrive`, `MecanumDrive`, or
  `RobotDriveBase` -- the helper classes `DriveTrain.py` uses for
  `tankDrive()` don't exist in this alpha at all.
- **Smaller removals already visible:** e.g.
  `DriverStation.silenceJoystickConnectionWarning()` no longer exists.
- **Not yet reproduced here, but reported upstream:** the WPILib yearly
  changelog for 2027 states `SmartDashboard`, `SendableChooser`, and
  `Sendable` are being replaced by new `Telemetry` and `Tunables` APIs.
  `SmartDashboard`/`SendableChooser` still exist as of `a6` (this was
  checked directly), so that change likely lands in `a7` or later --
  unverified here since `a7`'s native wheels weren't installable in this
  environment.
- **Java-side context:** a real team's 2027 alpha repo
  ([Drew-Robotics/2027beta](https://github.com/Drew-Robotics/2027beta),
  Team 8852, Java) describes their base as "WPILib 2027 alpha, Java 25, and
  Commands v3" -- i.e. the *Java* command framework is also being replaced,
  not just extended. Whatever RobotPy eventually ships for 2027 may not be
  a simple "commands2, but for 2027" upgrade.

None of this is a criticism of RobotPy or WPILib -- alpha software moving
between alphas is exactly what "currently alpha testing" means. It's just
why this branch stops at "here's what's blocked," not "here's a 2027 port."

## How to re-check

From a scratch virtualenv:

```
pip install robotpy  # see what the latest stable/alpha actually is now
pip index versions robotpy-commands-v2 robotpy-rev robotpy-navx \
    robotpy-pathplannerlib photonlibpy phoenix6 robotpy-apriltag robotpy-cscore
```

The moment `robotpy-commands-v2` and `robotpy-rev` (the two most
load-bearing for this robot) show a `2027.x` version, that's the real
signal to start a proper 2027 port -- at that point it's worth re-reading
this file's WPILib-restructuring notes above, since those affect every
subsystem file, and re-verifying each vendor API against the installed
packages the same way the 2026 port on
[`claude/frc-java-robotpy-port-i012ns`](../../tree/claude/frc-java-robotpy-port-i012ns)
was (see that branch's README "Verification" section).

## What's on this branch

Just enough to prove *something* runs under the 2027 alpha:

- `pyproject.toml` -- pinned to `robotpy==2027.0.0a7` (no vendor `requires`,
  no `commands2`/`apriltag` components -- none have a 2027 release).
- `robot.py` -- a bare `wpilib.TimedRobot` subclass with no subsystems, no
  `commands2` (it has no 2027 release either). This was constructed and
  had `robotInit()` called successfully under a simulated HAL running
  `robotpy==2027.0.0a6`.

Everything else from the 2026 port (`subsystems/`, `commands/`,
`autonomous/`, `tests/`, `physics.py`, `robotcontainer.py`, `constants.py`,
`vision_measurement.py`, `result.py`) was removed from this branch rather
than left in a broken, unimportable state -- they all depend on packages
that don't exist for 2027 yet. They're intact on
[`claude/frc-java-robotpy-port-i012ns`](../../tree/claude/frc-java-robotpy-port-i012ns).
