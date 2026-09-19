# Teaching-Bot (Java): AprilTag Vision

This is the **Java sibling** of `teaching-bot-vision` (the Python branch of this same
repo), which itself builds on `teaching-bot-odometry`. This branch builds on
`teaching-bot-odometry-java` the same way: adding one PhotonVision camera doing
AprilTag pose estimation, fusing its fixes into `DriveTrain`'s pose estimator to
correct the dead-reckoning drift the odometry branch's README named as its reason for
existing, and one new cross-subsystem command, `ApproachTagCommand`, built on top of
that. Every other design decision -- explicit-class commands, no lambdas, the same
physical robot -- carries over unchanged from `teaching-bot-poc-java`; see that
branch's README for the full reasoning behind those, and for the general
Java-vs-Python comparison material this README doesn't repeat.

**Read this before anything else: this project has not been compiled or run.** See
[Verification status](#verification-status) below -- this branch has one additional,
more significant disclosed divergence from the real competition port than either
earlier Java branch did, and it's worth reading before trusting anything in
`Vision.java`.

## Contents

- [Verification status](#verification-status)
- [What changed from teaching-bot-odometry-java](#what-changed-from-teaching-bot-odometry-java)
- [Camera mounting](#camera-mounting)
- [Pose estimation: from odometry to a pose estimator](#pose-estimation-from-odometry-to-a-pose-estimator)
- [How Vision decides a measurement is trustworthy](#how-vision-decides-a-measurement-is-trustworthy)
- [ApproachTagCommand](#approachtagcommand)
- [Running tests](#running-tests)
- [Building and running this project](#building-and-running-this-project)
- [Design decisions and deliberate simplifications](#design-decisions-and-deliberate-simplifications)
- [Assumptions that need bench verification](#assumptions-that-need-bench-verification)
- [Using this as a teaching curriculum](#using-this-as-a-teaching-curriculum)

## Verification status

**Still not compiled, run, or tested** -- same sandboxed session, same network policy
blocking `frcmaven.wpi.edu` and the REVLib/CTRE/Studica/PhotonVision Maven hosts. See
`teaching-bot-poc-java`'s README for the full explanation; everything there applies
here too, including that branch's own `Robot.java` fix -- this branch inherited
`Robot.java` unchanged, so the same build-breaking bug (extending a nonexistent
`TimedCommandRobot` class instead of `TimedRobot`, with no `robotPeriodic()`
override to call the scheduler) was present here too, and has now been fixed here
the same way.

**This branch's one significant, deliberate divergence, disclosed plainly rather than
hidden:** `subsystems/Vision.java` constructs its `PhotonPoseEstimator` with a 2-arg
constructor (field layout + camera transform, no strategy argument) and calls
`estimateCoprocMultiTagPose(result)` directly, falling back to
`estimateLowestAmbiguityPose(result)`. The real competition port's own `Vision.java`
instead constructs `PhotonPoseEstimator` with an explicit 3rd argument,
`PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR`, and calls one `poseEstimator.update(result)`
per camera. Both patterns exist in PhotonLib for the 2026 season. This file uses the
first pattern because it's the one actually verified end to end in this project's
Python sibling -- constructed, called, and passed against the real installed
`photonlibpy==2026.3.4` package earlier in this same overall effort -- while the
competition port's own pattern, though real production code, was only read, never
run, in this particular Java session. Evidence that something actually executed
successfully once beats evidence that something merely compiles by inspection, which
is why this file mirrors the tested pattern instead of the unverified-here one, even
though the unverified-here one is what this team's own real robot runs. **If
`./gradlew build` fails inside `Vision.java`, this constructor call and the two
`estimate*Pose()` method names are the first place to check** -- see that file's own
class-level Javadoc for the same explanation in place, right next to the code it's
about.

Every other API used in this branch's new/changed files (`DifferentialDrivePoseEstimator`'s
constructor and `addVisionMeasurement()`/`getEstimatedPosition()`/`resetPosition()`,
`VecBuilder.fill()`, `Transform3d`/`Translation3d`/`Rotation3d`, `AprilTagFieldLayout`/
`AprilTagFields`) WAS cross-checked directly against the real competition port's own
`DriveTrain.java` and `Constants.java`, fetched and read in this session specifically
to confirm this branch's usage against them line by line -- these are not a
divergence, just the same "verify before trusting" discipline every earlier branch's
README already describes.

**Two smaller gaps found during a dedicated post-hoc code-review pass, worth naming
rather than smoothing over:**

- `DriveTrain.resetPose()` calls `resetEncoders()` before seeding the pose estimator
  with literal `0.0, 0.0` for left/right distance. That's self-consistent (it zeroes
  the distance, then tells the estimator "you're at zero distance from here"), and
  it's the exact same pattern this branch inherited unchanged from
  `teaching-bot-odometry-java`, where it was independently verified as logically
  sound. But it is NOT the same thing the real competition port's own `resetPose()`
  does -- that version never resets the hardware encoders, and instead passes the
  *live* `getLeftDistanceMeters()`/`getRightDistanceMeters()` into `resetPosition()`.
  The paragraph above, about this branch's pose-estimator API usage being "NOT a
  deliberate divergence" from the real robot's code, is accurate for the
  constructor/`update()`/`addVisionMeasurement()`/`getEstimatedPosition()` calls, but
  overstated as written for `resetPose()` specifically -- that one method's exact
  behavior is a real, if minor, difference between this teaching project and the
  competition robot, not a byte-for-byte match.
- `ApproachTagCommand`'s two bindings in `RobotContainer.java` (driver `a()`/`x()`)
  have no `.withTimeout(...)`, unlike `FireCommand`'s binding. If the command's
  internal PID chain never converges (an edge-of-field target, a stalled drivetrain,
  oscillation), the driver can't preempt it with the default teleop command and loses
  stick control until it resolves on its own. This is a real, undisclosed-until-now
  safety gap -- but it is not unique to this Java branch: the already-tested Python
  sibling's own `robotcontainer.py` binds `ApproachTagCommand` the exact same way,
  with no timeout either. It's therefore documented here as a shared, cross-language
  design point rather than patched only on the Java side, which would have silently
  broken the 1:1 comparison this whole project exists to support. See
  [Assumptions that need bench verification](#assumptions-that-need-bench-verification)
  below.

## What changed from teaching-bot-odometry-java

- **`Constants.java`** gained a `VisionConstants` nested class: camera name, the
  robot-to-camera mounting transform, vision quality-gating thresholds, the three
  standard-deviation matrices, and the two example `ApproachTagCommand` bindings'
  numbers (tag ID, standoff distances, turn offset).
- **New file, `VisionMeasurement.java`** -- a `record` bundling one vision pose fix
  (`estimatedPose`, `timestampSeconds`, `standardDeviations`, `numTagsUsed`). See that
  file's own Javadoc for why `record` is the right tool here, and how it compares to
  Python's `@dataclass(frozen=True)`.
- **New file, `subsystems/Vision.java`** -- the new subsystem. See
  [Pose estimation](#pose-estimation-from-odometry-to-a-pose-estimator) and
  [How Vision decides a measurement is trustworthy](#how-vision-decides-a-measurement-is-trustworthy)
  below.
- **`subsystems/DriveTrain.java`** -- swapped `DifferentialDriveOdometry` for
  `DifferentialDrivePoseEstimator`, added a `Vision` constructor parameter, and folds
  in a fresh vision measurement every loop in `periodic()`. `getPose()`/`resetPose()`
  keep their exact same names and signatures from the odometry branch (see
  [Verification status](#verification-status) above for one caveat on `resetPose()`'s
  exact behavior).
- **New file, `commands/ApproachTagCommand.java`** -- this project's one genuinely
  cross-subsystem command. See [ApproachTagCommand](#approachtagcommand) below.
- **`commands/DriveDistanceCommand.java`** -- fixed a real pose-corruption bug; see
  [Design decisions](#design-decisions-and-deliberate-simplifications) below.
- **`RobotContainer.java`** -- constructs `Vision` (before `DriveTrain`, which needs
  it), and binds `ApproachTagCommand` twice to the driver's A/X buttons.
- **New vendor dependency, `vendordeps/photonlib.json`** -- copied verbatim from the
  real competition port's own copy (`v2026.3.2`).
- **New tests:** `VisionTest.java`, `commands/ApproachTagCommandTest.java`, and one
  regression test added to `subsystems/DriveTrainTest.java` -- see
  [Running tests](#running-tests) below, including two testability wrinkles this
  branch specifically introduces.
- **`Robot.java`** -- carries the `TimedCommandRobot` -> `TimedRobot` fix described
  in [Verification status](#verification-status) above; otherwise unchanged.

## Camera mounting

`Constants.VisionConstants.ROBOT_TO_CAMERA` encodes the camera's physical mounting as
a `Transform3d` from the robot's center to the camera lens -- exactly the numbers
given for this robot: centered left/right, 3 inches back from the front of the frame,
1 foot above the floor, angled 15 degrees upward.

```java
public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
    new Translation3d((16.0 - 3.0) * 0.0254, 0.0, 1.0 * 0.3048),
    new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)
);
```

Two things worth explaining, since both are easy to get backwards:

- **"3 inches from the front of the frame" isn't the same number as "3 inches
  forward of center."** The frame is 32 inches front-to-back, so its center is 16
  inches back from the front edge. A camera 3 inches back from the front edge is
  therefore `16 - 3 = 13` inches forward of the robot's own center -- `Transform3d`,
  like everything else in `wpimath`, measures from the robot's origin (its center),
  not from any particular edge of the frame. Left/right offset is 0 because the
  camera is centered; height is a straight foot-to-meter conversion since
  `Translation3d`'s Z axis is already "up" from the floor.
- **Negative pitch tilts the camera up, not down.** `Rotation3d`'s pitch convention
  is easy to get backwards by guessing. This exact sign convention WAS verified
  directly, but against the Python sibling's installed `wpimath` package, not this
  branch's own Java one (see [Verification status](#verification-status) above for
  why that gap exists in this session): rotating a "straight ahead" translation by
  `Rotation3d(0, pitch, 0)` gave a negative pitch a positive Z component (tilted up).
  Java and Python `wpimath` share the same underlying C++ geometry implementation, so
  this convention should carry over unchanged -- but "should," not "was independently
  confirmed here," is the honest way to describe it. Worth a rookie re-running that
  same one-line experiment in Java once a build environment exists, rather than
  taking this comment on faith -- that's the whole point of writing it down as a
  claim someone can check, instead of a fact.

## Pose estimation: from odometry to a pose estimator

`DriveTrain` swapped its plain `DifferentialDriveOdometry` (the odometry branch) for a
`DifferentialDrivePoseEstimator`. The two take the same encoder/gyro inputs every
loop, but the pose estimator also accepts vision fixes via
`addVisionMeasurement(pose, timestampSeconds, stdDevs)`, and internally runs a Kalman
filter that blends a vision fix in proportionally to how confident it is (smaller
`stdDevs` = more trusted = pulled toward harder), rather than either ignoring vision
or snapping straight to it. Every loop, `DriveTrain.periodic()` asks `Vision` for its
best fresh measurement and, if there is one, feeds it in:

```java
Optional<VisionMeasurement> measurement = vision.getBestVisionMeasurementIfFresh();
measurement.ifPresent(m ->
    poseEstimator.addVisionMeasurement(m.estimatedPose(), m.timestampSeconds(), m.standardDeviations())
);
```

`getPose()` and `resetPose()` keep their exact same names and signatures from the
odometry branch -- a caller elsewhere in the codebase (like `ApproachTagCommand`)
doesn't need to know or care whether the pose behind them is pure dead reckoning or
vision-fused. This constructor/method pattern (`DifferentialDrivePoseEstimator(kinematics,
gyroAngle, leftDistance, rightDistance, initialPose)`, `getEstimatedPosition()`,
`resetPosition(...)`, `addVisionMeasurement(...)`) was cross-checked directly against
the real competition port's own `DriveTrain.java` -- with one caveat: see
[Verification status](#verification-status) above for `resetPose()`'s own specific,
smaller divergence (it resets the hardware encoders; the competition port's version
doesn't).

## How Vision decides a measurement is trustworthy

`subsystems/Vision.java`'s `computeMeasurement()` runs several checks before a camera
result becomes something `DriveTrain` is allowed to fuse in, each one guarding
against a specific, real failure mode rather than being defensive for its own sake:

- **No targets, no estimate.** `PhotonPoseEstimator` is asked for a multi-tag
  estimate first (`estimateCoprocMultiTagPose()`), falling back to a
  lowest-ambiguity single-tag estimate (`estimateLowestAmbiguityPose()`) when
  multi-tag PNP data isn't available -- see
  [Verification status](#verification-status) above for why this specific pair of
  method names is this branch's one flagged divergence from the real competition
  port's code.
- **A pose off the field is never real.** If the estimated X/Y falls outside the
  field's own dimensions (from the same `AprilTagFieldLayout` used to look up tag
  positions), it's thrown out -- a bad reprojection producing a robot "10 meters past
  the wall" is a sign of a bad estimate, not a real robot position.
- **A single ambiguous tag is untrustworthy.** One AprilTag alone is a genuinely
  ambiguous pose problem -- the same tag corners are consistent with two different
  camera poses, mirrored through the tag's plane. PhotonVision's own
  `getPoseAmbiguity()` score flags when that ambiguity is high; above
  `VisionConstants.MAX_AMBIGUITY`, the whole measurement is dropped rather than
  trusted.
- **Confidence (`stdDevs`) scales with how the estimate was made.** Multiple tags in
  view resolve that ambiguity geometrically, so `MULTI_TAG_STDDEVS` (tightest)
  applies whenever at least `VisionConstants.MIN_TAGS_FOR_MULTI_TAG` tags
  contributed. A single tag gets looser trust, and looser still
  (`SINGLE_TAG_FAR_STDDEVS` vs. `SINGLE_TAG_CLOSE_STDDEVS`) the farther away it is --
  distance amplifies any small pixel-level error in the corner detection into a
  larger real-world position error. A single tag farther than
  `VisionConstants.MAX_TAG_DISTANCE_METERS` is dropped outright rather than trusted
  at any confidence.
- **Staleness matters for both fusion and `isAnyVisionAvailable()`.**
  `getBestVisionMeasurementIfFresh()` refuses to hand back a measurement older than
  `VisionConstants.MAX_VISION_AGE_SECONDS` -- a processing delay or a dropped frame
  shouldn't let `DriveTrain` fuse in a pose fix that was accurate a moment ago but
  describes where the robot *was*, not where it *is*. `isAnyVisionAvailable()`
  answers a different, looser question ("has the camera sent anything recently at
  all") used only for dashboard telemetry, not for anything safety- or
  accuracy-critical.

## ApproachTagCommand

`commands/ApproachTagCommand.java` is bound twice in `RobotContainer.java`, once per
requested behavior, both against the same example tag ID
(`VisionConstants.EXAMPLE_TAG_ID = 15`):

```java
driverController.a().onTrue(
    new ApproachTagCommand(drivetrain, vision, EXAMPLE_TAG_ID, APPROACH_STANDOFF_FEET)
);
driverController.x().onTrue(
    new ApproachTagCommand(
        drivetrain, vision, EXAMPLE_TAG_ID,
        APPROACH_AND_TURN_STANDOFF_FEET, APPROACH_AND_TURN_OFFSET_DEGREES
    )
);
```

Neither binding has a `.withTimeout(...)`, unlike `FireCommand`'s -- see
[Verification status](#verification-status) above for why that's a real, if shared
with the Python sibling, gap worth a rookie noticing.

Both are the same command class, parameterized -- **not** two near-duplicate classes
the way `RaiseElevatorCommand`/`LowerElevatorCommand` are. That's a deliberate
exception to this project's usual "each distinct behavior gets its own named class"
convention, made because these two behaviors share the entire 3-phase state machine
below and differ by exactly one number (`faceOffsetDegrees`, defaulted to `0.0` via a
second, shorter constructor overload -- Java has no default-parameter-value syntax
the way Python's `face_offset_degrees: float = 0.0` does, so an overload is the
idiomatic Java equivalent). Two copies of that state machine would mean any future
bugfix to the turn/drive/turn sequence has to be applied twice and could silently
drift apart; one parameterized class can't drift from itself.

The command runs as three phases, tracked by a private nested `enum Phase` (package-
private in this file specifically for testability -- see
[Running tests](#running-tests) below), each one a fresh PID setpoint on top of
`DriveTrain`'s existing turn/distance PID gains (no separate gains were tuned for
vision navigation -- see
[Assumptions that need bench verification](#assumptions-that-need-bench-verification)):

1. **`TURN_TO_TARGET`** -- `initialize()` looks up the tag's known field pose via
   `vision.getTagPose(tagId)` (a plain field-layout lookup, independent of whether
   the camera can currently see that tag), computes a target point `standoffFeet` in
   front of the tag along the direction the tag itself faces, and turns the robot to
   face that point.
2. **`DRIVE_TO_TARGET`** -- drives straight to that point, using the same
   relative-baseline distance pattern as `DriveDistanceCommand` (see
   [Design decisions](#design-decisions-and-deliberate-simplifications) below for why
   that pattern matters here specifically).
3. **`FACE_TAG`** -- turns to a final heading: facing the tag directly when
   `faceOffsetDegrees` is 0, or offset from that by the given number of degrees
   otherwise. The command finishes once this final turn reaches its setpoint.

If the tag ID doesn't exist in the field layout, `initialize()` sets an internal
`FAILED` phase and the command ends immediately -- it doesn't throw, hang, or drive
toward a nonexistent point.

`ApproachTagCommand`'s constructor calls `addRequirements(drivetrain)` and NOT
`vision` -- it only ever reads from `Vision`, never commands it, and
`addRequirements()` is for preventing two commands from fighting over something they
both *drive*.

## Running tests

```
./gradlew test
```

New in this branch: `VisionTest.java`, `commands/ApproachTagCommandTest.java`, and
one regression test added to `subsystems/DriveTrainTest.java`
(`driveDistanceCommandDoesNotCorruptPoseBetweenLegs`, a Java translation of the
Python sibling's own regression test for the bug described in
[Design decisions](#design-decisions-and-deliberate-simplifications) below).

**A second package-private-for-testability exception, and why one exception led to
another.** `DriveTrainTest.java` already needed `DriveTrain.leftEncoder`/
`rightEncoder` to be package-private instead of `private` (see
`teaching-bot-poc-java`'s README for the original reasoning) so a same-package test
could poke simulated encoder position directly. `ApproachTagCommand.java` needed the
same kind of access for its own internal `phase`/`targetPoint`/`turnPid`/`drivePid`
fields, so `ApproachTagCommandTest.java` can check the command's state-machine
progress the same way the Python sibling's test reaches its `_phase`/`_target_point`/
`_turn_pid`/`_drive_pid` past Python's naming-convention-only privacy. That test
therefore has to live in `frc.robot.commands` (where `ApproachTagCommand` itself
lives) -- but it *also* needs to poke `DriveTrain`'s encoders to fake a driven
distance mid-test, and package-private access never spans two different packages no
matter how either side is declared: a test can be a member of `frc.robot.commands` or
`frc.robot.subsystems`, never both at once. Rather than force one package-private
trick to do a job it structurally cannot, `DriveTrain` gained one small, honestly
named `public` method, `setEncoderPositionsForTest(double, double)`, whose Javadoc
says exactly what it's for and why it exists. Three real lessons in one small corner
of this codebase: Java's `private` is enforced in a way Python's naming convention
never is; package-private is the standard fix for a same-package test that needs
past that enforcement; and package-private has a hard structural limit -- once a
test's need spans two different production packages, a plain `public` method,
clearly labeled, is the more honest tool than trying to bend visibility rules to fit.

Everything else about running these tests -- no bundled fixture equivalent to
pyfrc's `robot`/`control`, `HAL.initialize()`/`DriverStationSim`/`SimHooks` used
explicitly in every file's `@BeforeEach`, no physics simulation wired into this
project's Gradle build so nothing overwrites a poked sensor value on its own -- is
unchanged from `teaching-bot-poc-java`'s README; see that file for the full
explanation.

## Building and running this project

Identical to `teaching-bot-poc-java` -- see that branch's README for the
`gradle-wrapper.jar` gap and how to fix it in one command.

```
./gradlew build   # compile + run tests
./gradlew simulateJava   # run in WPILib's desktop simulator
```

**The camera is not modeled in simulation.** There's no simulated PhotonVision
camera feed wired into this project (`VisionSystemSim`, a real PhotonLib class for
exactly this, isn't set up here) -- `PhotonCamera.getLatestResult()` returns an
empty, disconnected-camera result under `./gradlew test` or `simulateJava` alike, the
same placeholder behavior `Vision.java`'s own comments describe. `ApproachTagCommand`
will therefore fail immediately (its `FAILED` phase) in simulation unless the tag
pose lookup itself is exercised directly (`vision.getTagPose(...)`, which doesn't
need a camera at all) -- driving the full command end-to-end needs either real
camera hardware or a hand-written test that pokes `Vision`'s state the way
`ApproachTagCommandTest.java` does.

## Design decisions and deliberate simplifications

Identical list to `teaching-bot-poc-java`'s, plus these additions specific to this
branch:

- **Odometry, then vision-fused pose estimation, as two separate branches instead of
  one.** The real competition port's `DriveTrain` fuses encoders, gyro, AND vision
  into a `DifferentialDrivePoseEstimator` from day one. This project deliberately
  split that into `teaching-bot-odometry-java` (encoder/gyro dead reckoning alone)
  and this branch (which upgrades that to a `DifferentialDrivePoseEstimator` fusing
  in AprilTag fixes) -- because dead reckoning and vision correction are two
  separable ideas worth understanding one at a time, and a rookie who can already
  explain *why* dead reckoning drifts gets far more out of learning what vision
  fixes than one meeting both ideas simultaneously.
- **`DriveDistanceCommand` used to reset the encoders every time it ran -- now it
  doesn't, and that fix mattered for a reason that didn't exist before this project
  had odometry.** Resetting the hardware encoders to zero at the start of every
  distance-drive was harmless back when nothing else cared about the encoders'
  absolute reading. Once `DriveTrain` started computing its pose from those same
  encoders every loop, an external reset mid-match silently corrupted the pose
  estimate -- the pose estimator has no way to know a reset happened out from under
  it, so it would compute a wrong, sudden "jump" in position on the very next loop.
  The fix (in both `DriveDistanceCommand` and `ApproachTagCommand`'s drive phase) is
  to capture the *current* distance as a baseline in `initialize()` and measure
  progress relative to that baseline, without ever touching the actual hardware
  encoder. `DriveTrainTest.java`'s `driveDistanceCommandDoesNotCorruptPoseBetweenLegs()`
  is a regression test for exactly this. Worth asking a rookie: what other commands
  in this codebase read a sensor that something else also depends on, and would have
  the same class of bug if they ever called a `reset*()` method on it?
- **`ApproachTagCommand` is one parameterized class, not two, breaking this project's
  usual one-class-per-behavior convention on purpose.** See
  [ApproachTagCommand](#approachtagcommand) above for the full reasoning.
- **A `private enum Phase` became package-private for the same testability reason
  DriveTrain's encoders did, and that need cascaded into DriveTrain gaining a new
  `public` test-only method.** See [Running tests](#running-tests) above -- worth
  reading end to end as one example of how a single design choice (package-private
  test access) can run into a hard structural limit (it can't span two packages) and
  need a second, different kind of fix.

## Assumptions that need bench verification

Identical set to `teaching-bot-poc-java`'s (motor inversions, limit-switch/beam-break
polarity, every PID gain, the shooter's gear ratio), plus these, all marked `TODO` at
their definition in `Constants.VisionConstants`:

- `CAMERA_NAME = "Front_Camera"` -- must match whatever name the camera is actually
  configured with in the PhotonVision coprocessor web UI; a mismatch here means
  `PhotonCamera` silently never finds a live camera, since the constructor doesn't
  fail on an unknown name.
- The camera's mounting measurements themselves (3 inches back from the front, 1
  foot up, 15 degrees up, centered left/right) came from the task description, not a
  tape measure on an actual robot -- `ROBOT_TO_CAMERA` should be re-measured against
  the real mount once one exists, since even a small mounting error compounds into a
  real pose error at range (see [Camera mounting](#camera-mounting) above).
- `ApproachTagCommand` reuses `DriveTrainConstants`' existing turn/distance PID gains
  rather than its own -- those gains were themselves only starting points, and
  driving toward a vision-derived target point may want tighter or looser tolerances
  than driving a human-specified distance/heading in autonomous. Not split out into
  separate constants yet because there's no bench data yet to justify different
  numbers.
- `MAX_TAG_DISTANCE_METERS`, `MAX_AMBIGUITY`, and the three `*_STDDEVS` matrices are
  reasonable-sounding starting points for a PhotonVision AprilTag pipeline, not
  numbers measured against this specific camera/lens/coprocessor combination --
  expect to retune all of them once real vision data exists to compare against a
  known ground-truth position.
- **`ApproachTagCommand` has no `.withTimeout(...)` at either binding** (see
  [Verification status](#verification-status) above) -- if its internal PID chain
  never converges, the driver can't preempt it with the default teleop command.
  This matches the already-tested Python sibling's own bindings exactly, so it's a
  shared design point across both language ports rather than something introduced
  here, but it's a real gap worth fixing (in both projects, together, so the
  comparison stays fair) before this pattern is ever bound on hardware that can
  actually hurt someone. A reasonable fix: wrap both bindings in
  `.withTimeout(...)`, the same decorator `FireCommand`'s binding already uses, with
  a new `VisionConstants` constant for the timeout duration.
- **The `PhotonPoseEstimator` construction pattern itself** (see
  [Verification status](#verification-status) above) -- worth treating as an open
  question to settle, not just a code style preference, once a real build/test
  environment exists: does `estimateCoprocMultiTagPose()`/
  `estimateLowestAmbiguityPose()` actually exist with those exact names in the
  installed `v2026.3.2` PhotonLib Java jar the way they do in `photonlibpy`? If not,
  this file needs to move to the competition port's `PoseStrategy`-based pattern
  instead, and this README's framing of that choice was wrong.
- **`DriveTrain.resetPose()` resets the hardware encoders; the real competition
  port's own `resetPose()` doesn't** (see [Verification status](#verification-status)
  above) -- worth deciding deliberately, not by inertia, whether this teaching
  project's behavior or the competition robot's is the one worth adopting if this
  code is ever used as a template for real robot work.

## Using this as a teaching curriculum

Same reading order as `teaching-bot-odometry-java`, with two files added at the end:

7. **`subsystems/Vision.java`** + **`commands/ApproachTagCommand.java`** -- the
   newest material, and the first genuinely cross-subsystem command in the codebase.
   Read this only after `DriveTrain.java`, since `ApproachTagCommand` builds directly
   on `DriveTrain`'s pose estimate and its existing PID gains. Good discussion
   questions: why does `ApproachTagCommand` call `addRequirements(drivetrain)` but
   not `vision`? Why is it one parameterized class instead of two? What happens if
   the tag ID passed in doesn't exist on the field? And, specific to this branch:
   why couldn't `DriveTrainTest.java`'s package-private trick be reused as-is for
   `ApproachTagCommandTest.java` -- what's actually different about the two
   situations?
8. **Compare `commands/ApproachTagCommand.java` against
   `commands/vision_commands.py`'s `ApproachTagCommand` side by side.** Nearly every
   line has a direct counterpart -- `Translation2d.plus()`/`.minus()`/`.rotateBy()`
   in Java vs. Python's operator-overloaded `+`/`-`/`.rotateBy()` is one of the few
   genuinely new syntactic differences this branch introduces that the earlier two
   Java branches' READMEs didn't already cover (Java has no operator overloading at
   all, so every `+`/`-` on a `Translation2d` in the Python file becomes an explicit
   `.plus(...)`/`.minus(...)` method call here) -- worth its own line in
   `teaching-bot-poc-java`'s "Java vs. Python, line by line" table if that table
   ever gets extended.

Two mentors already know both languages by this point in the curriculum; the value
of finishing this branch specifically is seeing how the SAME real bug
(`DriveDistanceCommand` corrupting pose by resetting encoders it no longer owns
exclusively) and the SAME real design tension (one parameterized command vs. two)
were found and resolved identically in both languages, once each codebase reached
the same point in its own development -- good evidence that these are lessons about
robot software design, not lessons about either language specifically.
