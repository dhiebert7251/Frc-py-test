# Teaching-Bot (Java): Odometry and Kinematics

This is the **Java sibling** of `teaching-bot-odometry` (the Python branch of this
same repo), which itself builds on `teaching-bot-poc`. This branch builds on
`teaching-bot-poc-java` the same way: adding `DifferentialDriveKinematics` and
`DifferentialDriveOdometry` to `DriveTrain`, plus a `Field2d` widget, so the robot
tracks its estimated (X, Y, heading) position on the field from encoders and the
gyro alone. Every other design decision -- explicit-class commands, no lambdas, the
same physical robot -- carries over unchanged from `teaching-bot-poc-java`; see that
branch's README for the full reasoning behind those, and for the general
Java-vs-Python comparison material this README doesn't repeat.

**Read this before anything else: this project has not been compiled or run.** See
[Verification status](#verification-status) below.

## Contents

- [Verification status](#verification-status)
- [What changed from teaching-bot-poc-java](#what-changed-from-teaching-bot-poc-java)
- [Odometry and kinematics](#odometry-and-kinematics)
- [Running tests](#running-tests)
- [Building and running this project](#building-and-running-this-project)
- [Design decisions and deliberate simplifications](#design-decisions-and-deliberate-simplifications)
- [Using this as a teaching curriculum](#using-this-as-a-teaching-curriculum)

## Verification status

**Still not compiled, run, or tested** -- same sandboxed session, same network policy
blocking `frcmaven.wpi.edu` and the REVLib/CTRE/Studica Maven hosts. See
`teaching-bot-poc-java`'s README for the full explanation; everything there applies
here too.

**One specific, disclosed uncertainty new to this branch:** `DriveTrain.getPose()`
below calls `odometry.getPoseMeters()`. WPILib's `DifferentialDriveOdometry` class
extends a shared generic `Odometry<T>` base class (also used by
`MecanumDriveOdometry`, `SwerveDriveOdometry`, ...), and `getPoseMeters()` is that
base class's traditional accessor name across the WPILib seasons this was checked
against. It could **not** be independently confirmed against the installed 2026
Java WPILib package the way `DifferentialDrivePoseEstimator`'s API was confirmed
against the real competition port's actual source (that repo skips plain odometry
entirely and goes straight to a pose estimator, so there was no real `.java` file to
cross-check this specific accessor against). The Python sibling's equivalent
(`self._odometry.getPose()`) was independently verified by actually running its
tests in this same session, and RobotPy's `wpimath` bindings are generated directly
from the same WPILib C++ core Java also wraps -- strong circumstantial evidence, not
proof. **If `./gradlew build` reports a missing/renamed method here, this line is
the first place to check.**

## What changed from teaching-bot-poc-java

Only `subsystems/DriveTrain.java` and `src/test/java/frc/robot/subsystems/DriveTrainTest.java`:

- Two new fields: `DifferentialDriveKinematics kinematics` and
  `DifferentialDriveOdometry odometry`, plus a `Field2d field` widget.
- `odometry` is assigned in the **constructor body**, not inline at its field
  declaration -- see that constructor's doc comment for why (it needs
  `configureMotors()` to have already zeroed the encoders first, and Java runs field
  initializers and constructor-body statements in the order they're written).
- Four new public methods: `getWheelSpeeds()`, `getChassisSpeeds()`, `getPose()`,
  `resetPose(Pose2d)`.
- `periodic()` now calls `odometry.update(...)` every loop and
  `field.setRobotPose(getPose())`, plus two new dashboard keys,
  `DriveTrain/PoseXFeet`/`PoseYFeet`.
- Four new tests in `DriveTrainTest.java`: `poseStartsAtOrigin`,
  `odometryTracksStraightLineDriving`, `resetPoseSeedsOdometryAndZeroesEncoders`,
  `chassisSpeedsZeroWhenStopped` -- Java translations of the four the Python sibling
  added for the same reason.

Nothing else in the project (`Constants.java`, every command, every other
subsystem, `RobotContainer.java`, autonomous) changed at all.

## Odometry and kinematics

`DriveTrain` now tracks the robot's estimated position on the field as a `Pose2d`
(X meters, Y meters, heading), via two new pieces added in the constructor:

- **`DifferentialDriveKinematics(TRACK_WIDTH_METERS)`** -- the math relating each
  wheel's own speed to the whole robot's forward speed and turn rate (a
  `ChassisSpeeds`). It needs only the track width because, for a tank drive, that's
  the only geometry that matters: how far apart the wheels are determines how fast
  the robot turns for a given difference between the two sides' speeds.
  `getChassisSpeeds()` exposes this; nothing in this project currently drives from
  it, but it's the natural building block for anything that needs "how fast is the
  whole robot going" rather than "how fast is each wheel going."
- **`DifferentialDriveOdometry`** -- the part that actually accumulates position
  <i>over time</i>. Every loop, `periodic()` feeds it the current gyro heading and
  both encoder distances, and it integrates those into a running pose estimate --
  dead reckoning, the same technique ships have used for centuries: no outside
  reference, just "I know my heading and how far each wheel has turned, so here's
  where I must be now." `getPose()` reads that estimate; `resetPose()` seeds it with
  a known starting position (and, since odometry measures distance <i>since the
  last reset</i>, zeroes the encoders at the same time -- doing one without the
  other would make the two disagree about where "zero" is).

A `Field2d` widget, registered once in the constructor and updated every loop in
`periodic()`, draws this estimated pose on a picture of the field in
Shuffleboard/Glass.

**Dead reckoning drifts.** Nothing corrects this pose against reality -- a
slightly-off track-width measurement, wheel scrub while turning, or a wheel briefly
losing traction all introduce small errors that accumulate the longer the robot
drives without a reset. That's not a bug to fix here; it's <i>the</i> reason
vision-based correction is worth learning next, on `teaching-bot-vision-java`, which
fuses AprilTag detections back into this same pose to correct that drift instead of
trusting dead reckoning alone for an entire match -- identical framing to the
Python sibling's own README at this exact point in its lineage.

## Running tests

```
./gradlew test
```

`subsystems/DriveTrainTest.java` gained four tests over the poc-java branch's
version -- see [What changed](#what-changed-from-teaching-bot-poc-java) above.
`odometryTracksStraightLineDriving` calls `drivetrain.periodic()` directly rather
than stepping the scheduler, matching the Python sibling's pattern exactly (see that
branch's own README for why its version needs to do this to dodge a physics-engine
race -- this branch has no physics engine to race, so the direct call here is really
just for one deterministic update from a known sensor state, not a workaround for
anything).

## Building and running this project

Identical to `teaching-bot-poc-java` -- see that branch's README for the
`gradle-wrapper.jar` gap and how to fix it in one command.

```
./gradlew build   # compile + run tests
./gradlew simulateJava   # run in WPILib's desktop simulator
```

## Design decisions and deliberate simplifications

Identical to `teaching-bot-poc-java`'s list, plus one addition specific to this
branch:

- **Odometry added on its own, before vision, as a separate branch** -- not because
  Java needed it split up any differently than Python did, but because the same
  pedagogical argument applies in both languages: dead reckoning and vision
  correction are two separable ideas worth understanding one at a time. See
  [Odometry and kinematics](#odometry-and-kinematics) above.

## Using this as a teaching curriculum

Same reading order as `teaching-bot-poc-java`, with `DriveTrain.java` (now the
longest file in the project) read once more at the end specifically for its
constructor's field-initialization-order comment and the odometry section above --
a good moment to ask a rookie: what would go wrong if `odometry`'s field declaration
tried to call `getHeadingDegrees()` inline, the same way `kinematics`'s declaration
calls `TRACK_WIDTH_METERS` inline? (Answer: nothing stops that syntactically, but
the encoders wouldn't be zeroed yet, since `configureMotors()` hasn't run -- Java
initializes fields in declaration order, and `configureMotors()` is only called
later, in the constructor body.)
