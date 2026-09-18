# Teaching-Bot: AprilTag Vision

This branch (`teaching-bot-vision`) builds on `teaching-bot-odometry`
(which itself builds on `teaching-bot-poc`), adding one PhotonVision
camera doing AprilTag pose estimation, fusing its fixes into `DriveTrain`'s
pose estimator to correct the dead-reckoning drift the odometry branch's
README named as its reason for existing, and two example navigation
commands built on top of that: drive to a fixed distance in front of a
specific tag, facing it, optionally with a final rotation offset. See
[Vision and AprilTags](#vision-and-apriltags) below for the whole story.
Everything else below is otherwise unchanged from the earlier branches and
repeated here so this branch's README is complete on its own.

A small, standalone RobotPy project built to teach FRC Python to a
rookie-heavy programming subteam, using the same conventions and framework
as the team's real 2026 competition port (`claude/frc-java-robotpy-port-i012ns`
on this repo). It targets a **different, simpler physical robot** than the
competition bot -- see [Physical specs](#physical-specs) -- specifically so
mentors can hand this codebase to new members and let them read, break, and
rebuild it without any risk to the actual competition robot.

Two student mentors who wrote the original Java code, and who know Python in
general but not FRC-Python specifically, are the intended first readers.
Everything below is written for that audience: FRC-specific concepts are
explained, general Python is assumed.

## Contents

- [Physical specs](#physical-specs)
- [File structure](#file-structure)
- [Where do commands live?](#where-do-commands-live)
- [Naming and numbering conventions](#naming-and-numbering-conventions)
- [Subsystems](#subsystems)
- [Commands](#commands)
- [Controller bindings](#controller-bindings)
- [Autonomous](#autonomous)
- [Odometry and kinematics](#odometry-and-kinematics)
- [Vision and AprilTags](#vision-and-apriltags)
- [Running tests](#running-tests)
- [Simulating (`physics.py`)](#simulating-physicspy)
- [2027 alpha preview notes](#2027-alpha-preview-notes)
- [Design decisions and deliberate simplifications](#design-decisions-and-deliberate-simplifications)
- [Assumptions that need bench verification](#assumptions-that-need-bench-verification)
- [Using this as a teaching curriculum](#using-this-as-a-teaching-curriculum)

## Physical specs

- **Drivetrain:** 6-wheel "drop center" differential (tank) drive -- 3
  wheels per side, the center wheel mounted 1/4" LOWER than the front/back
  wheels, so only the center wheel plus one end (front or back) actually
  touches the ground at a time, not all three. That shortens the effective
  ground-contact wheelbase and reduces turning friction compared to a
  6-wheel-flat drivetrain, while keeping 6-wheel traction/durability for
  driving straight. 2x NEO 2.0 motors per side (4 total) through REV
  SparkMax controllers, 8.4:1 gearing. 6" diameter, 1"-wide wheels; 13"
  between wheel centers front-to-back per side; 23" between the left and
  right wheel centerlines.
- **Frame:** 32" front-to-back, 28" side-to-side, 118 lb with bumpers.
- **Shooter:** single 5 lb flywheel, 4x 4" compliant wheels as the shooting
  surface, driven by one Kraken X60 (TalonFX) motor.
- **Trigger:** small NEO-driven cam that flicks a game piece into the
  shooter. One limit switch defines the cam's rest ("home") position; two
  beam-break sensors report loading status upstream.
- **Elevator:** 2-stage single-mast cascade elevator (AndyMark "Elevator in
  a Box" style), spring-assisted extension, rope retraction via a geared
  Redline motor (brushed, no encoder). A motor-driven roller gripper sits at
  the end.
- **Vision:** one PhotonVision camera, centered left/right, mounted 3
  inches back from the front of the frame (13 inches forward of the
  robot's center), 1 foot above the floor, angled 15 degrees upward. See
  [Vision and AprilTags](#vision-and-apriltags) for the exact mounting
  transform and how its sign convention was verified.

## File structure

```
teaching-bot/
├── robot.py                   # Entry point -- commands2.TimedCommandRobot
├── robotcontainer.py          # Wires subsystems, bindings, and autonomous together
├── constants.py                # All numeric/boolean constants, one class per subsystem
├── vision_measurement.py        # One dataclass: a single vision pose fix
├── physics.py                  # `python -m robotpy sim` physics model (drivetrain only)
├── pyproject.toml               # RobotPy project + vendor dependency versions
├── subsystems/                  # Hardware objects + plain actions/queries only
│   ├── drivetrain.py            # DriveTrain
│   ├── shooter.py                # Shooter
│   ├── trigger.py                # Trigger
│   ├── elevator.py                # Elevator
│   ├── gripper.py                 # Gripper
│   └── vision.py                  # Vision
├── commands/                    # Every Command, as an explicit class -- see below
│   ├── drivetrain_commands.py     # TeleopDriveCommand, ResetGyroCommand, DriveDistanceCommand, TurnToAngleCommand
│   ├── shooter_commands.py         # SpinUpShooterCommand
│   ├── trigger_commands.py          # FireCommand
│   ├── elevator_commands.py          # RaiseElevatorCommand, LowerElevatorCommand
│   ├── gripper_commands.py            # IntakeCommand, EjectCommand
│   └── vision_commands.py              # ApproachTagCommand -- this project's one cross-subsystem command
├── autonomous/
│   ├── routines.py               # The two autonomous routines (built from commands/)
│   └── chooser.py                 # Builds the SmartDashboard auto chooser
├── tests/                        # `python -m robotpy test` suite
└── .wpilib/wpilib_preferences.json
```

Notably **absent** compared to the competition bot: no `result.py` (this
project's `VisionMeasurement` doesn't need the competition bot's separate
multi-camera result-arbitration type -- one camera means there's nothing
to arbitrate between), and no `deploy/pathplanner/` (no PathPlanner --
autonomous and `ApproachTagCommand` are both plain PID commands built on
wheel encoders, the gyro, and now AprilTag fixes).

## Where do commands live?

Every command in this project -- teleop driving, both autonomous PID
commands, firing the trigger, raising/lowering the elevator, running the
gripper -- is its own explicit class in `commands/`, subclassing
`commands2.Command` and overriding whichever of `initialize()`/`execute()`/
`isFinished()`/`end()` it actually needs. Subsystems (`subsystems/`) only
expose plain, one-shot hardware actions: `drive()`, `stop()`,
`get_left_distance_meters()`, and so on -- nothing that returns a `Command`.

This is **not** the only valid way to organize a commands2 project, and
it's worth naming the alternative explicitly, since it's the one shown in
most RobotPy examples and in the competition bot this teaching bot is based
on: a **factory-method** style, where each subsystem has methods like
`drive_distance_command(self, feet)` that build and return a command
in one line using `cmd.run(...)`, `cmd.sequence(...)`, `.until(...)`,
`.finallyDo(...)`, and similar composition helpers. That style is genuinely
more idiomatic among experienced RobotPy users, and it's what an earlier
version of this exact codebase used.

The explicit-class style was chosen here instead for one reason: it makes
the command lifecycle -- *initialize once, execute repeatedly, ask
"finished yet?" every loop, end exactly once whether finished normally or
interrupted* -- into four separate, named methods a rookie can point at,
instead of an implicit consequence of how a one-liner composes helper
functions. Compare `commands/trigger_commands.py`'s `FireCommand` (an
explicit class with a plain `self._has_left_home` boolean) to what it
looked like as a factory method:

```python
# The factory-method version this replaced -- functionally identical,
# but its one bit of state has to be a `nonlocal` variable captured in a
# closure, and "when does this run" is implicit in how .sequence()/.until()/
# .finallyDo() are composed rather than four named methods.
def fire_command(self) -> Command:
    has_left_home = False

    def _init() -> None:
        nonlocal has_left_home
        has_left_home = False

    def _finished() -> bool:
        nonlocal has_left_home
        if not has_left_home:
            if not self.is_at_home():
                has_left_home = True
            return False
        return self.is_at_home()

    return (
        cmd.sequence(cmd.runOnce(_init, self), cmd.run(self.run_cam, self).until(_finished))
        .withTimeout(TriggerConstants.FIRE_TIMEOUT_SECONDS)
        .finallyDo(lambda interrupted: self.stop_cam())
    )
```

Both versions do the same thing. The explicit-class version needs `nonlocal`
nowhere, its lifecycle methods are separately readable, and each one is a
plain function that can be stepped through in a debugger without also
having to understand closures and command-decorator composition at the
same time. The cost is more boilerplate per command (an `__init__` and
`addRequirements()` call every class needs). For a rookie-facing codebase,
that trade is worth it; for the competition bot -- written by mentors who
already think fluently in the factory style -- it wasn't, and that
codebase keeps the terser pattern. Neither is "more correct"; this file
exists so the choice, and the reasoning, are visible instead of assumed.

**No lambdas or inline commands anywhere in this project, including
one-shot actions.** Two places that would commonly reach for a lambda
instead get their own class:

- Resetting the gyro (bound to the driver's Back button) is
  `commands/drivetrain_commands.py`'s `ResetGyroCommand` -- a full class
  whose `initialize()` does the work and whose `isFinished()` returns
  `True` immediately, rather than the shorter (but lambda-based)
  `cmd.runOnce(self.drivetrain.reset_gyro, self.drivetrain)`.
- `TeleopDriveCommand` takes the driver's `CommandXboxController` object
  directly and calls `.getLeftY()`/`.getRightY()` on it inside its own
  `execute()`, rather than taking two `Callable[[], float]` arguments
  filled in with `lambda: -self._driver_controller.getLeftY()` at the
  binding site. Same behavior, one idea (a controller object) instead of
  two (a callable *and* a closure capturing `self`).

Neither rewrite changes what the robot does -- both are exactly the kind of
"same behavior, more boilerplate, less to understand at once" trade the
explicit-class choice above already makes. The line for *this* codebase is
simple: if writing a command would otherwise require a lambda or an inline
`cmd.xyz(...)` call, give it a class and a name instead, even if that class
is only two or three lines long.

**A decision this project made earlier turned out to be wrong, once
vision entered the picture -- and that's fine.** The `teaching-bot-poc`
README said outright: "nothing here needs a command that spans more than
one subsystem." `commands/vision_commands.py`'s `ApproachTagCommand` needs
both `DriveTrain` (to move) and `Vision` (to know where a tag is), so that
statement no longer holds. The right response to a decision the codebase
has outgrown is to update it, not defend it -- this file, and that one's
`commands/` file structure, are worth reading as an example of a project
revising its own earlier reasoning in public, rather than pretending it
was always going to end up here. `ApproachTagCommand` still calls
`self.addRequirements(drivetrain)` and NOT `vision` -- it only ever reads
from `Vision`, never commands it, and `addRequirements()` is for
preventing two commands from fighting over something they both *drive*.

## Naming and numbering conventions

Same conventions as the competition bot's README, reused here so a
student moving between the two codebases doesn't have to relearn anything.

### FRC-wide conventions

- **SI units internally, imperial at the boundary.** WPILib math
  (`wpimath`) works in meters/radians/seconds even though FRC parts are
  specified in inches and degrees. Conversions happen once, right where a
  human-facing constant becomes a WPILib value -- see every
  `* 0.0254` (inches to meters) in `constants.py`, and
  `constants.METERS_PER_FOOT`, used at exactly two boundaries:
  `DriveDistanceCommand`'s constructor (feet in, meters used internally)
  and `DriveTrain.periodic()`'s dashboard telemetry (meters out, feet
  displayed). Angles have no separate "imperial" unit, so headings stay in
  degrees everywhere.
- **Controller ports 0 and 1 = driver and operator.** Not a WPILib
  requirement, but close to universal across FRC teams.
- **Positive rotation is counterclockwise**, 0 degrees = whatever heading
  the gyro was most recently reset to. `DriveTrainConstants.TURN...` and
  `Auto.DRIVE_TURN_DRIVE_TURN_DEGREES` follow this: positive = turn left.

### Team 2408 conventions (reused from the competition bot)

- **CAN IDs are grouped by subsystem, by tens.** `20`-`23` = DriveTrain,
  `30`-`31` = Shooter/Trigger (grouped together since Trigger feeds the
  Shooter directly), `40`-`41` = Elevator/Gripper (grouped together since
  the Gripper physically rides on the Elevator). A new subsystem should
  claim its own decade rather than reusing one.
- **Constants live in one file, grouped into a namespace class per
  subsystem.** `constants.py` has `DriveTrainConstants`, `ShooterConstants`,
  `TriggerConstants`, `ElevatorConstants`, `GripperConstants`, `Auto`,
  `OperatorConstants`.
- **One file per subsystem/command-group, filename matches what it
  contains, in lowercase:** `DriveTrain` -> `subsystems/drivetrain.py`;
  its commands -> `commands/drivetrain_commands.py`.
- **Every Command is an explicit class, in `commands/`, named
  `<Verb><Noun>Command` -- including one-shot actions, and never a
  lambda:** `TeleopDriveCommand`, `ResetGyroCommand`,
  `DriveDistanceCommand`, `FireCommand`, `RaiseElevatorCommand`. See
  [Where do commands live?](#where-do-commands-live) for why, and how this
  differs from the competition bot's convention.
- **Private/internal state is `_`-prefixed:** `_target_rpm`,
  `_telemetry_loop_counter`, `_has_left_home`, etc.

### Python naming (PEP 8)

| | Convention | Example |
|---|---|---|
| Functions/methods/variables | `snake_case` | `get_left_distance_meters()` |
| Classes | `PascalCase` | `DriveTrain`, `DriveDistanceCommand` |
| Constants | `ALL_CAPS_WITH_UNDERSCORES` | `SHOOTER_MOTOR_ID` |
| Files | `snake_case.py`, one module per file | `drivetrain.py` |

**The one thing that trips people up:** our own code follows the table
above, but every call *into* a vendor library (`wpilib`, `commands2`,
`rev`) still uses that library's own naming, which is Java-style
`camelCase` as of the 2026 season (`phoenix6`, used here for the Kraken
shooter motor, is the one exception -- it already ships native
`snake_case`, a choice CTRE made independently of WPILib's own timeline).
So a single line can legitimately mix both:
`self._left_encoder.getPosition()` -- our snake_case variable, calling a
camelCase method from REV's library. This isn't a mistake to fix; it's a
real seam between our code and code we don't own, worth pointing out
explicitly the first time a rookie notices it.

**Is `CommandXboxController` deprecated?** No -- verified directly against
the installed `robotpy-commands-v2==2026.2.2` package: no deprecation
warning on construction or use, and it's the only class with Xbox-specific
button names (`a()`, `b()`, `leftBumper()`, etc.) for command-based binding.
There is no `GameController`/`CommandGameController` class in RobotPy.
What *does* exist, and is worth knowing about, is a real rename already
visible in the 2027 alpha -- see the next section.

## Subsystems

| Subsystem | File | Purpose | Hardware | Key methods |
|---|---|---|---|---|
| **DriveTrain** | `subsystems/drivetrain.py` | Moves the robot (tank/differential drive) and tracks where it is on the field | 4x REV SparkMax NEO 2.0 (CAN 20-23), navX2 gyro (SPI/MXP) | `drive()`, `stop()`, `get_left/right_distance_meters()`, `get_left/right_velocity_meters_per_second()`, `get_heading_degrees()`, `reset_gyro()`, `reset_encoders()`, `get_pose()`, `reset_pose()`, `get_wheel_speeds()`, `get_chassis_speeds()` |
| **Shooter** | `subsystems/shooter.py` | Spins the flywheel to a fixed target RPM | 1x Kraken/TalonFX (CAN 30) | `set_target_rpm()`, `stop()`, `get_current_rpm()`, `is_at_target_speed()` |
| **Trigger** | `subsystems/trigger.py` | Fires one game piece into the shooter per cam revolution | 1x NEO/SparkMax (CAN 31), 1 limit switch (DIO 0), 2 beam breaks (DIO 1-2) | `run_cam()`, `stop_cam()`, `is_at_home()`, `has_ball_at_stage_1()`, `has_ball_at_stage_2()` |
| **Elevator** | `subsystems/elevator.py` | Raises/lowers the 2-stage mast | 1x geared Redline/SparkMax, brushed, no encoder (CAN 40), top/bottom limit switches (DIO 3-4) | `set_speed()`, `stop()`, `is_at_top()`, `is_at_bottom()` |
| **Gripper** | `subsystems/gripper.py` | Spinning roller intake at the end of the elevator | 1x NEO 550/SparkMax (CAN 41) | `set_speed()`, `stop()` |
| **Vision** | `subsystems/vision.py` | Estimates the robot's field pose from AprilTags, and looks up where a specific tag is | 1x PhotonVision camera + coprocessor | `get_tag_pose()`, `get_best_vision_measurement_if_fresh()`, `is_any_vision_available()` |

Notice none of these have a method that returns a `Command` anymore --
that's the point of the split described in
[Where do commands live?](#where-do-commands-live). `DriveTrain`'s key
methods above are unchanged from the odometry branch by name -- `get_pose()`
and `reset_pose()` still mean the same thing to a caller -- but under the
hood they now read from/reset a `DifferentialDrivePoseEstimator` that also
folds in `Vision`'s fixes, not a plain `DifferentialDriveOdometry`. See
[Vision and AprilTags](#vision-and-apriltags).

## Commands

| Command | Subsystem | Bound as | What it does |
|---|---|---|---|
| `TeleopDriveCommand` | DriveTrain | Default command | Tank drive read directly from the driver controller's two joystick Y-axes |
| `ResetGyroCommand` | DriveTrain | `onTrue` | One-shot: resets the navX heading to 0 |
| `DriveDistanceCommand(feet)` | DriveTrain | Used in autonomous | PID-drives straight to a target distance, in feet |
| `TurnToAngleCommand(degrees)` | DriveTrain | Used in autonomous | PID-turns to an absolute heading using the navX gyro |
| `SpinUpShooterCommand` | Shooter | `toggleOnTrue` | Toggle: commands the flywheel to a fixed target RPM, or stops it |
| `FireCommand` | Trigger | `onTrue`, with `.withTimeout(...)` | Runs the cam motor until it leaves and then returns to the home (limit-switch) position; safety-timed out at the binding site in case the switch never re-triggers |
| `RaiseElevatorCommand` / `LowerElevatorCommand` | Elevator | `whileTrue` | Drives the lift motor while held, auto-stopping at the relevant limit switch even if the button is still held |
| `IntakeCommand` / `EjectCommand` | Gripper | `whileTrue` | Spins the roller in/out while held |
| `ApproachTagCommand(tag_id, standoff_feet, face_offset_degrees=0.0)` | DriveTrain (reads Vision) | `onTrue` | 3-phase PID: turn to face a point a fixed standoff distance in front of a specific tag, drive to that point, then turn to a final heading (facing the tag, or offset from it) |

Every one of these overrides some subset of `initialize()`/`execute()`/
`isFinished()`/`end()` -- see each file in `commands/` for which, and why.

## Controller bindings

**Driver (port 0) -- drive only:**

| Input | Action |
|---|---|
| Left stick Y | Left-side drive speed |
| Right stick Y | Right-side drive speed |
| Back button | Reset gyro heading to 0 (do this before every autonomous run) |
| A | Approach AprilTag 15, stop 3 feet away facing it |
| X | Approach AprilTag 15, stop 5 feet away, then rotate to face 45 degrees right of facing it |

**Operator (port 1) -- everything else:**

| Input | Action |
|---|---|
| A | Toggle shooter spin-up |
| B | Fire trigger |
| X | Gripper intake (while held) |
| Y | Gripper eject (while held) |
| Right Bumper | Raise elevator (while held) |
| Left Bumper | Lower elevator (while held) |

## Autonomous

Two real routines plus a safe default, selected from a `SendableChooser` on
SmartDashboard/Shuffleboard ("Auto Chooser"):

| Option | What it does |
|---|---|
| **Do Nothing** (default) | Runs `cmd.none()` -- the robot does nothing during autonomous. Chosen as the default deliberately: an unselected chooser should never surprise anyone by driving. |
| **Drive Forward 10 ft** | `DriveDistanceCommand(drivetrain, 10.0)`, straight. |
| **Drive 5ft, Turn Left 90, Drive 3ft** | `DriveDistanceCommand(drivetrain, 5.0)`, then `TurnToAngleCommand(drivetrain, 90.0)` (left), then `DriveDistanceCommand(drivetrain, 3.0)`. |

Both routines are built entirely from `commands/drivetrain_commands.py`'s
two PID commands -- no PathPlanner, no vision, no pre-authored paths. All
the actual numbers (`10.0`, `5.0`, `90.0`, `3.0`) live in `constants.Auto`,
in feet/degrees, not buried in `autonomous/routines.py`.

## Odometry and kinematics

This section describes what the `teaching-bot-odometry` branch this one
builds on added, unchanged, because the ideas (kinematics, dead reckoning,
why it drifts) are the right foundation for what
[Vision and AprilTags](#vision-and-apriltags) below builds on top of them.
One thing **has** changed since: `get_pose()`/`reset_pose()` are no longer
backed by a plain `DifferentialDriveOdometry` in this branch's actual code
-- see that section for the upgrade. Reading this section first, as pure
dead reckoning with nothing correcting it, is still the right order to
learn it in.

`DriveTrain` tracks the robot's estimated position on the field as a
`Pose2d` (X meters, Y meters, heading), via two pieces added in
`__init__()` on the odometry branch:

- **`DifferentialDriveKinematics(TRACK_WIDTH_METERS)`** -- the math
  relating each wheel's own speed to the whole robot's forward speed and
  turn rate (a `ChassisSpeeds`). It needs only the track width because,
  for a tank drive, that's the only geometry that matters: how far apart
  the wheels are determines how fast the robot turns for a given
  difference between the two sides' speeds. `get_chassis_speeds()`
  exposes this; nothing in this project currently drives from it, but
  it's the natural building block for anything that needs "how fast is
  the whole robot going" rather than "how fast is each wheel going." This
  piece is unchanged by the vision branch.
- **`DifferentialDriveOdometry`** -- the part that actually accumulates
  position *over time*. Every loop, `DriveTrain.periodic()` fed it the
  current gyro heading and both encoder distances, and it integrated
  those into a running pose estimate -- dead reckoning, the same
  technique ships have used for centuries: no outside reference, just "I
  know my heading and how far each wheel has turned, so here's where I
  must be now." `get_pose()` read that estimate; `reset_pose()` seeded it
  with a known starting position (and, since odometry measures distance
  *since the last reset*, zeroed the encoders at the same time -- doing
  one without the other would make the two disagree about where "zero"
  is).

A `wpilib.Field2d()` widget, registered once in `__init__()` and updated
every loop in `periodic()`, draws this estimated pose on a picture of the
field in Shuffleboard/Glass -- both live on the real robot and in
`python -m robotpy sim`.

**Dead reckoning drifts.** Nothing described so far corrects this pose
against reality -- a slightly-off track-width measurement, wheel scrub
while turning, or a wheel briefly losing traction all introduce small
errors that accumulate the longer the robot drives without a reset. That
was the odometry branch's own stated reason vision-based correction was
worth learning next -- see [Vision and AprilTags](#vision-and-apriltags)
immediately below for how this branch actually does that, including the
swap from `DifferentialDriveOdometry` to `DifferentialDrivePoseEstimator`
that vision fusion requires.

## Vision and AprilTags

This branch adds one PhotonVision camera and fuses what it sees back into
`DriveTrain`'s pose, correcting the drift described just above instead of
trusting dead reckoning alone for an entire match. It also adds one command,
`ApproachTagCommand`, that uses the camera's knowledge of where a specific
tag is to drive the robot to a point relative to that tag.

### Camera mounting

`VisionConstants.ROBOT_TO_CAMERA` in `constants.py` encodes the camera's
physical mounting as a `Transform3d` from the robot's center to the camera
lens -- exactly the numbers given for this robot: centered left/right, 3
inches back from the front of the frame, 1 foot above the floor, angled 15
degrees upward.

```python
ROBOT_TO_CAMERA = Transform3d(
    Translation3d((16.0 - 3.0) * 0.0254, 0.0, 1.0 * 0.3048),
    Rotation3d(0.0, math.radians(-15.0), 0.0),
)
```

Two things worth explaining, since both are easy to get backwards:

- **"3 inches from the front of the frame" isn't the same number as "3
  inches forward of center."** The frame is 32 inches front-to-back
  (see [Physical specs](#physical-specs)), so its center is 16 inches back
  from the front edge. A camera 3 inches back from the front edge is
  therefore `16 - 3 = 13` inches forward of the robot's own center --
  `Transform3d`, like everything else in `wpimath`, measures from the
  robot's origin (its center), not from any particular edge of the frame.
  Left/right offset is 0 because the camera is centered; height is a
  straight foot-to-meter conversion since `Translation3d`'s Z axis is
  already "up" from the floor.
- **Negative pitch tilts the camera up, not down -- verified directly,
  not assumed.** `Rotation3d`'s pitch convention is easy to get backwards
  by guessing, so before writing this constant, a small script constructed
  a `Transform3d` with a test pitch, applied it to a level "camera looks
  along the ground" `Pose3d`, and checked which way the resulting forward
  vector's Z component actually moved. A negative pitch moved it up. That
  same check would also have caught a units mistake (degrees vs. radians
  passed to `Rotation3d`, which takes radians) had one been made. This is
  worth a rookie repeating for themselves rather than taking on faith --
  it's a one-line experiment, and "the sign convention I want is probably
  right" is exactly the kind of assumption this whole project tries to
  model *not* making.

### Pose estimation: from odometry to a pose estimator

`DriveTrain` swapped its plain `DifferentialDriveOdometry` (the odometry
branch) for a `DifferentialDrivePoseEstimator`. The two take the same
encoder/gyro inputs every loop, but the pose estimator also accepts vision
fixes via `addVisionMeasurement(pose, timestamp, std_devs)`, and
internally runs a Kalman filter that blends a vision fix in proportionally
to how confident it is (smaller `std_devs` = more trusted = pulled toward
harder), rather than either ignoring vision or snapping straight to it.
Every loop, `DriveTrain.periodic()` asks `Vision` for its best fresh
measurement and, if there is one, feeds it in:

```python
measurement = self._vision.get_best_vision_measurement_if_fresh()
if measurement is not None:
    self._pose_estimator.addVisionMeasurement(
        measurement.estimated_pose,
        measurement.timestamp_seconds,
        measurement.standard_deviations,
    )
```

`get_pose()` and `reset_pose()` keep their exact same names and signatures
from the odometry branch -- a caller elsewhere in the codebase (like
`ApproachTagCommand`) doesn't need to know or care whether the pose behind
them is pure dead reckoning or vision-fused.

### How `Vision` decides a measurement is trustworthy

`subsystems/vision.py`'s `_compute_measurement()` runs several checks
before a camera result becomes something `DriveTrain` is allowed to fuse
in, each one guarding against a specific, real failure mode rather than
being defensive for its own sake:

- **No targets, no estimate.** `PhotonPoseEstimator` is asked for a
  multi-tag estimate first (`estimateCoprocMultiTagPose()`), falling back
  to a lowest-ambiguity single-tag estimate
  (`estimateLowestAmbiguityPose()`) when multi-tag PNP data isn't
  available -- see the note in `subsystems/vision.py` about this being a
  2026 `photonlibpy` API change from the Java version.
- **A pose off the field is never real.** If the estimated X/Y falls
  outside the field's own dimensions (from the same
  `AprilTagFieldLayout` used to look up tag positions), it's thrown out --
  a bad reprojection producing a robot "10 meters past the wall" is a sign
  of a bad estimate, not a real robot position.
- **A single ambiguous tag is untrustworthy.** One AprilTag alone is a
  genuinely ambiguous pose problem -- the same tag corners are consistent
  with two different camera poses, mirrored through the tag's plane.
  PhotonVision's own `getPoseAmbiguity()` score flags when that ambiguity
  is high; above `VisionConstants.MAX_AMBIGUITY`, the whole measurement is
  dropped rather than trusted.
- **Confidence (`std_devs`) scales with how the estimate was made.**
  Multiple tags in view resolve that ambiguity geometrically, so
  `MULTI_TAG_STDDEVS` (tightest) applies whenever at least
  `VisionConstants.MIN_TAGS_FOR_MULTI_TAG` tags contributed. A single tag
  gets looser trust, and looser still (`SINGLE_TAG_FAR_STDDEVS` vs.
  `SINGLE_TAG_CLOSE_STDDEVS`) the farther away it is -- distance amplifies
  any small pixel-level error in the corner detection into a larger
  real-world position error. A single tag farther than
  `VisionConstants.MAX_TAG_DISTANCE_METERS` is dropped outright rather
  than trusted at any confidence.
- **Staleness matters for both fusion and `is_any_vision_available()`.**
  `get_best_vision_measurement_if_fresh()` refuses to hand back a
  measurement older than `VisionConstants.MAX_VISION_AGE_SECONDS` -- a
  processing delay or a dropped frame shouldn't let `DriveTrain` fuse in a
  pose fix that was accurate a moment ago but describes where the robot
  *was*, not where it *is*. `is_any_vision_available()` answers a
  different, looser question ("has the camera sent anything recently at
  all") used only for dashboard telemetry, not for anything safety- or
  accuracy-critical.

### `ApproachTagCommand`

`commands/vision_commands.py`'s `ApproachTagCommand` is bound twice in
`robotcontainer.py`, once per requested behavior, both against the same
example tag ID (`VisionConstants.EXAMPLE_TAG_ID = 15`):

```python
self._driver_controller.a().onTrue(
    ApproachTagCommand(
        self.drivetrain, self.vision,
        VisionConstants.EXAMPLE_TAG_ID, VisionConstants.APPROACH_STANDOFF_FEET,
    )
)
self._driver_controller.x().onTrue(
    ApproachTagCommand(
        self.drivetrain, self.vision,
        VisionConstants.EXAMPLE_TAG_ID, VisionConstants.APPROACH_AND_TURN_STANDOFF_FEET,
        VisionConstants.APPROACH_AND_TURN_OFFSET_DEGREES,
    )
)
```

Both are the same command class, parameterized -- **not** two near-duplicate
classes the way `RaiseElevatorCommand`/`LowerElevatorCommand` are. That's a
deliberate exception to this project's usual "each distinct behavior gets
its own named class" convention (see
[Where do commands live?](#where-do-commands-live)), made because these two
behaviors share the entire 3-phase state machine below and differ by
exactly one number (`face_offset_degrees`, 0 for "just face the tag" and 45
for "face 45 degrees right of it"). Two copies of that state machine would
mean any future bugfix to the turn/drive/turn sequence has to be applied
twice and could silently drift apart; one parameterized class can't drift
from itself. See
[Design decisions and deliberate simplifications](#design-decisions-and-deliberate-simplifications)
for the flip side of this argument.

The command runs as three phases, each one a fresh PID setpoint on top of
`DriveTrain`'s existing turn/distance PID gains (no separate gains were
tuned for vision navigation -- see
[Assumptions that need bench verification](#assumptions-that-need-bench-verification)):

1. **`TURN_TO_TARGET`** -- `initialize()` looks up the tag's known field
   pose via `vision.get_tag_pose(tag_id)` (a plain field-layout lookup,
   independent of whether the camera can currently see that tag), computes
   a target point `standoff_feet` in front of the tag along the direction
   the tag itself faces, and turns the robot to face that point.
2. **`DRIVE_TO_TARGET`** -- drives straight to that point, using the same
   relative-baseline distance pattern as `DriveDistanceCommand` (see
   [Design decisions and deliberate simplifications](#design-decisions-and-deliberate-simplifications)
   for why that pattern matters here specifically).
3. **`FACE_TAG`** -- turns to a final heading: facing the tag directly when
   `face_offset_degrees` is 0, or offset from that by the given number of
   degrees otherwise. The command finishes once this final turn reaches its
   setpoint.

If the tag ID doesn't exist in the field layout, `initialize()` sets an
internal `FAILED` phase and the command ends immediately -- it doesn't
throw, hang, or drive toward a nonexistent point.

## Running tests

```
python -m robotpy test
```

This runs `pyfrc`'s pytest plugin, which provides the `robot`/`control`
fixtures used throughout `tests/`. 26 tests currently cover: the standard
pyfrc smoke tests, a full disabled -> autonomous -> teleop lifecycle cycle,
DriveTrain's encoder bookkeeping, odometry, and kinematics, both PID
commands actually reaching their setpoint and stopping, Trigger's
`FireCommand` edge-detection state machine (including its timeout),
Elevator's limit-switch-gated raise/lower commands, and this branch's
additions in `tests/test_vision.py` (tag-pose lookup and "no camera data
yet" defaults) and `tests/test_vision_commands.py`
(`ApproachTagCommand`'s target-point math and its progression through all
three phases). All 26 currently pass, verified against `robotpy==2026.2.2`
and its pinned vendor dependencies (see `pyproject.toml`).

**A gotcha worth teaching explicitly**, hit while writing `tests/test_trigger.py`
and `tests/test_elevator.py`: a `Command` can only be `.schedule()`d while
the robot is *enabled* (its `runsWhenDisabled()` defaults to `False`). If
you call `command.schedule()` before a test has enabled the robot via
`control.step_timing(..., enabled=True)`, the scheduler silently drops the
command and `isScheduled()` reads `False` forever afterward -- there's no
error, just a command that never runs. Every test in this suite that
schedules a command directly does one small `step_timing(enabled=True)`
step first for exactly this reason.

**Another one, found while writing this branch's odometry tests, that
corrects something the previous branch's tests got wrong**: `physics.py`'s
`PhysicsEngine` actually DOES run under `python -m robotpy test`, not only
under `python -m robotpy sim` as an earlier version of this file claimed
(including this same README, on the `teaching-bot-poc` branch this one
builds on -- corrected there too). Every loop, it recomputes each
simulated motor's encoder reading (and the navX's simulated yaw) from real
motor physics and overwrites whatever was there before -- including a
value a test just poked in directly. That's harmless for
`test_drive_distance_command_finishes_once_target_reached()`/
`test_turn_to_angle_command_finishes_once_heading_reached()`: they poke a
sensor and immediately check that same loop's `isFinished()`-driven
`isScheduled()` result, and the scheduled command reads the poked value
before physics.py's own recomputation overwrites it a moment later. It is
**not** harmless for checking a value after the fact, which is exactly
what testing odometry needs to do -- so `test_odometry_tracks_straight_line_driving()`
and friends poke the encoders/gyro and then call `drivetrain.periodic()`
directly, bypassing the scheduler (and therefore physics.py's hook into
it) entirely, for one deterministic odometry update from a known sensor
state. See `tests/test_drivetrain.py`'s module docstring for the full
explanation.

**A third gotcha, specific to this branch:** the navX sim quirk above also
applies to `tests/test_vision_commands.py`'s phase-progression test, but
with an extra wrinkle -- a raw `SimDeviceSim("navX-Sensor[4]").getDouble("Yaw").set(x)`
poke isn't synchronously visible via `AHRS.getAngle()` even within the same
`control.step_timing()` call; it takes an actual simulated time step to
refresh, and `physics.py` may then overwrite it again a moment later. So
that test checks `command._phase` (the command's own Python state,
untouched by physics.py) immediately after each poke-and-step, the same
discipline as the odometry tests above, rather than trying to read a raw
sensor value back out afterward. See `tests/test_vision_commands.py`'s
module docstring.

## Simulating (`physics.py`)

```
python -m robotpy sim
```

Only the drivetrain is modeled. The REV NEO 2.0 has no built-in `DCMotor`
factory in `wpimath` as of the 2026 season, so it's hand-built in
`physics.py` from REV's published spec sheet (free speed 5676 RPM -- same
as the original NEO -- stall torque 3.75 N·m, stall current 150 A, free
current 1.8 A). The raw `DCMotor(nominalVoltage, stallTorque, stallCurrent,
freeCurrent, freeSpeed, numMotors)` constructor was verified directly
against the installed `robotpy-wpimath==2026.2.2` package before being
relied on here. Shooter, Trigger, Elevator, and Gripper have no physical
model -- their telemetry in sim reflects commanded setpoints, not a
simulated physical response.

**The camera is not modeled either.** `physics.py` only drives the
drivetrain's simulated encoders and gyro from motor voltage; there's no
simulated camera feed, so `PhotonCamera.getLatestResult()` returns an
empty, disconnected-camera result under `python -m robotpy sim` just as it
does under `python -m robotpy test` (see `tests/test_vision.py`'s module
docstring for exactly what that placeholder result looks like).
`ApproachTagCommand` will therefore fail immediately in sim unless the tag
pose lookup itself is exercised directly (`vision.get_tag_pose(...)`,
which doesn't need a camera at all) -- driving the full command end-to-end
needs either real camera hardware or a hand-written test that pokes
`Vision`'s internal state the way `tests/test_vision_commands.py` does.

## 2027 alpha preview notes

The rest of this codebase runs on 2026-stable RobotPy, because the
command-based framework it's built on (`commands2`/`robotpy-commands-v2`)
has **no 2027 release at all yet** -- confirmed by checking PyPI/the
installed alpha environment in this session. None of what follows is
usable today; it's recorded here so whoever eventually ports this project
forward knows what's already changed underneath, verified directly against
`robotpy==2027.0.0a6` (the newest alpha with published native wheels for
this environment at the time of writing -- `a7` exists but is missing a
native package's wheels, the same gap this repo's `2027-alpha-tracking`
branch already documents):

- **`wpilib.XboxController` has already been renamed to
  `wpilib.NiDsXboxController`.** PS4/PS5/Stadia controller classes were
  renamed the same way (`NiDsPS4Controller`, etc.); the generic
  `wpilib.GenericHID` base class was **not** renamed. Since `commands2` has
  no 2027 release, there is no `CommandNiDsXboxController` yet -- if
  RobotPy's `Command<Class>` naming pattern holds once it catches up,
  that's the name to expect.
- **`wpilib.TimedRobot.robotInit()` no longer exists as a lifecycle method**
  (checked directly: it's simply absent from the class in `2027.0.0a6`). A
  4th mode, "Utility," has been added alongside disabled/autonomous/teleop
  (`utilityInit()`/`utilityPeriodic()`/`utilityExit()`, `isUtility()`).
  `robot.py`'s `robotInit()` in this project is unaffected today only
  because `TimedCommandRobot` (the class it actually subclasses) is a
  `commands2` class, and doesn't exist in 2027 yet either.
- **`wpilib.MatchState` and `wpilib.RobotState` already exist as their own
  top-level classes**, splitting apart state that today all lives on
  `wpilib.DriverStation`.
- **Method naming inside `wpilib` is still `camelCase`** in `2027.0.0a6` --
  `NiDsXboxController.getLeftY()`, not `get_left_y()`. The wholesale
  snake_case migration mentioned in WPILib's own yearly changelog notes
  (see this repo's `2027-alpha-tracking` branch) hadn't landed in the core
  `wpilib` bindings as of this specific alpha; don't assume it's already
  everywhere just because `phoenix6` (a separate, CTRE-maintained vendor
  library, unrelated to this timeline) already ships native snake_case.

## Design decisions and deliberate simplifications

Each of these is a real choice, made and stated here rather than left for
someone to reverse-engineer later:

- **Every command is an explicit class in `commands/`, not a factory
  method on a subsystem.** See
  [Where do commands live?](#where-do-commands-live) for the full
  reasoning and a side-by-side comparison.
- **Odometry, then vision-fused pose estimation, as two separate branches
  instead of one.** The competition bot's DriveTrain fuses encoders, gyro,
  AND vision into a `DifferentialDrivePoseEstimator` from day one. This
  project deliberately split that into `teaching-bot-odometry` (encoder/gyro
  dead reckoning alone, via `DifferentialDriveKinematics` +
  `DifferentialDriveOdometry`) and this branch, `teaching-bot-vision` (which
  upgrades that to a `DifferentialDrivePoseEstimator` fusing in AprilTag
  fixes) -- see [Odometry and kinematics](#odometry-and-kinematics) and
  [Vision and AprilTags](#vision-and-apriltags) -- because dead reckoning
  and vision correction are two separable ideas worth understanding one at
  a time, and a rookie who can already explain *why* dead reckoning drifts
  gets far more out of learning what vision fixes than one meeting both
  ideas simultaneously.
- **`DriveDistanceCommand` used to reset the encoders every time it ran --
  now it doesn't, and that fix mattered for a reason that didn't exist
  before this project had odometry.** Resetting the hardware encoders to
  zero at the start of every distance-drive was harmless back when nothing
  else cared about the encoders' absolute reading. Once `DriveTrain` started
  computing odometry from those same encoders every loop, an external reset
  mid-match silently corrupted the pose estimate -- odometry has no way to
  know a reset happened out from under it, so it would compute a wrong,
  sudden "jump" in position on the very next loop. The fix (in both
  `DriveDistanceCommand` and `ApproachTagCommand`'s drive phase) is to
  capture the *current* distance as a baseline in `initialize()` and measure
  progress relative to that baseline, without ever touching the actual
  hardware encoder. `tests/test_drivetrain.py`'s
  `test_drive_distance_command_does_not_corrupt_pose_between_legs()` is a
  regression test for exactly this. Worth asking a rookie: what other
  commands in this codebase read a sensor that something else also depends
  on, and would have the same class of bug if they ever called a `reset_*()`
  method on it?
- **`ApproachTagCommand` is one parameterized class, not two, breaking this
  project's usual one-class-per-behavior convention on purpose.** See
  [`ApproachTagCommand`](#approachtagcommand) for the full reasoning. It's
  worth naming the tension directly: `RaiseElevatorCommand`/
  `LowerElevatorCommand` chose two classes for two behaviors that share
  almost nothing (different direction, different limit switch) beyond
  "drive a motor." `ApproachTagCommand`'s two bound behaviors share an
  entire 3-phase state machine and differ by one number. The rule this
  project actually follows isn't "one class per button binding" -- it's
  "one class per genuinely distinct piece of logic," and two callers of the
  same logic with different arguments are still one piece of logic.
- **Every command has the same four-method shape, but not every command
  overrides all four -- and that's informative, not arbitrary.**
  `SpinUpShooterCommand` has no `execute()` at all: Phoenix 6's velocity
  control is closed-loop on the TalonFX itself (see `subsystems/shooter.py`),
  so the command only needs to *tell* it a target once (`initialize()`) and
  *tell* it to stop once (`end()`) -- there's nothing to redo every 20ms
  loop. Every open-loop, duty-cycle motor (drivetrain teleop, elevator
  lift, gripper roller) has no such loop running on the motor controller,
  so those commands' `execute()` really is doing something every single
  call. Worth asking a rookie, after they've read both, why
  `RaiseElevatorCommand` needs an `execute()` and `SpinUpShooterCommand`
  doesn't.
- **`DriveDistanceCommand` is PID, not "drive at a fixed speed until the
  encoder says stop."** An earlier version of this command did the latter,
  and it either overshot (the motors were still at full speed right up to
  the instant the target was crossed) or needed a hand-tuned "stop a bit
  early" fudge factor to compensate. A PID loop instead recalculates "how
  hard to push" every loop from the remaining distance, so the commanded
  speed naturally tapers off on approach. Its output is also clamped to
  `DriveTrainConstants.DRIVE_DISTANCE_MAX_OUTPUT` as an independent safety
  margin on top of a conservative `KP` -- worth asking a rookie what would
  happen (hint: full-power lurch toward a 10-foot target from a dead stop)
  if that clamp were removed.
- **Elevator is entirely open-loop.** The Redline motor is brushed with no
  built-in encoder, and no external encoder is installed on this robot, so
  there's no sensor to close a position loop against. Two limit switches
  (top/bottom) are the only feedback, and the commands stop the motor the
  instant either one trips -- even if the operator is still holding the
  button. A real elevator that needs to stop at intermediate heights (not
  just fully up/down) would need an added encoder (a through-bore or
  versa-style external one) and a position PID loop, much like
  `DriveDistanceCommand`; that's a good example of "the code can't do more
  than the sensors let it."
- **Trigger's "fire" is an edge-detection state machine, not a single
  sensor read.** The cam has exactly one sensor: a limit switch at "home."
  One complete fire cycle means *leaving* home and then *returning* to it
  -- reading the switch once, at the start, would falsely report "done"
  immediately (it starts at home). `FireCommand` tracks that as one plain
  instance attribute, `self._has_left_home`, reset every time the command
  restarts in `initialize()`. It has no timeout of its own; the safety
  timeout (in case the switch never re-triggers -- a jam, a broken wire) is
  applied as a `.withTimeout()` decorator where the command is actually
  bound to a button, in `robotcontainer.py` -- decorators like
  `.withTimeout()`/`.andThen()`/`.until()` work identically on an explicit
  class and a one-liner `cmd.run(...)`, which is exactly why it doesn't
  matter that different commands in this project build their behavior in
  different ways internally.

## Assumptions that need bench verification

Every one of these is marked `TODO` at its definition in `constants.py`.
None of them are guesses that matter for whether the *code* is correct --
they're guesses about the *robot* that this code hasn't met yet:

- Every motor's inversion direction (`*_INVERTED` constants) -- verify with
  the robot up on blocks before ever driving on the ground.
- Every limit switch's and beam break's wiring polarity (normally-open vs.
  normally-closed) -- a wrong guess here means `is_at_home()` or
  `is_at_top()` reports the opposite of reality, which is dangerous for a
  mechanism that relies on that switch to stop itself.
- `DriveTrainConstants.DRIVE_DISTANCE_KP/KI/KD` and `TURN_KP/KI/KD` -- both
  PID commands' gains are starting points, not measured. Tune distance
  first (drive a known distance, watch overshoot/settling), then heading.
- The shooter's gear ratio (assumed 1.0, direct-drive from the Kraken X60)
  and its `TARGET_RPM`/PID gains -- these are starting points, not
  measurements.
- The elevator's raise/lower duty cycles (`RAISE_SPEED`/`LOWER_SPEED`) --
  tuned by feel once the real spring/rope mechanism exists, not calculated.
- `VisionConstants.CAMERA_NAME = "Front_Camera"` -- must match whatever name
  the camera is actually configured with in the PhotonVision coprocessor
  web UI; a mismatch here means `PhotonCamera` silently never finds a live
  camera, since the constructor doesn't fail on an unknown name.
- The camera's mounting measurements themselves (3 inches back from the
  front, 1 foot up, 15 degrees up, centered left/right) came from the task
  description, not a tape measure on the actual robot -- `ROBOT_TO_CAMERA`
  in `constants.py` should be re-measured against the real mount once one
  exists, since even a small mounting error compounds into a real pose
  error at range (see [Vision and AprilTags](#vision-and-apriltags)'s
  explanation of why farther single-tag readings get less trust).
- `ApproachTagCommand` reuses `DriveTrainConstants`' existing turn/distance
  PID gains rather than its own -- those gains were themselves only
  starting points (see above), and driving toward a vision-derived target
  point may want tighter or looser tolerances than driving a
  human-specified distance/heading in autonomous. Not split out into
  separate constants yet because there's no bench data yet to justify
  different numbers.
- `VisionConstants.MAX_TAG_DISTANCE_METERS`, `MAX_AMBIGUITY`, and the three
  `*_STDDEVS` tuples are reasonable-sounding starting points for a
  PhotonVision AprilTag pipeline, not numbers measured against this
  specific camera/lens/coprocessor combination -- expect to retune all of
  them once real vision data exists to compare against a known ground-truth
  position.

## Using this as a teaching curriculum

This codebase is meant to be read start-to-finish, not just run. A
suggested order for a rookie who knows Python but not FRC:

1. **`constants.py`** first -- no logic, just names and numbers, and every
   comment explains a physical fact about the robot. This is the
   vocabulary for everything else.
2. **`subsystems/gripper.py`** + **`commands/gripper_commands.py`** next --
   the simplest subsystem/command pair (one motor, two commands, no
   sensors). Good first "explain this back to a mentor" exercise, and the
   clearest possible look at the subsystem/command split itself.
3. **`subsystems/elevator.py`** + **`commands/elevator_commands.py`** --
   adds limit switches and the idea of a command that has to protect
   hardware (auto-stop at a limit) even while the operator is actively
   commanding it.
4. **`subsystems/trigger.py`** + **`commands/trigger_commands.py`** -- the
   one genuinely tricky piece of logic in this codebase (`FireCommand`'s
   edge-detection state machine). Worth a mentor walking through it on a
   whiteboard before reading the code, and worth comparing against the
   factory-method version quoted in
   [Where do commands live?](#where-do-commands-live).
5. **`subsystems/shooter.py`** + **`commands/shooter_commands.py`** --
   introduces closed-loop control (`VelocityVoltage`) and a command with no
   `execute()` at all, contrasted with everything read so far.
6. **`subsystems/drivetrain.py`** + **`commands/drivetrain_commands.py`** --
   the most hardware (4 motors + a gyro) and all four of its commands,
   including the two PID ones, plus this branch's addition: kinematics and
   odometry (see [Odometry and kinematics](#odometry-and-kinematics)). Read
   last since it's the longest pair of files, once PID itself and the
   smaller command patterns are both familiar.
7. **`subsystems/vision.py`** + **`commands/vision_commands.py`** -- the
   newest material, and the first genuinely cross-subsystem command in the
   codebase (see
   [Where do commands live?](#where-do-commands-live)'s closing section).
   Read this only after step 6, since `ApproachTagCommand` builds directly
   on `DriveTrain`'s pose estimate and its existing PID gains. Good
   discussion questions: why does `ApproachTagCommand` call
   `addRequirements(drivetrain)` but not `vision`? Why is it one
   parameterized class instead of two? What happens if the tag ID passed in
   doesn't exist on the field?
8. **`robotcontainer.py`** -- ties everything together; should now read as
   "the map of the whole robot" rather than new material.
9. **`tests/`** -- write one new test for a method that doesn't have one
   yet. Forces reading a subsystem or command closely enough to predict
   its behavior, with no hardware risk at all.

Two mentors already know Python; the gap to close is FRC-specific:
`commands2`'s command lifecycle (`initialize`/`execute`/`isFinished`/`end`,
and what `cmd.run()`/`cmd.sequence()`/decorators like `.withTimeout()`
build on top of it even for a factory-style command elsewhere in the
codebase), what a `Subsystem`'s `periodic()` is for, PID control in
general (a genuinely new idea to most rookies, regardless of language), and
the vendor-library seam noted in
[Naming and numbering conventions](#naming-and-numbering-conventions).
`python -m robotpy sim` is worth introducing on day one -- it starts in
seconds, so a rookie can change one line and immediately see the result,
without ever touching real hardware.
