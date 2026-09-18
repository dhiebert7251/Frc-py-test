# Teaching-Bot Proof of Concept

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
- [Naming and numbering conventions](#naming-and-numbering-conventions)
- [Subsystems](#subsystems)
- [Commands](#commands)
- [Controller bindings](#controller-bindings)
- [Autonomous](#autonomous)
- [Running tests](#running-tests)
- [Simulating (`physics.py`)](#simulating-physicspy)
- [Design decisions and deliberate simplifications](#design-decisions-and-deliberate-simplifications)
- [Assumptions that need bench verification](#assumptions-that-need-bench-verification)
- [Using this as a teaching curriculum](#using-this-as-a-teaching-curriculum)

## Physical specs

- **Drivetrain:** 6-wheel "drop center" differential (tank) drive -- 3
  wheels per side, the center wheel mounted 1/4" higher so it only touches
  the ground once the frame flexes under load. 2x NEO 2.0 motors per side
  (4 total) through REV SparkMax controllers, 8.4:1 gearing. 6" diameter,
  1"-wide wheels; 13" between wheel centers front-to-back per side; 23"
  between the left and right wheel centerlines.
- **Frame:** 32" front-to-back, 28" side-to-side, 118 lb with bumpers.
- **Shooter:** single 5 lb flywheel, 4x 4" compliant wheels as the shooting
  surface, driven by one Kraken (TalonFX) motor.
- **Trigger:** small NEO-driven cam that flicks a game piece into the
  shooter. One limit switch defines the cam's rest ("home") position; two
  beam-break sensors report loading status upstream.
- **Elevator:** 2-stage single-mast cascade elevator (AndyMark "Elevator in
  a Box" style), spring-assisted extension, rope retraction via a geared
  Redline motor (brushed, no encoder). A motor-driven roller gripper sits at
  the end.

## File structure

```
teaching-bot/
├── robot.py                 # Entry point -- commands2.TimedCommandRobot
├── robotcontainer.py         # Wires subsystems, bindings, and autonomous together
├── constants.py              # All numeric/boolean constants, one class per subsystem
├── physics.py                 # `python -m robotpy sim` physics model (drivetrain only)
├── pyproject.toml             # RobotPy project + vendor dependency versions
├── subsystems/
│   ├── drivetrain.py          # DriveTrain
│   ├── shooter.py             # Shooter
│   ├── trigger.py             # Trigger
│   ├── elevator.py            # Elevator
│   └── gripper.py             # Gripper
├── autonomous/
│   ├── routines.py            # The two autonomous routines
│   └── chooser.py             # Builds the SmartDashboard auto chooser
├── tests/                      # `python -m robotpy test` suite
└── .wpilib/wpilib_preferences.json
```

Notably **absent** compared to the competition bot: no `commands/` package
(no composite/cross-subsystem commands are needed here -- see
[Design decisions](#design-decisions-and-deliberate-simplifications)), no
`vision_measurement.py`/`result.py` (no vision on this robot), and no
`deploy/pathplanner/` (no PathPlanner -- autonomous is plain encoder/gyro
code).

## Naming and numbering conventions

Same conventions as the competition bot's README, reused here so a
student moving between the two codebases doesn't have to relearn anything.

### FRC-wide conventions

- **SI units internally, imperial at the boundary.** WPILib math
  (`wpimath`) works in meters/radians/seconds even though FRC parts are
  specified in inches and degrees. Conversions happen once, right where a
  human-facing constant becomes a WPILib value -- see every
  `* 0.0254` (inches to meters) in `constants.py`.
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
- **One file per subsystem, filename matches the class it defines, in
  lowercase:** `DriveTrain` -> `drivetrain.py`, `Shooter` -> `shooter.py`.
- **Command-returning methods on a subsystem end in `_command`:**
  `teleop_drive_command()`, `fire_command()`, `spin_up_command()`,
  `intake_command()`. Grepping for `_command` finds every command a
  subsystem offers.
- **Private/internal state is `_`-prefixed:** `_target_rpm`,
  `_telemetry_loop_counter`, etc.

### Python naming (PEP 8)

| | Convention | Example |
|---|---|---|
| Functions/methods/variables | `snake_case` | `get_left_distance_meters()` |
| Classes | `PascalCase` | `DriveTrain` |
| Constants | `ALL_CAPS_WITH_UNDERSCORES` | `SHOOTER_MOTOR_ID` |
| Files | `snake_case.py`, one module per file | `drivetrain.py` |

**The one thing that trips people up:** our own code follows the table
above, but every call *into* a vendor library (`wpilib`, `commands2`,
`rev`) still uses that library's own naming, which is Java-style
`camelCase` as of the 2026 season (`phoenix6`, used here for the Kraken
shooter motor, is the one exception -- it's already native `snake_case`).
So a single line can legitimately mix both:
`self._left_encoder.getPosition()` -- our snake_case variable, calling a
camelCase method from REV's library. This isn't a mistake to fix; it's a
real seam between our code and code we don't own, worth pointing out
explicitly the first time a rookie notices it.

## Subsystems

| Subsystem | File | Purpose | Hardware | Key methods |
|---|---|---|---|---|
| **DriveTrain** | `subsystems/drivetrain.py` | Moves the robot (tank/differential drive) | 4x REV SparkMax NEO 2.0 (CAN 20-23), navX2 gyro (SPI/MXP) | `drive()`, `teleop_drive_command()`, `drive_distance_command()`, `turn_to_angle_command()`, `get_heading_degrees()`, `reset_gyro()` |
| **Shooter** | `subsystems/shooter.py` | Spins the flywheel to a fixed target RPM | 1x Kraken/TalonFX (CAN 30) | `spin_up_command()`, `set_target_rpm()`, `stop()`, `get_current_rpm()`, `is_at_target_speed()` |
| **Trigger** | `subsystems/trigger.py` | Fires one game piece into the shooter per cam revolution | 1x NEO/SparkMax (CAN 31), 1 limit switch (DIO 0), 2 beam breaks (DIO 1-2) | `fire_command()`, `is_at_home()`, `has_ball_at_stage_1()`, `has_ball_at_stage_2()` |
| **Elevator** | `subsystems/elevator.py` | Raises/lowers the 2-stage mast | 1x geared Redline/SparkMax, brushed, no encoder (CAN 40), top/bottom limit switches (DIO 3-4) | `raise_command()`, `lower_command()`, `is_at_top()`, `is_at_bottom()` |
| **Gripper** | `subsystems/gripper.py` | Spinning roller intake at the end of the elevator | 1x NEO 550/SparkMax (CAN 41) | `intake_command()`, `eject_command()`, `stop()` |

## Commands

| Command | Subsystem | Pattern | What it does |
|---|---|---|---|
| `teleop_drive_command()` | DriveTrain | `cmd.run()` (default command) | Tank drive from the driver's two joystick Y-axes |
| `drive_distance_command(meters)` | DriveTrain | `cmd.sequence(...).finallyDo(...)` | Drives straight at a constant speed until the average encoder distance reaches the target, then stops |
| `turn_to_angle_command(degrees)` | DriveTrain | `cmd.run().until().finallyDo()` | PID-turns to an absolute heading using the navX gyro |
| `spin_up_command()` | Shooter | `cmd.startEnd()` | Toggle: commands a fixed target RPM once, or coasts to a stop once -- see [why `startEnd` here](#design-decisions-and-deliberate-simplifications) |
| `fire_command()` | Trigger | `cmd.sequence(...).withTimeout(...).finallyDo(...)` | Runs the cam motor until it leaves and then returns to the home (limit-switch) position; safety-timed out in case the switch never re-triggers |
| `raise_command()` / `lower_command()` | Elevator | `cmd.run().finallyDo()` | Drives the lift motor while held, auto-stopping at the relevant limit switch even if the button is still held |
| `intake_command()` / `eject_command()` | Gripper | `cmd.run().finallyDo()` | Spins the roller in/out while held |

## Controller bindings

**Driver (port 0) -- drive only:**

| Input | Action |
|---|---|
| Left stick Y | Left-side drive speed |
| Right stick Y | Right-side drive speed |
| Back button | Reset gyro heading to 0 (do this before every autonomous run) |

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
| **Drive Forward 10 ft** | `drivetrain.drive_distance_command()` for 10 feet, straight. |
| **Drive 5ft, Turn Left 90, Drive 3ft** | `drive_distance_command()` for 5 feet, `turn_to_angle_command(90)` (left), then `drive_distance_command()` for 3 feet. |

Both routines use only wheel encoders and the navX gyro -- no PathPlanner,
no vision, no pre-authored paths. All the actual numbers (`10.0`, `5.0`,
`90.0`, `3.0`) live in `constants.Auto`, not buried in `autonomous/routines.py`,
so they're easy to find and change.

## Running tests

```
python -m robotpy test
```

This runs `pyfrc`'s pytest plugin, which provides the `robot`/`control`
fixtures used throughout `tests/`. 13 tests currently cover: the standard
pyfrc smoke tests, a full disabled -> autonomous -> teleop lifecycle cycle,
DriveTrain's encoder bookkeeping and autonomous-command construction,
Trigger's fire-command edge-detection state machine (including its
timeout), and Elevator's limit-switch-gated raise/lower behavior. All 13
currently pass, verified against `robotpy==2026.2.2` and its pinned vendor
dependencies (see `pyproject.toml`).

**A gotcha worth teaching explicitly**, hit while writing `tests/test_trigger.py`
and `tests/test_elevator.py`: a `Command` can only be `.schedule()`d while
the robot is *enabled* (its `runsWhenDisabled()` defaults to `False`). If
you call `command.schedule()` before a test has enabled the robot via
`control.step_timing(..., enabled=True)`, the scheduler silently drops the
command and `isScheduled()` reads `False` forever afterward -- there's no
error, just a command that never runs. Every test in this suite that
schedules a command directly does one small `step_timing(enabled=True)`
step first for exactly this reason.

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

## Design decisions and deliberate simplifications

Each of these is a real choice, made and stated here rather than left for
someone to reverse-engineer later:

- **No odometry / pose estimation.** The competition bot's DriveTrain
  fuses encoders, gyro, and vision into a `DifferentialDrivePoseEstimator`
  with a `Field2d` widget. This teaching bot deliberately stops at raw
  `get_left/right_distance_meters()` and `get_heading_degrees()` -- pose
  estimation is a natural *next* lesson once encoders/gyro/PID are
  comfortable, not a starting one.
- **No `commands/` cross-subsystem package.** Unlike the competition bot's
  `commands/emergency.py`, nothing here needs a command that spans more
  than one subsystem. If a real robot needs, say, "auto-fire once the
  gripper senses a piece and the shooter is at speed," that's exactly the
  kind of composite command that package pattern is for -- a good second
  lesson once single-subsystem commands are solid.
- **`spin_up_command()` uses `cmd.startEnd()`, but almost everything else
  uses `cmd.run().finallyDo()`.** This is intentional, not inconsistent:
  Phoenix 6's velocity control is closed-loop on the TalonFX itself, so the
  command only needs to *tell* it a target once and *tell* it to stop once
  -- there's nothing to redo every 20ms loop. Every open-loop, duty-cycle
  motor (drivetrain teleop, trigger cam, elevator lift, gripper roller)
  *does* need re-commanding every loop, hence `cmd.run()`. Worth walking a
  rookie through both patterns side by side and asking them which one
  `Elevator.raise_command()` should use, and why.
- **Elevator is entirely open-loop.** The Redline motor is brushed with no
  built-in encoder, and no external encoder is installed on this robot, so
  there's no sensor to close a position loop against. Two limit switches
  (top/bottom) are the only feedback, and the commands stop the motor the
  instant either one trips -- even if the operator is still holding the
  button. A real elevator that needs to stop at intermediate heights (not
  just fully up/down) would need an added encoder (a through-bore or
  versa-style external one) and a position PID loop; that's a good example
  of "the code can't do more than the sensors let it."
- **Trigger's "fire" is an edge-detection state machine, not a single
  sensor read.** The cam has exactly one sensor: a limit switch at "home."
  One complete fire cycle means *leaving* home and then *returning* to it
  -- reading the switch once, at the start, would falsely report "done"
  immediately (it starts at home). `fire_command()` tracks a `has_left_home`
  flag with a Python closure (`nonlocal`) to handle this, and has a
  timeout in case the switch never re-triggers (a jam or a broken wire),
  so a hardware fault can't hang the robot in autonomous.

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
- The shooter's gear ratio (assumed 1.0, direct-drive from the Kraken) and
  its `TARGET_RPM`/PID gains -- these are starting points, not measurements.
- Whether the Kraken is an X60 or X44 (not specified) -- irrelevant to this
  code today since `physics.py` doesn't model the shooter, but relevant the
  moment someone adds that simulation.
- The elevator's raise/lower duty cycles (`RAISE_SPEED`/`LOWER_SPEED`) --
  tuned by feel once the real spring/rope mechanism exists, not calculated.

## Using this as a teaching curriculum

This codebase is meant to be read start-to-finish, not just run. A
suggested order for a rookie who knows Python but not FRC:

1. **`constants.py`** first -- no logic, just names and numbers, and every
   comment explains a physical fact about the robot. This is the
   vocabulary for everything else.
2. **`subsystems/gripper.py`** next -- the simplest subsystem (one motor,
   two commands, no sensors). Good first "explain this file back to a
   mentor" exercise.
3. **`subsystems/elevator.py`** -- adds limit switches and the idea of a
   command that has to protect hardware (auto-stop at a limit) even while
   the operator is actively commanding it.
4. **`subsystems/trigger.py`** -- the one genuinely tricky piece of logic
   in this codebase (the edge-detection state machine). Worth a mentor
   walking through it on a whiteboard before reading the code.
5. **`subsystems/shooter.py`** -- introduces closed-loop control
   (`VelocityVoltage`) and the `cmd.startEnd()` pattern, contrasted with
   everything read so far.
6. **`subsystems/drivetrain.py`** -- the most hardware (4 motors + a gyro)
   and the two autonomous-relevant commands. Read last since it's the
   longest file, once the smaller patterns are familiar.
7. **`robotcontainer.py`** -- ties everything together; should now read as
   "the map of the whole robot" rather than new material.
8. **`tests/`** -- write one new test for a method that doesn't have one
   yet. Forces reading a subsystem closely enough to predict its behavior,
   with no hardware risk at all.

Two mentors already know Python; the gap to close is FRC-specific:
`commands2`'s command lifecycle (`initialize`/`execute`/`isFinished`/`end`,
and what `cmd.run()`/`cmd.startEnd()`/`cmd.sequence()` build on top of it),
what a `Subsystem`'s `periodic()` is for, and the vendor-library seam noted
in [Naming and numbering conventions](#naming-and-numbering-conventions).
`python -m robotpy sim` is worth introducing on day one -- it starts in
seconds, so a rookie can change one line and immediately see the result,
without ever touching real hardware.
