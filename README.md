# Teaching-Bot (Java): Proof of Concept

This is the **Java sibling** of `teaching-bot-poc` (the Python branch of this same
repo), built specifically so a team debating Java vs. Python can put the same robot,
built to the same design rules, up side by side and compare the languages directly --
not a language comparison tangled up with a design-philosophy comparison. Every
decision documented in the Python README's "Design decisions" section was
deliberately kept identical here: the same subsystems, the same explicit-class
commands with no lambdas, the same physical robot, the same controller bindings, the
same autonomous routines, the same TODO-marked unverified constants. Only the
language, and the syntax each language forces onto the same ideas, differs.

**Read this before anything else: this project has not been compiled or run.** See
[Verification status](#verification-status) below for exactly why, and exactly what
that does and doesn't mean for trusting this code.

## Contents

- [Verification status](#verification-status)
- [Physical specs](#physical-specs)
- [File structure](#file-structure)
- [Where do commands live?](#where-do-commands-live)
- [Naming and numbering conventions](#naming-and-numbering-conventions)
- [Java vs. Python, line by line](#java-vs-python-line-by-line)
- [Subsystems](#subsystems)
- [Commands](#commands)
- [Controller bindings](#controller-bindings)
- [Autonomous](#autonomous)
- [Running tests](#running-tests)
- [Building and running this project](#building-and-running-this-project)
- [Design decisions and deliberate simplifications](#design-decisions-and-deliberate-simplifications)
- [Assumptions that need bench verification](#assumptions-that-need-bench-verification)
- [Using this as a teaching curriculum](#using-this-as-a-teaching-curriculum)

## Verification status

**This code has never been compiled, run, or tested.** It was written in a sandboxed
session whose network policy blocks every Maven repository a WPILib Java project
needs to even download its own dependencies (`frcmaven.wpi.edu`, REVLib's, CTRE's,
and Studica's Maven hosts all returned policy-denied 403s when a build was attempted)
-- not a code problem, a network policy one, confirmed directly before any of this
project's source was written. There was no way to run `./gradlew build` or
`./gradlew test` and see real output, the way the Python sibling's README can report
an actual, reproduced "14 passed" from `python -m robotpy test`.

What that means concretely:

- Every WPILib/REVLib/Phoenix6/Studica class name, method name, and argument order
  used below was cross-checked against the REAL competition port
  (`2026_competition_code`, this team's actual 2026 Java robot code) rather than
  written from memory -- `DriveTrain.java` and `Shooter.java` in particular were read
  directly from that repo first, specifically to get REVLib's `SparkMaxConfig`
  pattern and Phoenix 6's `TalonFXConfiguration` pattern exactly right. Where
  something couldn't be cross-checked that way (see below), it's called out
  explicitly rather than presented as equally certain.
- One specific gap: the JUnit tests poke REVLib's SparkMax encoder simulation
  directly via `RelativeEncoder.setPosition(...)`, the same real, documented method
  `DriveTrain`'s own constructor already uses to zero the encoders at startup -- not
  a guessed simulation backdoor. An earlier draft of this file's tests instead
  guessed at a `SimDeviceSim` device name/key for REVLib's simulated encoder, which
  could not be verified in this session and has been removed in favor of that safer,
  confirmed approach (see [Running tests](#running-tests) for the small, deliberate
  visibility change that required).
- **Before trusting this for a real side-by-side comparison, run
  `./gradlew build test` yourself** (see
  [Building and running this project](#building-and-running-this-project) for the one
  known gap -- a missing binary wrapper jar -- and how to fix it in one command) and
  treat any compile error you find as a real bug report, not a surprise. This is
  exactly the kind of code an independent build should verify before anyone relies on
  the comparison it's meant to support.

## Physical specs

Identical to the Python sibling -- this is the same robot, on paper:

- **Drivetrain:** 6-wheel "drop center" differential (tank) drive -- 3 wheels per
  side, the center wheel mounted 1/4" LOWER than the front/back wheels, so only the
  center wheel plus one end (front or back) actually touches the ground at a time,
  not all three. That shortens the effective ground-contact wheelbase and reduces
  turning friction compared to a 6-wheel-flat drivetrain, while keeping 6-wheel
  traction/durability for driving straight. 2x NEO 2.0 motors per side (4 total)
  through REV SparkMax controllers, 8.4:1 gearing. 6" diameter, 1"-wide wheels; 13"
  between wheel centers front-to-back per side; 23" between the left and right wheel
  centerlines.
- **Frame:** 32" front-to-back, 28" side-to-side, 118 lb with bumpers.
- **Shooter:** single 5 lb flywheel, 4x 4" compliant wheels as the shooting surface,
  driven by one Kraken X60 (TalonFX) motor.
- **Trigger:** small NEO-driven cam that flicks a game piece into the shooter. One
  limit switch defines the cam's rest ("home") position; two beam-break sensors
  report loading status upstream.
- **Elevator:** 2-stage single-mast cascade elevator (AndyMark "Elevator in a Box"
  style), spring-assisted extension, rope retraction via a geared Redline motor
  (brushed, no encoder). A motor-driven roller gripper sits at the end.

## File structure

```
teaching-bot-java/
├── build.gradle                 # GradleRIO plugin, WPILib + vendor dependencies, JUnit
├── settings.gradle               # Standard GradleRIO plugin-repository setup
├── gradlew, gradlew.bat           # Gradle wrapper scripts (see Building, below, for the wrapper jar gap)
├── gradle/wrapper/                # gradle-wrapper.properties (pins the Gradle version)
├── vendordeps/                    # REVLib, Studica (navX), Phoenix 6, WPILib-New-Commands
├── .wpilib/wpilib_preferences.json
└── src/
    ├── main/java/frc/robot/
    │   ├── Main.java              # Entry point -- do not modify
    │   ├── Robot.java             # extends TimedCommandRobot
    │   ├── RobotContainer.java    # Wires subsystems, bindings, and autonomous together
    │   ├── Constants.java          # All numeric/boolean constants, one nested class per subsystem
    │   ├── subsystems/
    │   │   ├── DriveTrain.java
    │   │   ├── Shooter.java
    │   │   ├── Trigger.java
    │   │   ├── Elevator.java
    │   │   └── Gripper.java
    │   ├── commands/                # Every Command, as an explicit class -- see below
    │   │   ├── TeleopDriveCommand.java, ResetGyroCommand.java, DriveDistanceCommand.java, TurnToAngleCommand.java
    │   │   ├── SpinUpShooterCommand.java
    │   │   ├── FireCommand.java
    │   │   ├── RaiseElevatorCommand.java, LowerElevatorCommand.java
    │   │   └── IntakeCommand.java, EjectCommand.java
    │   └── autonomous/
    │       ├── AutoRoutines.java   # The two autonomous routines (built from commands/)
    │       └── AutoChooser.java    # Builds the SmartDashboard auto chooser
    └── test/java/frc/robot/         # JUnit 5 test suite -- see "Running tests"
```

Notice this is a slightly deeper directory tree than the Python sibling's flat
`commands/`/`subsystems/` folders -- Java's package system requires a directory
layout that mirrors the package declaration at the top of every file
(`package frc.robot.subsystems;` means the file must live in
`.../frc/robot/subsystems/`), where Python's import system has no such requirement
(Python's `commands/drivetrain_commands.py` and `subsystems/drivetrain.py` are just
files in folders that happen to also be Python packages via an `__init__.py`, but
nothing forces the folder name to match anything inside the file).

## Where do commands live?

Identical answer, and identical reasoning, to the Python sibling: every command in
this project -- teleop driving, both autonomous PID commands, firing the trigger,
raising/lowering the elevator, running the gripper -- is its own explicit class in
`commands/`, subclassing `Command` and overriding whichever of `initialize()`/
`execute()`/`isFinished()`/`end(boolean interrupted)` it actually needs. Subsystems
only expose plain, one-shot hardware actions (`drive()`, `stop()`,
`getLeftDistanceMeters()`, ...) -- nothing that returns a `Command`.

The alternative this project deliberately did NOT take -- and the one most WPILib
Java example code (including this team's own real competition port) actually uses --
is a **factory-method** style: a subsystem method like
`Command driveDistanceCommand(double feet)` that builds and returns a command in one
expression using `Commands.run(...)`, `.until(...)`, `.finallyDo(...)`, and similar
composition helpers. Compare this project's `commands/FireCommand.java` to what it
would look like as a factory method instead:

```java
// The factory-method version this replaced -- functionally identical, but its one
// bit of state has to be captured by a lambda closing over a local variable
// (Java requires that captured variable to be "effectively final," which rules out
// a plain local boolean the lambda could reassign -- an AtomicBoolean or a one-
// element array is the usual workaround), and "when does this run" is implicit in
// how .until()/.finallyDo() are composed rather than four named methods.
public Command fireCommand() {
    var hasLeftHome = new java.util.concurrent.atomic.AtomicBoolean(false);
    return Commands.runOnce(() -> hasLeftHome.set(false), this)
        .andThen(Commands.run(this::runCam, this)
            .until(() -> {
                if (!hasLeftHome.get()) {
                    if (!isAtHome()) {
                        hasLeftHome.set(true);
                    }
                    return false;
                }
                return isAtHome();
            }))
        .finallyDo(interrupted -> stopCam());
}
```

Both versions do the same thing. The explicit-class version needs no
`AtomicBoolean` workaround for shared mutable state (a plain `private boolean` field
is enough, since it's a real field on a real object, not a variable captured by a
closure), its lifecycle methods are separately readable, and each one is a plain
method that can be stepped through in a debugger without also having to understand
lambda captures and command-decorator composition at the same time. The cost is more
boilerplate per command (a constructor and an `addRequirements()` call every class
needs). For a rookie-facing codebase, that trade is worth it in Java for exactly the
same reason the Python README gives for Python -- if anything, the trade is a little
MORE worth it in Java, since Java's "effectively final" capture rule makes the
lambda version's shared mutable state noticeably more awkward than Python's
equivalent (Python closures can freely reassign an enclosing variable with
`nonlocal`, no atomic wrapper required).

**No lambdas or inline commands anywhere in this project, including one-shot
actions** -- the same rule, enforced the same way, as the Python sibling.
`ResetGyroCommand.java` is a full class whose `initialize()` does the work and whose
`isFinished()` returns `true` immediately, rather than the shorter (but lambda-based)
`Commands.runOnce(drivetrain::resetGyro, drivetrain)`.

## Naming and numbering conventions

Same FRC-wide and team conventions as the Python sibling and the real competition
port (CAN IDs grouped by subsystem in tens, constants in one file/class, one
file/class per subsystem, `<Verb><Noun>Command` naming, SI units internally with
imperial only at input/output boundaries, positive rotation = counterclockwise). Two
things are different specifically because Java is a different language:

- **Java naming (not PEP 8):** classes `PascalCase` (matches Python), methods and
  fields `camelCase` (Python: `snake_case`), constants `ALL_CAPS_WITH_UNDERSCORES`
  (matches Python), packages all-lowercase (`frc.robot.subsystems`; Python:
  `subsystems`, no case convention needed since Python packages are just folder
  names). A rookie moving between this project and its Python sibling will see
  `getLeftDistanceMeters()` here and `get_left_distance_meters()` there for the
  exact same method -- same words, different capitalization convention, nothing
  more.
- **No separate "vendor library uses different casing" seam.** The Python README
  calls out, as a real seam worth noticing, that Python code in that project calls
  `self._left_encoder.getPosition()` -- snake_case code calling a camelCase REVLib
  method, because REVLib's Python bindings were never rewritten to match PEP 8. In
  Java, there is no seam to notice: this project's own code and every vendor
  library's code use the exact same camelCase convention, because Java has only ever
  had one dominant naming convention across its whole ecosystem. `phoenix6`'s Java
  API is `com.ctre.phoenix6.hardware.TalonFX`, same camelCase as everything else --
  contrast the Python README's note that `phoenix6`'s Python bindings are the ONE
  Python library in that project that already ships native snake_case, independent
  of everything else. Genuinely nothing to teach here in Java that isn't already
  covered by "Java is camelCase, always."

## Java vs. Python, line by line

This section exists only in the Java branch -- its whole purpose is calling out where
the two languages actually differ, syntax-for-syntax, on the exact same lines of
code. See each command class's own comments for the fullest version of this (starting
with `commands/TeleopDriveCommand.java`'s constructor, which walks through every
piece individually), but the short version:

| | Java | Python |
|---|---|---|
| Type on a parameter/field | Required, before the name: `DriveTrain drivetrain` | Optional, after the name with a colon: `drivetrain: DriveTrain` |
| Enforced by | The compiler, always | Nothing at runtime; only external tools if a hint is present |
| "Private" | `private` keyword, compiler-enforced | `_leadingUnderscore` naming convention only |
| A method returning nothing | `void methodName()` | `def method_name() -> None:` (the `-> None` is optional) |
| A constructor | No return type at all -- not even `void` is legal | `def __init__(self, ...) -> None:` -- an ordinary method that happens to always return `None` |
| Calling the parent constructor | Automatic if omitted (calls the no-arg parent constructor) | Never automatic -- always an explicit `super().__init__()` |
| The current object | `this` (often omittable; required here only when a field and parameter share a name) | `self` (always an explicit first parameter, always required to access anything on the instance) |
| A file of related functions with no class | Not possible -- must be `static` methods on some class (see `AutoRoutines.java`) | A plain `.py` file of top-level `def`s (see `autonomous/routines.py`) |
| A "constant" | `public static final TYPE NAME = value;` -- three keywords doing three separate jobs | `NAME = value` as a class attribute -- constant only by convention |

## Subsystems

| Subsystem | File | Purpose | Hardware | Key methods |
|---|---|---|---|---|
| **DriveTrain** | `subsystems/DriveTrain.java` | Moves the robot (tank/differential drive) | 4x REV SparkMax NEO 2.0 (CAN 20-23), navX2 gyro (SPI/MXP) | `drive()`, `stop()`, `getLeft/RightDistanceMeters()`, `getLeft/RightVelocityMetersPerSecond()`, `getHeadingDegrees()`, `resetGyro()`, `resetEncoders()` |
| **Shooter** | `subsystems/Shooter.java` | Spins the flywheel to a fixed target RPM | 1x Kraken/TalonFX (CAN 30) | `setTargetRpm()`, `stop()`, `getCurrentRpm()`, `isAtTargetSpeed()` |
| **Trigger** | `subsystems/Trigger.java` | Fires one game piece into the shooter per cam revolution | 1x NEO/SparkMax (CAN 31), 1 limit switch (DIO 0), 2 beam breaks (DIO 1-2) | `runCam()`, `stopCam()`, `isAtHome()`, `hasBallAtStage1()`, `hasBallAtStage2()` |
| **Elevator** | `subsystems/Elevator.java` | Raises/lowers the 2-stage mast | 1x geared Redline/SparkMax, brushed, no encoder (CAN 40), top/bottom limit switches (DIO 3-4) | `setSpeed()`, `stop()`, `isAtTop()`, `isAtBottom()` |
| **Gripper** | `subsystems/Gripper.java` | Spinning roller intake at the end of the elevator | 1x NEO 550/SparkMax (CAN 41) | `setSpeed()`, `stop()` |

Notice none of these have a method that returns a `Command` -- that's the point of
the split described in [Where do commands live?](#where-do-commands-live).

## Commands

| Command | Subsystem | Bound as | What it does |
|---|---|---|---|
| `TeleopDriveCommand` | DriveTrain | Default command | Tank drive read directly from the driver controller's two joystick Y-axes |
| `ResetGyroCommand` | DriveTrain | `onTrue` | One-shot: resets the navX heading to 0 |
| `DriveDistanceCommand(feet)` | DriveTrain | Used in autonomous | PID-drives straight to a target distance, in feet |
| `TurnToAngleCommand(degrees)` | DriveTrain | Used in autonomous | PID-turns to an absolute heading using the navX gyro |
| `SpinUpShooterCommand` | Shooter | `toggleOnTrue` | Toggle: commands the flywheel to a fixed target RPM, or stops it |
| `FireCommand` | Trigger | `onTrue`, with `.withTimeout(...)` | Runs the cam motor until it leaves and then returns to the home (limit-switch) position |
| `RaiseElevatorCommand` / `LowerElevatorCommand` | Elevator | `whileTrue` | Drives the lift motor while held, auto-stopping at the relevant limit switch |
| `IntakeCommand` / `EjectCommand` | Gripper | `whileTrue` | Spins the roller in/out while held |

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

| Option | What it does |
|---|---|
| **Do Nothing** (default) | `Commands.none()` -- the robot does nothing during autonomous |
| **Drive Forward 10 ft** | `AutoRoutines.driveForwardOnly(drivetrain)`, straight |
| **Drive 5ft, Turn Left 90, Drive 3ft** | `AutoRoutines.driveTurnDrive(drivetrain)` -- a `Commands.sequence(...)` of the two PID commands with a turn in between |

## Running tests

```
./gradlew test
```

Five files in `src/test/java/frc/robot/`, mirroring the Python sibling's `tests/`
one-for-one:

| Java | Python | Covers |
|---|---|---|
| `RobotLifecycleTest.java` | `pyfrc_test.py` + `test_robot_lifecycle.py` | Full disabled → autonomous → teleop mode cycle, no exceptions |
| `subsystems/DriveTrainTest.java` | `test_drivetrain.py` | Encoder bookkeeping, both PID commands reaching their setpoint, autonomous routines building without error |
| `ElevatorTest.java` | `test_elevator.py` | Limit-switch-gated raise/lower commands |
| `TriggerTest.java` | `test_trigger.py` | `FireCommand`'s edge-detection state machine, including its timeout |

**Java has no equivalent of pyfrc's pytest plugin.** The Python sibling's tests get a
ready-made `robot`/`control` fixture pair for free (`control.step_timing(...)`
advances simulated time AND flips the enabled/autonomous flags in one call); nothing
bundled with GradleRIO does the same. Every Java test file here does that setup
explicitly instead, in its own `@BeforeEach`/`@AfterEach`, using `HAL.initialize()`
to boot the simulated hardware layer, `DriverStationSim` to set the enabled/autonomous
flags a real driver station would set, and `SimHooks.stepTiming()` to advance
simulated time and run the CommandScheduler's periodic loop -- consistent with this
whole project's "explicit over implicit" rule, and a legitimate example of Python's
testing ecosystem providing more out of the box than Java's does here.

**A real design change this branch made, worth reading as its own lesson:**
`subsystems/DriveTrain.java`'s `leftEncoder`/`rightEncoder` fields are
**package-private** (no access modifier at all), not `private` like every other
hardware field in this project. `DriveTrainTest.java` needs to poke their simulated
position directly with `.setPosition(...)` to test `DriveDistanceCommand`/
`TurnToAngleCommand` without a real robot. The Python sibling's equivalent test does
the same poke by reaching straight past its `_left_encoder`'s leading-underscore
naming convention -- Python has no enforced privacy to get past in the first place, so
nothing about its test file needed to change to make that possible. Java's `private`
is compiler-enforced with no such loophole, so getting the same test access for real
needed an actual, coarser access level (package-private: visible to any class in
`frc.robot.subsystems`, invisible everywhere else) instead of a naming hint a test
could just ignore. This is a genuine, structural difference between the languages,
not a translation artifact -- worth a rookie sitting with both files side by side.

## Building and running this project

```
./gradlew build   # compile + run tests
./gradlew simulateJava   # run in WPILib's desktop simulator
```

**One known gap:** `gradle/wrapper/gradle-wrapper.jar` -- the small binary file that
makes `./gradlew` work without a separately-installed Gradle -- is **not included**
in this branch. It's a compiled binary, and the tooling used to write this project
could only commit text files. Before running `./gradlew` for the first time, generate
it with either:

```
gradle wrapper --gradle-version 8.11   # if you have Gradle installed some other way
```

or open this folder in VS Code with the WPILib extension installed and let it manage
the wrapper for you (the extension bundles its own Gradle/JDK and doesn't strictly
need `./gradlew` to work), or copy `gradle/wrapper/gradle-wrapper.jar` from any other
2026 GradleRIO project you already have checked out (this team's own
`2026_competition_code` repo has one).

## Design decisions and deliberate simplifications

Identical list to the Python sibling -- see that branch's README for the full
reasoning behind each of these, which applies here unchanged:

- Every command is an explicit class, not a factory method (see
  [Where do commands live?](#where-do-commands-live) above for the Java-specific
  side-by-side comparison).
- `DriveDistanceCommand` is PID, not "drive at a fixed speed until the encoder says
  stop," with its output independently clamped as a safety margin on top of a
  conservative `KP`.
- Elevator is entirely open-loop -- no encoder exists to close a position loop
  against, only two limit switches.
- Trigger's "fire" is an edge-detection state machine, not a single sensor read, since
  the cam starts every cycle already at its one "home" sensor.
- Every command has the same four-method shape, but not every command needs all four
  -- `SpinUpShooterCommand` has no `execute()` because Phoenix 6's velocity control is
  closed-loop on the TalonFX itself.

## Assumptions that need bench verification

Identical set of TODO-marked constants to the Python sibling, all in `Constants.java`
-- motor inversions, limit-switch/beam-break polarity, every PID gain, the shooter's
gear ratio. None of these are guesses about whether the CODE is correct; they're
guesses about the ROBOT this code hasn't met yet. See the Python README's own section
of the same name for the full list and why each one matters.

## Using this as a teaching curriculum

Same suggested reading order as the Python sibling (`Constants.java` first, then
`Gripper`/`GripperCommands` as the simplest pair, then `Elevator`, `Trigger`,
`Shooter`, `DriveTrain` last, then `RobotContainer.java`, then `src/test/`) -- with
one addition specific to this branch: **read the same file in both languages back to
back.** `commands/TeleopDriveCommand.java`'s constructor here and
`commands/drivetrain_commands.py`'s `TeleopDriveCommand.__init__` there do the
identical job, three lines of real logic each, and the differences between them (type
placement, `this`/`self`, `private`/underscore, the constructor return-type rule) are
exactly the differences a team weighing Java vs. Python for next season needs to see
next to each other, not described in the abstract.
