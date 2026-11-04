# FRC 5735 Control Freaks

How code, comments and Javadoc are written in this repo. For people and agents alike: follow it for new code and
for any code you touch. Examples are Java (WPILib / `command3`), since that's what this repo is written in.

## Conditionals: explicit `if`, never hidden in an expression

No ternaries (`a ? b : c`), no `x = y != null ? y : default`, and no short-circuit `&&` / `||` used to compute a
value (`boolean done = isDone != null && isDone.getAsBoolean();`). Assign the default, then override in an `if`:

```java
double hoodAngleDegrees = HoodConstants.LOWEST_ANGLE_DEGREES;
if (requestedAngleDegrees != null) {
    hoodAngleDegrees = requestedAngleDegrees;
}

boolean hasHitSetpoint = false;
if (isDone != null) {
    hasHitSetpoint = isDone.getAsBoolean();
}
```

- **Guards in an `if` condition are fine**: `if (goal != null && goal.kind() == LaunchGoal.SCORE)`,
  `if (hallLimit.get() && !isZeroed)`.
- When the override depends on the old value, compute into a local first so the default doesn't clobber it:

  ```java
  double oldAngleDegrees = hood.getHoodAngle();  // snapshot before the default below overwrites it
  double hoodAngleDegrees = HoodConstants.LOWEST_ANGLE_DEGREES;
  if (nudgeUp) {
      hoodAngleDegrees = oldAngleDegrees + NUDGE_DEGREES;  // depends on the real old angle, not the default
  }
  ```

  Assigning the default straight into `hoodAngleDegrees` first, then writing `hoodAngleDegrees = hoodAngleDegrees +
  NUDGE_DEGREES` in the override, would nudge off of `LOWEST_ANGLE_DEGREES` instead of the hood's actual position.
- A plain boolean built from comparisons isn't a conditional and is fine:
  `return errorDegrees <= TurretConstants.TOLERANCE_DEGREES && errorRpm <= LauncherConstants.RPM_TOLERANCE;`.

## Purity and state

- **Prefer methods that return values** over methods with side effects. Small helpers (builders, metrics,
  lookups) return what they make and the caller stores it (`LaunchParams params = LaunchCalculator.calculate(...)`,
  then `this.cachedParams = params;`).
- **Static helpers don't mutate their arguments.** Low-level helpers are `private static` under
  `// ---- low-level helpers (static, no side effects)` and return new state instead of writing into a field:
  `double hoodAngleDegrees = HoodMechanism.interp1(...)`. The one exception is an object that carries its own
  history (a `ProfiledPIDController`, a `LinearFilter`), and its Javadoc says so.
- **State changes stay shallow.** A public method changes state itself or through one level of stateful helper. No
  chains like `periodic -> updateAimSolution -> updateTurretSetpoint -> updateHoodSetpoint`: fold the chain into
  one helper, or compute the decision with a pure helper and assign it in the public method
  (`LaunchParams params = LaunchCalculator.calculate(goal, drivetrain, chassisVelocities, turretPose); this.cachedParams = params;`).
- **Getters don't mutate**, lazily or otherwise. All state is assigned in the constructor.
- **Structured results are records**, not loose doubles, arrays, or maps: `LaunchCalculator.LaunchParams` (with
  `isValid` false when no shot solution exists at the current range).
- **Plain, not functional-looking**: no streams (`.map().filter().reduce()`), and no composed calls `f(g(x))`. Name
  the intermediate:

  ```java
  Pose2d turretPoseField = drivetrain.getEstimatedPosition().transformBy(RobotConstants.ROBOT_TO_TURRET);
  Translation2d turretToTarget = FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER).minus(turretPoseField.getTranslation());
  ```

## Names

- **Methods are verbs**: what they do, not the end state. Prefixes `get set is has compute measure make solve
  update convert check`: `getHoodPosition` (not `hoodPosition`), `computeLaunchParams` (not `launchParams`),
  `isInExclusionZone` (not `inExclusionZone`), `makeHoodMap`.
- **No bare-field access from outside the class.** Use `getX()` / `setX()` methods; multi-value setters take
  arguments in their natural order (`TurretMechanism.setSoftLimits(Angle reverseLimit, Angle forwardLimit)`).
- **Arguments say what they are**: `turretPoseSupplier` (not `poseSupplier`), `exclusionZones` (not `zones`),
  `launchGoal` (not `goal`), `hoodAngleDegrees` (not `angle`).
- **Frames in names.** Suffix poses, translations and velocities with the frame they're expressed in: field frame
  or the robot (chassis) frame — `turretPoseField`, `targetTranslationField`, `chassisVelocitiesRobot`. Methods
  that convert or act in a specific frame say so, matching what's already in the codebase:
  `holdFieldRelative`, `autoDriveRobotRelative`, `toFieldRelative`, `getAngleTurretRel`. Constants pin the frame
  the same way: `REVERSE_LIMIT_BOT_REL`.
- **Units in names.** Prefer WPILib's typed units (`Angle`, `Distance`, `AngularVelocity`) over raw `double`s. When
  a raw `double` is unavoidable, suffix the name with its unit: `LOWEST_ANGLE_DEGREES`, `DYNAMIC_TOLERANCE_DEGREES`,
  `RPM_TOLERANCE`.
- Alliance-dependent field locations go through `FieldConstants.alliance(...)` (e.g.
  `FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER)`), never a hand-written sign flip on red/blue.

## Representations, time and units

- **Durations are seconds, never loop counts**: `RECALC_PERIOD_S = 0.02`, not `_LOOPS = 1`. Elapsed-time checks
  allow half a loop of slack (`elapsed >= timeoutS - 0.5 * LOOP_PERIOD_S`), since a 20 ms periodic loop checked
  against a 3.0 s timeout can land a loop either side of exactly 3.0 s.
- **Constants** are `public static final`, grouped with related constants, each with a comment giving units and
  meaning:

  ```java
  public static final double RPM_TOLERANCE = 35;  // RPM, ~1% of the default 3000 RPM setpoint
  public static final Angle LOWEST_ANGLE = Degrees.of(8.0);  // physical minimum the hood servo can reach
  ```

## Layout

- **Mini blocks.** Separate distinct steps inside a method with a blank line, and put a few-word comment above each
  saying what it does (`// solve for the shot`, `// drive the flywheel to the RPM setpoint`).
- **Formatting** (spacing, wrapping, brace placement) is handled by clang-format (`format.java.sh`, `.clang-format`
  at the repo root, `BlockIndent`/120-column) — don't hand-format against it. A signature that doesn't fit on one
  line wraps with each argument block-indented, as clang-format already produces here:

  ```java
  public static void
  calculate(LaunchGoal goal, DrivetrainMechanism drivetrain, ChassisVelocities robotVel, Pose2d turretPose) {
  ```
- **Line length 120**, comments included (wrap long comments as continuation comment lines).
- **Classes**, top to bottom: fields (each with a short trailing comment), the constructor, the public API, private
  stateful helpers called by public methods, the `private static` pure helpers, then a nested `@Utility Tester`
  class if the mechanism has one.
- **Imports** all at the top of the file, no inline imports. Order, each group separated by a blank line: static
  unit imports (`org.wpilib.units.Units.*`); vendor libraries (`com.ctre.phoenix6.*`); WPILib (`org.wpilib.*`);
  `java.*`; our own packages (`first.*`) last.

## Where things live

- **`*Constants` classes hold robot and mechanism facts only** (gear ratios, PID/FF gains, physical limits). CAN
  IDs and DIO pins live in `IDs.java`; field dimensions and scoring locations live in `FieldConstants.java`.
  Tuning values picked at competition don't get hardcoded into mechanism logic — they go in the matching
  `*Constants` class.
- **Values or algorithms borrowed from elsewhere are references.** Logic taken from another team's public repo or
  write-up (e.g. `LaunchCalculator`'s "Logic based on FRC 6328 and write-ups by the independent developer Oblarg")
  is copied in with a provenance comment; we don't take it on as a dependency.
- **Bring-up and tuning code lives in a nested `Tester`.** A mechanism that needs hands-on tuning gets a nested
  `@Utility public static class Tester implements OpMode`, bound to controller buttons
  (`HoodMechanism.Tester`, `LauncherMechanism.Tester`). Don't extract shared test helpers across mechanisms unasked.

## Docstrings

Docstrings are Javadoc (`/** */`) — the format `javadoc`, VS Code, and IntelliJ all read directly.

1. **Semantic first.** Lead with what the call means and what it's for, then the mechanics. Brief, factual, plain
   words; light on jargon and math (a formula only when it's the clearest statement).
2. **Describe what is, not the journey.** No history, rationale, "previously", or "we changed X because".
3. **Keep usage gotchas and invariants** in the overview: what a caller must know to use it correctly (that
   `holdRobotRel`'s goal is clamped to the turret's functional range automatically, that a `Tester` needs the
   robot disabled before rebinding).
4. **Very brief tags.** Each `@param`, `@return` and `@throws` entry is one sentence. Units and frames go in that
   sentence (`Robot-relative {@link Angle} the turret should hold.`); what `null` means is spelled out.
5. **Define every symbol in the method's own Javadoc**, not only the class's: what a magic number means, the frame
   a value is expressed in, the unit.

### Methods

```java
/**
 * Linear interpolation of xq between (x0, y0) and (x1, y1). Use it to convert between a hood servo setpoint and
 * the physical angle it drives to.
 *
 * @param x0 Input value at the first reference point.
 * @param x1 Input value at the second reference point; must not equal x0.
 * @param y0 Output value at the first reference point.
 * @param y1 Output value at the second reference point.
 * @param xq Input value to interpolate at; need not lie between x0 and x1.
 * @return   Interpolated output value.
 * @throws IllegalArgumentException When x0 equals x1.
 */
public static double interp1(double x0, double x1, double y0, double y1, double xq) {
```

```java
/**
 * Runs the turret to a specified, moving {@link Angle}.
 *
 * @param goal Robot-relative {@link Angle}; clamped to the turret's functional range automatically.
 * @return {@link Command} that sets the {@link ProfiledPIDController}'s goal to the parameter, then repeatedly
 *         applies its output to the motor.
 */
public Command holdRobotRel(Angle goal) {
```

- The overview is first, on the line after the opening `/**`.
- `@param` lists every argument, as one sentence. Leave the tag out entirely when there are none.
- `@return` is one sentence, left out when the method returns `void`.
- `@throws ErrorType When it's thrown.` for each exception thrown on purpose, left out when nothing is thrown.
- Classes: the class Javadoc is the overview of what an instance is and how it's used. Records document their
  fields with `@param` on the record declaration itself (one sentence each, same as a method argument):

  ```java
  /**
   * Solution to a single shot: the turret, hood and flywheel setpoints needed to hit the target.
   *
   * @param isValid          False when the target is out of range; the other fields are then meaningless.
   * @param turretAngle      Turret angle, robot-relative.
   * @param turretVelocity   Turret angular velocity to track while driving.
   * @param hoodAngle        Hood angle.
   * @param flywheelVelocity Flywheel angular velocity.
   */
  public record LaunchParams(
      boolean isValid, Angle turretAngle, AngularVelocity turretVelocity, Angle hoodAngle, AngularVelocity flywheelVelocity
  ) {}
  ```

## Comments

- Same rules as Javadoc: what the code does and what a reader must know, not how it got that way. No history, no
  "fixed", "new", "was X".
- One short comment per mini block; trailing comments for fields and non-obvious lines (units, frames, what `null`
  means). Gotchas (a Phoenix6 firmware quirk, a CAN bus ordering requirement) belong in comments.
- Delete stale comments. When filling in code the user started, keep their comment lines.

## Docs

README entries describe current behaviour and are updated with each API change.

## Language

Inclusive terms: primary / main / leader (not master), replica / secondary (not slave), allowlist / denylist (not
whitelist / blacklist).
