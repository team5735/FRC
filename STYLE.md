# FRC 5735 Control Freaks

How code, comments and Javadoc are written in this repo. For people and agents
alike: follow it for new code and for any code you touch. All code is Java.

## Units

Use the units library obsessively. When input (e.g. motor readings such as
current RPM) is retrieved (without units-library packaging), add the type as
quickly as possible.

Don't:
```java
double shooterCurrentRPM = motor.getVelocity();
double distanceFromTagMeters = lltable.get("dx", 0.0);
```

Do:
```java
AngularVelocity shooterSpeed = RPM.of(motor.getVelocity());
Distance distanceFromTag = Meters.of(lltable.get("dx", 0.0));
```

## Logic

Always use control structures, such as if-statements and loops. Don't use
ternaries or short-circuiting logic.

Don't:
```java
Pose2d retrieved = drivetrain.getEstimatedPosition();
Pose2d pose = retrieved == null ? new Pose2d() : retrieved;

boolean done = boolean done = isDone != null && isDone.getAsBoolean();
```

Do:
```java
Angle hoodAngle = HoodConstants.LOWEST_ANGLE;
if (requestedAngle != null) {
    hoodAngle = requestedAngle;
}

boolean hasHitSetpoint = false;
if (isDone != null) {
    hasHitSetpoint = isDone.getAsBoolean();
}
```

Don't forget to snapshot things you're nudging:
```java
Angle oldAngle = hood.getHoodAngle();  // snapshot before the default below overwrites it
Angle hoodAngle = HoodConstants.LOWEST_ANGLE;
if (nudgeUp) {
    hoodAngle = oldAngle + NUDGE_ANGLE;  // depends on the real old angle, not the default
}
```

## Purity and state

A pure function is one that always returns the same thing for the same inputs.
State is a general term for anything that outlives a single call to a function.
State should be minimized, but writing purely functional (no state) robot code
is really hard, so this style guide merely cautions against too much state. If a
function changes state, document it with Javadocs.

Don't:
```java
public class HoodMechanism {
    public static double targetAngle;
    
    // (1)
    public static void interp1(double start, double end, double t) {
        targetAngle = start + (end - start) * t;
    }
}
```

(1) is much better implemented as a pure function:
```java
public static double interp1(double start, double end, double t) {
    return start + (end - start) * t;
}
```

(Note: most classes in org.wpilib.math.geometry provide interpolation methods.
Use those instead, if applicable.)

## Names

If you have units in the name of a variable, you should (most of the time) be
using the units library instead. Suffixing a variable with its units when it
already knows its units (due to being of type Distance instead of double, for
example) isn't just redundant, it's error-prone: if the actual units change, you
have to remember to change the name, and because we're human, that is not going
to happen correctly 100% of the time.

Methods are verbs, because they do something. Following this produces code that
reads more like English and therefore more naturally. The prefixes `get set is
has compute measure make solve update convert check` are good starting points.

Don't:
```java
public Angle hoodPosition() { ... }
public LaunchParams launchParams(...) { ... }
public boolean inExclusionZone() { ... }
```

Do:
```java
public Angle getHoodPosition() { ... }
public LaunchParams computeLaunchParams(...) { ... }
public boolean isInExclusionZone() { ... }
```

Don't access class fields directly. Use `getX()` / `setX()` methods.

Prefer verbosity: say as much as is reasonable in names. With liberal use of
autocompletion, you don't have to type your long names out every single time you
use them. Terseness tends to elide information the writer wrongly assumes the
reader has: the purpose of a function, what angle a quantity measures, and so
on.

If there is ever a chance of somebody confusing reference frames, use them
liberally in names. For example, the Limelight can give us the position of the
robot in tag space, the position of the tag in robot space, or the position of
the robot in field space. Variables (and constants) being clear about what frame
of reference they're assuming helps greatly.

## Formatting

The formatting enforced by the formatting configuration (.clang-format) should
be learned. The pre-push hook that runs clang-format on the project is used to
enforce consistent formatting, but you should not rely on it. If the spacing is
consistent, that's one less thing you have to think about when reading code.

Write your code in paragraphs. If you have a chunk of logic that it makes sense
to separate from the rest, simply add newlines. Every chunk should do one thing
with minimal spill/blurring. If it's unclear what a chunk does, it may help to
put a comment before the chunk starts quickly explaining its logic and purpose.

Do not fight the auto-formatter. You will lose. Sometimes, it produces ugly
code, obscuring the structure or just being wasteful with screen space, but that
is worth it for the consistency it offers. Note that while clang-format does
respect individual blank lines, trusting you to use them wisely per the previous
paragraph, it does not respect most other spacing (such as putting a line break
in the middle of a long chain of functions), unlike some other tools.

## Project structure

All robot code is in src/main/java/first/robot. Each subsystem gets its own
folder here. For subsystem X, src/main/java/first/robot/x/ can/will contain:
- `XMechanism.java` (required): the mechanism class for this subsystem
- `XConstants.java`: constants go here (`public static final`, SCREAMING_SNAKE)
- `XTester.java`: only separate the tester class if it's too long to fit
                  inside the mechanism class.

Constants files should not contain logic. Logic typically belongs in the main
mechanism class, but if the mechanism class is getting too large, refactoring
logic out into a separate file in the x directory is a good idea.

Constants files should (and therefore, mechanism files should not) contain
things like tuned values. For example, some motor's default RPM, or perhaps the
max speed at which the robot should drive.

If code is borrowed, try to keep it in a separate file (unless it's really not
that much code). (Of course, you should attribute your borrowing.)

Unlike 2026 code, all CAN/DIO/etc IDs live in ...robot/IDs.java. However, use
judgement in what you put here: unless a constant really is an ID (identifier)
of some kind, it should stay out of this file.

Tester classes live at the end of their respective mechanism classes. They
should look something like the following, continuing the example of using the X
subsystem from earlier:
```java
@Utility(name = "X Tester", group = "Testers")
public static class Tester implements OpMode {
    // ...potentially things here...
    public Tester(Robot robot) { ... }
}
```

## Documentation

Use javadoc. Multiline comments (`/* this is commented */`) that start with an
extra * (`/** this is a javadoc comment */`) are javadoc comments. Javadoc is a
sort of mini-language used to describe the interfaces and purposes of a
function. Here is an example Javadoc, taken from NTEntry.java as it stands:
```java
/**
* Creates a tunable for the named entry with the given getter, setter,
* classType, and config.
*
* @param getter      function called when NT wants to update the tunable's
*                    value
* @param setCallback function called when the tunable is changed in NT
* @param classType   relevant type
* @param config      config of the tunable
*
* @return the Tunable
*/
```
The first line ("Creates a tunable...") is what the function does. It should
be structured as fitting after the function name: "`tunable` creates a...".
If you're going to make a javadoc, it should describe all the parameters and
return value (if any) with as many `@param`s and `@return`s as necessary.
Javadocs should not be redundant with what is actually written, as then they
easily become outdated. Say what it's supposed to do, not what it does. Don't
waste the reader's time, either (tags should be as short as you can be while
maintaining clarity).

## Comments

Comments that are not javadocs should be used judiciously, if and only if there
is no better way to make the code's purpose and workings clearer. Comments that
duplicate code are worse than redundant, because eventually, regardless of how
diligent you may be, one will be updated without the other, at which point the
comment is worse than useless. Perfect code needs no comments, because the code
reads better than the comments ever would (no need to translate computer
concepts into english if the recipient is a computer), but perfect code also
does not exist.

## Docs

README entries describe current behaviour and are updated with each API change.

## Language

Inclusive terms: main/focus instead of master, client instead of slave,
allowlist/denylist instead of whitelist/blacklist, etc.
