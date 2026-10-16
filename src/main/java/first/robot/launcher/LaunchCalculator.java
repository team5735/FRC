// Logic based on FRC 6328 and write-ups by the independent developer Oblarg

package first.robot.launcher;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import java.util.function.BooleanSupplier;

import first.robot.DrivetrainTelemetry;
import first.robot.FieldConstants;
import first.robot.drivetrain.DrivetrainMechanism;
import first.robot.hood.HoodConstants;
import first.robot.hood.HoodMechanism;
import first.robot.spindex.SpinDexMechanism;
import first.robot.turret.TurretMechanism;
import first.robot.util.NTable;

public class LaunchCalculator {
    // key: distance (m) value: angle(deg)
    public static final InterpolatingDoubleTreeMap scoreHoodMap = new InterpolatingDoubleTreeMap();
    // key: distance(m) value: evlocity(RPM)
    public static final InterpolatingDoubleTreeMap scoreSpeedMap = new InterpolatingDoubleTreeMap();
    // key: distance(m) value: Time of Flight (s)
    public static final InterpolatingDoubleTreeMap scoreTOFMap = new InterpolatingDoubleTreeMap();

    // key: distance (m) value: angle(deg)
    public static final InterpolatingDoubleTreeMap ferryHoodMap = new InterpolatingDoubleTreeMap();
    // key: distance (m) value: velocity(RPM)
    public static final InterpolatingDoubleTreeMap ferrySpeedMap = new InterpolatingDoubleTreeMap();
    // key: distance (m) value: Time of Flight (s)
    public static final InterpolatingDoubleTreeMap ferryTOFMap = new InterpolatingDoubleTreeMap();

    static {
        // This is to compenate for variabilities in hood angles
        // that have occured since we calibrated shooting
        // This is needed if/when the hood is detached and reattached
        // to its servo, because its position may be slightly different.
        // this compensates for that
        double hoodTweakOffset = 1.0;

        scoreHoodMap.put(Units.inchesToMeters(56), 8.0 + hoodTweakOffset);
        scoreHoodMap.put(Units.inchesToMeters(71.75), 15.0 + hoodTweakOffset);
        scoreHoodMap.put(Units.inchesToMeters(87.8), 15.0 + hoodTweakOffset);
        scoreHoodMap.put(Units.inchesToMeters(102.2), 20.0 + hoodTweakOffset);
        scoreHoodMap.put(Units.inchesToMeters(216), 20.0 + hoodTweakOffset);

        double rpmMult = 1.02;
        scoreSpeedMap.put(Units.inchesToMeters(56), 2600.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(71.75), 2650.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(87.8), 2675.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(102.2), 2800.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(117), 2900.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(132), 3050.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(147), 3200.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(163), 3350.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(177), 3450.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(192), 3650.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(207), 3850.0 * rpmMult);
        scoreSpeedMap.put(Units.inchesToMeters(216), 4000.0 * rpmMult);

        // TODO - determine real TOFs
        scoreTOFMap.put(0., 0.);
        scoreTOFMap.put(50., 0.);

        // temp values that cause constant trajectory regardless of distance when
        // ferrying
        ferryHoodMap.put(0., HoodConstants.HIGHEST_ANGLE_DEGREES);
        ferryHoodMap.put(50., HoodConstants.HIGHEST_ANGLE_DEGREES);

        ferrySpeedMap.put(0., 4000.);
        ferrySpeedMap.put(50., 4000.);

        ferryTOFMap.put(0., 0.);
        ferryTOFMap.put(0., 0.);
    }

    private static final LinearFilter turretVelFiler = LinearFilter.movingAverage(50);

    private static final double MIN_SCORE_DIST_M = Units.inchesToMeters(56);
    private static final double MAX_SCORE_DIST_M = Units.inchesToMeters(216);

    private static Angle oldTurretAngle;
    private static LaunchParams cachedParams = new LaunchParams(
        false, Rotations.of(0), RotationsPerSecond.of(0), Degrees.of(HoodConstants.LOWEST_ANGLE_DEGREES), RPM.of(0)
    );

    public static final double TOF_TOLERANCE = 0.05;
    public static final double DRIVETRAIN_VELOCITY_SCALING = 0.5;
    public static final double RECALC_PERIOD = 0.02;

    public record LaunchParams(
        boolean isValid,
        Angle turretAngle,  // robot-relative
        AngularVelocity turretVelocity, Angle hoodAngle, AngularVelocity flywheelVelocity
    ) {}

    public enum LaunchGoal {
        SCORE,
        FERRY,
        DEMO,  // for low-powered demo shots //TODO
        LOOP,  // for fun/in-pit testing
    }

    static final NTable table = NTable.root("launch calculator");

    public static void
    calculate(LaunchGoal goal, DrivetrainMechanism drivetrain, ChassisVelocities robotVel, Pose2d turretPose) {
        Translation2d launchTarget;
        var hoodMap = scoreHoodMap;
        var speedMap = scoreSpeedMap;
        var TOFMap = scoreTOFMap;

        switch (goal) {
        case SCORE:
            launchTarget = FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER);
            break;
        case FERRY:
            launchTarget = (FieldConstants.isRed())
                               ? new Translation2d(
                                     FieldConstants.FIELD_LENGTH_X.in(Meters), drivetrain.getEstimatedPosition().getY()
                                 )
                               : new Translation2d(0, drivetrain.getEstimatedPosition().getY());
            double y = launchTarget.getY();
            y += (FieldConstants.BLUE_HUB_CENTER.getY() - y) / 2.0;
            launchTarget = new Translation2d(launchTarget.getX(), y);
            DrivetrainTelemetry.field.getObject("ferry_target").setPose(new Pose2d(launchTarget, Rotation2d.ZERO));

            hoodMap = ferryHoodMap;
            speedMap = ferrySpeedMap;
            TOFMap = ferryTOFMap;
            break;
        case DEMO:
            cachedParams =
                new LaunchParams(true, Rotations.of(0), RotationsPerSecond.of(0), Degrees.of(25), RPM.of(3500));
            return;
        case LOOP:
        default:
            cachedParams = new LaunchParams(
                true,
                Rotations.of(0.875),                             // Roughly face the robot's center
                RotationsPerSecond.of(0),                        // Hold the pose
                Degrees.of(HoodConstants.LOWEST_ANGLE_DEGREES),  // Minimum horizontal velocity component
                RPM.of(750)
            );  // TODO find appropriate speed
            return;
        }

        double robotAngle = drivetrain.getEstimatedPosition().getRotation().getRadians();

        Translation2d launchOrigin = turretPose.getTranslation();

        double turretVelX =
            robotVel.vx + robotVel.omega * (drivetrain.constants.robotToTurretCenter().getY() * Math.cos(robotAngle) -
                                            drivetrain.constants.robotToTurretCenter().getX() * Math.sin(robotAngle));

        table.set("turret_vx", turretVelX);

        double turretVelY =
            robotVel.vy + robotVel.omega * (drivetrain.constants.robotToTurretCenter().getX() * Math.cos(robotAngle) -
                                            drivetrain.constants.robotToTurretCenter().getY() * Math.sin(robotAngle));

        table.set("turret_vy", turretVelX);

        double launchDist = launchTarget.getDistance(launchOrigin);  // naive initial estimate

        table.set("naive_dist", launchDist);
        double timeOfFlight;
        Translation2d lookaheadOrigin = launchOrigin;
        double oldTOF = 100000;  // arbitrarily large; be glad I didn't make this 67676767
        for (int i = 0; i < 20; i++) {
            timeOfFlight = TOFMap.get(launchDist);
            double deltaX = turretVelX * timeOfFlight;
            double deltaY = turretVelY * timeOfFlight;

            lookaheadOrigin = launchOrigin.plus(new Translation2d(deltaX, deltaY));
            launchDist = launchTarget.getDistance(lookaheadOrigin);

            if (MathUtil.isNear(oldTOF, timeOfFlight, TOF_TOLERANCE)) {
                break;
            }

            oldTOF = timeOfFlight;
        }

        launchOrigin = lookaheadOrigin;

        table.set("wise_dist", launchDist);

        Rotation2d turretRot =
            launchTarget.minus(launchOrigin).getAngle().get().minus(drivetrain.getEstimatedPosition().getRotation());

        Angle turretAngle = turretRot.getMeasure();

        DrivetrainTelemetry.field.getObject("launch origin").setPose(turretPose);
        DrivetrainTelemetry.field.getObject("launch lookahead")
            .setPose(new Pose2d(launchOrigin, turretRot.plus(drivetrain.getEstimatedPosition().getRotation())));

        if (oldTurretAngle == null) oldTurretAngle = turretAngle;

        AngularVelocity turretVel =
            RotationsPerSecond.of(turretVelFiler.calculate(turretAngle.minus(oldTurretAngle).in(Rotations) / 0.02));

        oldTurretAngle = turretAngle;
        Angle hoodAngle = Degrees.of(hoodMap.get(launchDist));

        AngularVelocity flywheelVel = RPM.of(speedMap.get(launchDist));
        table.set("turret_angle_rot", turretAngle.in(Rotations));
        table.set("turret_vel_rps", turretVel.in(RotationsPerSecond));
        table.set("hood_angle_deg", hoodAngle.in(Degrees));
        table.set("flywheel_vel_rpm", flywheelVel.in(RPM));

        cachedParams = new LaunchParams(
            ((launchDist >= MIN_SCORE_DIST_M) && (launchDist <= MAX_SCORE_DIST_M)) ||
                goal.equals(LaunchGoal.FERRY),  // TODO - set exclusion zones
            turretAngle, turretVel, hoodAngle, flywheelVel
        );
    }

    public static LaunchParams getCachedParams() { return cachedParams; }

    private static Command reCalcParams(LaunchGoal goal, TurretMechanism turret, DrivetrainMechanism drivetrain) {
        return Command
            .noRequirements(
                _ -> calculate(goal, drivetrain, drivetrain.getChassisVelocitiesFieldRel(), turret.getMechanismPose())
            )
            .named("recalc params");
    }

    public static Command dynamicLaunchCommand(
        LaunchGoal goal, BooleanSupplier override, HoodMechanism hood, TurretMechanism turret,
        DrivetrainMechanism drivetrain, LauncherMechanism launcher, SpinDexMechanism spindex
    ) {
        return Command
            .parallel(
                reCalcParams(goal, turret, drivetrain),
                hood.runRepeatedly(() -> hood.setHoodAngle(getCachedParams().hoodAngle.in(Degrees)))
                    .named("launch calculator: hood angle"),
                turret.trackRobotRelWithVelocity(
                    () -> getCachedParams().turretAngle, () -> getCachedParams().turretVelocity
                ),
                launcher.getLaunchFuelSupplier(() -> getCachedParams().flywheelVelocity),
                spindex.idle()
                    .until(() -> {
                        LaunchParams params = getCachedParams();
                        boolean hoodCheck = MathUtil.isNear(
                            params.hoodAngle.in(Degrees), hood.getNormalizedAngle(),
                            HoodConstants.DYNAMIC_TOLERANCE_DEGREES
                        );
                        table.set("hood check", hoodCheck);
                        boolean turretCheck = turret.isDynamicAimedAt(getCachedParams().turretAngle) &&
                                              !TurretMechanism.isInDynamicDeadZone(getCachedParams().turretAngle);
                        table.set("turret check", turretCheck);
                        boolean launcherCheck = params.flywheelVelocity.in(RPM) < launcher.getRPM();
                        table.set("launcher check", launcherCheck);
                        return (
                            (params.isValid && hoodCheck && turretCheck && launcherCheck) || override.getAsBoolean()
                        );
                    })
                    .optional(Command.waitFor(Seconds.of(3)).named("dynamic launch timeout"))
                    .withAutomaticName()
                    .andThen(spindex.getInformedRun(
                        ()
                            -> !TurretMechanism.isInDynamicDeadZone(getCachedParams().turretAngle) ||
                                   override.getAsBoolean()
                    ))
                    .withAutomaticName()
            )
            .named("launch calculator: dynamic launch");
    }

    private static Timer timeOut = new Timer();

    public static Command dynamicLaunchTeleop(
        CommandXboxController controller, LaunchGoal goal, BooleanSupplier override, HoodMechanism hood,
        TurretMechanism turret, DrivetrainMechanism drivetrain, LauncherMechanism launcher, SpinDexMechanism spindex
    ) {
        return Command.noRequirements(_ -> timeOut.restart())
            .named("launch calculator: restart timeout")
            .andThen(dynamicLaunchCommand(
                         goal,
                         ()
                             -> {
                             table.set("timeOut", timeOut.get());
                             return timeOut.get() > 1.;
                         },
                         hood, turret, drivetrain, launcher, spindex
            )
                         .alongWith(drivetrain.joystickDriveCommand(
                             ()
                                 -> controller.getLeftX() * DRIVETRAIN_VELOCITY_SCALING,
                             ()
                                 -> controller.getLeftY() * DRIVETRAIN_VELOCITY_SCALING,
                             ()
                                 -> controller.getLeftTrigger() * DRIVETRAIN_VELOCITY_SCALING,
                             ()
                                 -> controller.getRightTrigger() * DRIVETRAIN_VELOCITY_SCALING,
                             () -> controller.y().getAsBoolean(), () -> controller.menu().getAsBoolean()
                         ))
                         .withAutomaticName())
            .withAutomaticName();
    }

    public static Command dynamicLaunchAuto(
        LaunchGoal goal, HoodMechanism hood, TurretMechanism turret, DrivetrainMechanism drivetrain,
        LauncherMechanism launcher, SpinDexMechanism spindex
    ) {
        return Command
            .noRequirements(coro -> {
                timeOut.restart();
                coro.await(dynamicLaunchCommand(goal, () -> {
                    table.set("timeOut", timeOut.get());
                    return timeOut.get() > 1.;
                }, hood, turret, drivetrain, launcher, spindex));
            })
            .named("launch calculator: dynamic launch auto");
    }

    public static Command dryAimTurret(LaunchGoal goal, TurretMechanism turret, DrivetrainMechanism drivetrain) {
        return Command
            .parallel(
                reCalcParams(goal, turret, drivetrain),
                turret.trackRobotRelWithVelocity(
                    () -> getCachedParams().turretAngle, () -> getCachedParams().turretVelocity
                )
            )
            .withAutomaticName();
    }
}
