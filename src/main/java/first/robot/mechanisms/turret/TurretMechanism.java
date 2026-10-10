package first.robot.mechanisms.turret;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchType;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.State;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import first.robot.FieldConstants;
import first.robot.IDs;
import first.robot.Robot;
import first.robot.util.SysIdRoutine;

import frc.robot.constants.robot.RobotConstants;
import frc.robot.util.TunableProfiledPIDController;

public class TurretMechanism implements Mechanism, TelemetryLoggable {
    private final TalonFX kraken = new TalonFX(Constants.TURRET_MOTOR_ID);
    private final DigitalInput hallLimit = new DigitalInput(IDs.TURRET_LIMIT_PIN);
    public final Trigger limitTrigger = new Trigger(() -> !hallLimit.get());
    public boolean isZeroed = false;

    private Supplier<Pose2d> robotPoseSupplier;
    private double prevVel = 0;
    private RobotConstants driveConstants;
    private Supplier<Boolean> turretEnabled;
    private Consumer<Pose2d> setTurretPose;

    private TelemetryTable table;

    private final ProfiledPIDController pid = new ProfiledPIDController(
        TurretConstants.KP, TurretConstants.KI, TurretConstants.KD,
        new TrapezoidProfile.Constraints(
            TurretConstants.MAX_VEL.in(RotationsPerSecond), TurretConstants.MAX_ACC.in(RotationsPerSecondPerSecond)
        )
    );
    private final SimpleMotorFeedforward ff =
        new SimpleMotorFeedforward(TurretConstants.KS, TurretConstants.KV, TurretConstants.KA);

    public TurretMechanism(
        Supplier<Pose2d> robotPoseSupplier, Supplier<Boolean> turretEnabled, Consumer<Pose2d> setTurretPose
    ) {
        super();
        this.setTurretPose = setTurretPose;
        kraken.getConfigurator().apply(new TalonFXConfiguration());
        kraken.getConfigurator().apply(new MotorOutputConfigs()
                                           .withNeutralMode(NeutralModeValue.Brake)
                                           .withInverted(InvertedValue.Clockwise_Positive));
        kraken.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(10));
        kraken.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
                                           .withForwardSoftLimitEnable(true)
                                           .withForwardSoftLimitThreshold(TurretConstants.FORWARD_LIMIT_TUR_REL)
                                           .withReverseSoftLimitEnable(true)
                                           .withReverseSoftLimitThreshold(TurretConstants.REVERSE_LIMIT_TUR_REL));
        kraken.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(45)));

        resetAngle(TurretConstants.START_POS_BOT_REL);
        pid.reset(robotRelToTurretRel(TurretConstants.START_POS_BOT_REL).in(Rotations));
        pid.setTolerance(Units.degreesToRotations(1));
        this.robotPoseSupplier = robotPoseSupplier;
        this.turretEnabled = turretEnabled;

        this.limitTrigger.onTrue(run(coro -> {
                                     resetAngle(TurretConstants.HALL_LIMIT_POS_BOT_REL);
                                     isZeroed = true;
                                 }).named("zero due to hitting limit switch"));
    }

    public static Angle turretRelToRobotRel(Angle input) {
        return Rotations.of(MathUtil.inputModulus(input.plus(TurretConstants.ZERO_OFFSET).in(Rotations), 0, 1));
    }

    public static Angle robotRelToTurretRel(Angle input) {
        return Rotations.of(MathUtil.inputModulus(input.minus(TurretConstants.ZERO_OFFSET).in(Rotations), 0, 1));
    }

    public static boolean isInDeadZone(Angle input) {
        return !robotRelToTurretRel(input).isNear(formatInputPosRobotRel(input), Degrees.of(0.2));
    }

    public static boolean isInDynamicDeadZone(Angle input) {
        return !robotRelToTurretRel(input).isNear(formatInputPosRobotRel(input), TurretConstants.DYNAMIC_TOLERANCE);
    }

    public static Angle formatInputPosTurretRel(Angle input) {
        double softReverseLimit =
            TurretConstants.REVERSE_LIMIT_TUR_REL.plus(TurretConstants.SOFT_PADDING).in(Rotations);
        double softForwardLimit =
            TurretConstants.FORWARD_LIMIT_TUR_REL.minus(TurretConstants.SOFT_PADDING).in(Rotations);
        double moddedInput = MathUtil.inputModulus(input.in(Rotations), 0, 1);
        return Rotations.of(Math.clamp(moddedInput, softReverseLimit, softForwardLimit));
    }

    public static State formatInputStateTurretRel(State input) {
        double softReverseLimit =
            TurretConstants.REVERSE_LIMIT_TUR_REL.plus(TurretConstants.SOFT_PADDING).in(Rotations);
        double softForwardLimit =
            TurretConstants.FORWARD_LIMIT_TUR_REL.minus(TurretConstants.SOFT_PADDING).in(Rotations);
        double moddedInput = MathUtil.inputModulus(input.position, 0, 1);
        double newPos = Math.clamp(moddedInput, softReverseLimit, softForwardLimit);

        double newVel = input.velocity;
        double maxVelRPS = TurretConstants.MAX_VEL.in(RotationsPerSecond);
        newVel = Math.clamp(newVel, -maxVelRPS, maxVelRPS);

        if (newVel > 0) {
            if (MathUtil.isNear(softForwardLimit, newPos, TurretConstants.MAX_DECEL_PADDING.in(Rotations))) {
                newVel = Math.clamp(
                    newVel, 0,
                    Math.sqrt(
                        maxVelRPS * maxVelRPS -
                        2 * TurretConstants.MAX_ACC.in(RotationsPerSecondPerSecond) *
                            (newPos - softForwardLimit + TurretConstants.MAX_DECEL_PADDING.in(Rotations))
                    )
                );
            }
        } else if (newVel < 0) {
            if (MathUtil.isNear(softReverseLimit, newPos, TurretConstants.MAX_DECEL_PADDING.in(Rotations))) {
                newVel = -Math.clamp(
                    newVel,
                    -Math.sqrt(
                        maxVelRPS * maxVelRPS +
                        2 * TurretConstants.MAX_ACC.in(RotationsPerSecondPerSecond) *
                            (newPos - softReverseLimit - TurretConstants.MAX_DECEL_PADDING.in(Rotations))
                    ),
                    0
                );
            }
        }

        return new State(newPos, newVel);
    }

    public static Angle formatInputPosRobotRel(Angle input) {
        return formatInputPosTurretRel(robotRelToTurretRel(input));
    }

    public static State formatInputStateRobotRel(State input) {
        return formatInputStateTurretRel(
            new State(input.position - TurretConstants.ZERO_OFFSET.in(Rotations), input.velocity)
        );
    }

    private void aimedAtHubTelemetry() {
        Translation2d turret = getMechanismPose().getTranslation();

        Translation2d blue = FieldConstants.BLUE_HUB_CENTER;
        Translation2d red = FieldConstants.redElement(FieldConstants.BLUE_HUB_CENTER);
        Translation2d closer = blue.getDistance(turret) < red.getDistance(turret) ? blue : red;
        this.table.log("closer hub", closer);

        Translation2d turretToHub = closer.minus(turret);
        this.table.log("turret to hub", turretToHub);

        Angle shouldBeAngle = turretToHub.getAngle().orElse(Rotation2d.fromDegrees(-1)).getMeasure();
        this.table.log("should be (deg)", shouldBeAngle.in(Degrees));

        Angle isAngle = this.getMechanismPose().getRotation().getMeasure();
        this.table.log("is (deg)", isAngle.in(Degrees));

        double delta = MathUtil.angleModulus(shouldBeAngle.in(Radians)) - MathUtil.angleModulus(isAngle.in(Radians));
        this.table.log("aimed", Math.abs(delta) < Degrees.of(4).in(Radians));
    }

    void doPeriodicTelemetry() {
        this.table.log("posRots", getAngle().in(Rotations));
        this.table.log("velRPS", kraken.getVelocity().getValue().in(RotationsPerSecond));
        this.table.log("setpointPosRots", turretRelToRobotRel(Rotations.of(pid.getSetpoint().position)).in(Rotations));
        this.table.log("setpointVelRPS", pid.getSetpoint().velocity);
        this.table.log("volts", kraken.getMotorVoltage().getValueAsDouble());
        this.table.log("posErrorRots", pid.getPositionError());
        this.table.log("limitEngaged", !hallLimit.get());
        this.table.log("isZeroed", isZeroed);
        this.table.log("atForwardSoftwareLimit", isAtForwardLim.getAsBoolean());
        this.table.log("atReverseSoftwareLimit", isAtReverseLim.getAsBoolean());
        this.table.log("isAtGoalPos", isAtGoalPos());
        this.table.log(
            "distance to hub", FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER)
                                   .getDistance(this.getMechanismPose().getTranslation())
        );
        this.table.log("canTurnTo", canTurnTo(FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER)));
        this.table.log("statorCurrent", kraken.getStatorCurrent().getValueAsDouble());

        this.setTurretPose.accept(getMechanismPose());

        aimedAtHubTelemetry();
    }

    @Override
    public void logTo(TelemetryTable table) {
        this.table = table;
        doPeriodicTelemetry();
    }

    public Command getHardRunForward() {
        return run(_ -> kraken.setVoltage(0.75)).whenCanceled(() -> kraken.setVoltage(0)).named("hard run forward");
    }

    public Command getSoftRunForward() {
        return run(_ -> kraken.setVoltage(0.65)).whenCanceled(() -> kraken.setVoltage(0)).named("soft run forward");
    }

    public Command getSoftRunReverse() {
        return run(_ -> kraken.setVoltage(-0.75)).whenCanceled(() -> kraken.setVoltage(0)).named("soft run reverse");
    }

    public Command getHardRunReverse() {
        return run(_ -> kraken.setVoltage(-1)).whenCanceled(() -> kraken.setVoltage(0)).named("hard run reverse");
    }

    public Command getStop() { return run(_ -> kraken.setVoltage(0)).named("stop turret"); }

    public final BooleanSupplier isAtForwardLim = () -> {
        return getAngleTurretRel().gte(TurretConstants.FORWARD_LIMIT_TUR_REL.minus(TurretConstants.SOFT_PADDING));
    };
    public final BooleanSupplier isAtReverseLim = () -> {
        return getAngleTurretRel().lte(TurretConstants.REVERSE_LIMIT_TUR_REL.plus(TurretConstants.SOFT_PADDING));
    };

    private SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.15).per(Second), Volts.of(1), null, null,
            v
            -> kraken.setVoltage(v.in(Volts)),
            log
            -> {
                log.motor("turret_motor")
                    .voltage(kraken.getMotorVoltage().getValue())
                    .angularVelocity(kraken.getVelocity().getValue())
                    .angularPosition(getAngleTurretRel());
            },
            "turret"
        ),
        this
    );

    /**
     * SysId command for this subsystem
     *
     * @return {@link Command} that runs a {@link SysIdRoutine} for this subsystem,
     *         forward and backward, dynamic and quasistatic. All parts of the
     *         routine end when they hit the upper and lower limit of this
     *         subsystem.
     */
    public Command sysId() {
        return run(coro -> {
                   System.out.println("starting turret sysid");

                   routine.dynamicRun(SysIdRoutine.Direction.FORWARD, coro, isAtForwardLim);
                   routine.dynamicRun(SysIdRoutine.Direction.REVERSE, coro, isAtReverseLim);

                   routine.quasistaticRun(SysIdRoutine.Direction.FORWARD, coro, isAtForwardLim);
                   routine.quasistaticRun(SysIdRoutine.Direction.REVERSE, coro, isAtReverseLim);

                   System.out.println("done with turret sysid");
               }
        ).named("turret sysid");
    }

    /**
     * Runs the turret to track a specified, moving {@link State}. Private because
     * the
     * positional aspect is not clamped. Use {@link #trackRobotRelWithVelocity()}
     * instead.
     *
     * @param goalSupplier Combined position/velocity supplier in the form of a
     *                     {@link State}.
     * @return {@link Command} that repeatedly applies the output of the
     *         {@link ProfiledPIDController} to the motor.
     */
    private Command trackStateTurretRel(Supplier<State> goalSupplier) {
        return run(coro -> {
                   pid.reset(new State(
                       getAngleTurretRel().in(Rotations), kraken.getVelocity().getValue().in(RotationsPerSecond)
                   ));
                   pid.setGoal(goalSupplier.get());
                   prevVel = kraken.getVelocity().getValue().in(RotationsPerSecond);

                   while (true) {
                       if (!turretEnabled.get()) return;

                       double pidOut = pid.calculate(getAngleTurretRel().in(Rotations), goalSupplier.get());
                       this.table.log("pid_out", pidOut);

                       double newVel = pid.getSetpoint().velocity;
                       double ffOut = (!MathUtil.isNear(0, newVel, 0.075))
                                          ? ff.calculate(newVel, (newVel - prevVel) / 0.05)
                                      : (!isAtGoalPos()) ? Math.copySign(KS, pidOut)
                                                         : 0;
                       this.table.log("feedforward_out", ffOut);

                       double voltsToSet = (!isAtGoalPos()) ? pidOut + ffOut : 0;
                       kraken.setVoltage(voltsToSet);
                       prevVel = newVel;
                   }
               })
            .whenCanceled(() -> kraken.setVoltage(0))
            .named("track state turret rel");
    }

    /**
     * Runs the turret to a specified, moving {@link Angle}.
     *
     * @param goal Robot-relative {@link Angle}, will be
     *             clamped to the functional range of this subsystem
     *             automatically.
     * @return {@link Command} that sets the {@link ProfiledPIDController}'s goal to
     *         the parameter, then repeatedly applies its output to the motor.
     */
    public Command holdRobotRel(Angle goal) {
        return run(coro -> {
                   pid.reset(new State(
                       getAngleTurretRel().in(Rotations), kraken.getVelocity().getValue().in(RotationsPerSecond)
                   ));
                   pid.setGoal(new State(TurretConstants.formatInputPosRobotRel(goal).in(Rotations), 0));
                   prevVel = kraken.getVelocity().getValue().in(RotationsPerSecond);

                   while (true) {
                       if (!turretEnabled.get()) return;

                       double pidOut = pid.calculate(getAngleTurretRel().in(Rotations));
                       this.table.log("pid_out", pidOut);

                       double newVel = pid.getSetpoint().velocity;
                       double ffOut = (!MathUtil.isNear(0, newVel, 0.075))
                                          ? ff.calculate(newVel, (newVel - prevVel) / 0.05)
                                      : (MathUtil.isNear(0, pidOut, 0.75 * TurretConstants.KS))
                                          ? 0
                                          : Math.copySign(0.5 * TurretConstants.KS, pidOut);
                       this.table.log("feedforward_out", ffOut);

                       double voltsToSet = pidOut + ffOut;
                       if (isAtGoalPos()) voltsToSet = 0;
                       kraken.setVoltage(voltsToSet);
                       prevVel = newVel;
                   }
               })
            .whenCanceled(() -> kraken.setVoltage(0))
            .named("hold robot relative");
    }

    /**
     * Runs the turret to a specified, moving {@link Angle}.
     *
     * @param angleSupplier Supplier for a robot-relative {@link Angle}, will be
     *                      clamped to the functional range of this subsystem
     *                      automatically.
     * @return {@link Command} that repeatedly applies the output of the
     *         {@link ProfiledPIDController} to the motor.
     */
    public Command trackRobotRel(Supplier<Angle> angleSupplier) {
        return trackStateTurretRel(() -> new State(formatInputPosRobotRel(angleSupplier.get()).in(Rotations), 0));
    }

    /**
     * Runs the turret to a specified, moving {@link Angle}, with a goal velocity at
     * said angle.
     *
     * @param angleSupplier   supplier for a robot-relative {@link Angle}, will be
     *                        clamped to the functional range of this subsystem
     *                        automatically.
     * @param velocitySuppler supplier for a robot-relative {@link AngularVelocity}
     *                        that should be reached at the goal position
     * @return {@link Command} that repeatedly applies the output of the
     *         {@link ProfiledPIDController} to the motor.
     */
    public Command
    trackRobotRelWithVelocity(Supplier<Angle> angleSupplier, Supplier<AngularVelocity> velocitySupplier) {
        return trackStateTurretRel(
            ()
                -> formatInputStateRobotRel(
                    new State(angleSupplier.get().in(Rotations), velocitySupplier.get().in(RotationsPerSecond))
                )
        );
    }

    /**
     * Runs the turret to a specified, static {@link Angle}.
     *
     * @param fieldAngle Field-relative {@link Angle}.
     *
     * @return {@link Command} that repeatedly applies the output of the
     *         {@link ProfiledPIDController} to the motor.
     */
    public Command holdFieldRelative(Angle fieldAngle) {
        return trackRobotRel(() -> fieldAngle.minus(robotPoseSupplier.get().getRotation().getMeasure()));
    }

    /**
     * Runs the turret to aim at a specified, static {@link Translation2d}.
     *
     * @param positionToTrack Field-relative {@link Translation2d}.
     * @return {@link Command} that repeatedly applies the output of the
     *         {@link ProfiledPIDController} to the motor.
     */
    public Command trackFieldPos(Translation2d positionToTrack) {
        return trackRobotRel(() -> {
            Pose2d robotPoseInField = robotPoseSupplier.get();
            Translation2d mechanismInField = getMechanismPose().getTranslation();
            Angle fieldAngle = positionToTrack.minus(mechanismInField).getAngle().get().getMeasure();
            return fieldAngle.minus(robotPoseInField.getRotation().getMeasure());
        });
    }

    public Command trackFieldPosDynamic(Supplier<Translation2d> positionSupplier) {
        return trackRobotRel(() -> {
            Pose2d robotPoseInField = robotPoseSupplier.get();
            Translation2d mechanismInField = getMechanismPose().getTranslation();
            Angle fieldAngle = positionSupplier.get().minus(mechanismInField).getAngle().get().getMeasure();
            return fieldAngle.minus(robotPoseInField.getRotation().getMeasure());
        });
    }

    /**
     * Testing method that resets the Turret's {@link TunableProfiledPIDController}
     * to the constants set in NT
     */
    public void remakePID() {
        State currentState = new State(getAngleTurretRel().in(Rotations), 0);
        pid.reset(currentState);
    }

    /** {@return whether we're at the setpoint} */
    public boolean atGoal() { return this.pid.atGoal(); }

    /**
     * @return the {@link Rotation2d} representing the robot-relative angle of the
     *         turret
     */
    public Rotation2d getRotation() { return Rotation2d.fromRotations(getAngle().in(Rotations)); }

    /**
     * @return the robot-relative {@link Angle} of the turret
     */
    public Angle getAngle() { return turretRelToRobotRel(getAngleTurretRel()); }

    /**
     * @return the turret-relative {@link Angle} of the turret.
     *         <p>
     *         Turret-relative is
     *         an offset position space, where zero is halfway between the turrets
     *         limits; it is only used for internal control logic.
     */
    public Angle getAngleTurretRel() { return kraken.getPosition().getValue(); }

    /**
     * Sets the known angle of the turret subsystem to a new value
     *
     * @param newPos the new robot-relative {@link Angle}
     */
    private void resetAngle(Angle newPos) { kraken.setPosition(robotRelToTurretRel(newPos)); }

    /**
     * @return the field-relative {@link Pose2d} of the turret's center, offset from
     *         the robot's position, with its rotational heading being the angle of
     *         the turret
     */
    public Pose2d getMechanismPose() {
        Pose2d robotPoseInField = robotPoseSupplier.get();
        return new Pose2d(
            driveConstants.getRobotToTurretCenter()
                .rotateBy(robotPoseInField.getRotation())
                .plus(robotPoseInField.getTranslation()),
            getRotation().plus(robotPoseInField.getRotation())
        );
    }

    public boolean isAtGoalPos() {
        return MathUtil.isNear(
            pid.getGoal().position, getAngleTurretRel().in(Rotations), TurretConstants.TOLERANCE.in(Rotations)
        );
    }

    public boolean isDynamicAimed() {
        return MathUtil.isNear(
            pid.getGoal().position, getAngleTurretRel().in(Rotations), TurretConstants.DYNAMIC_TOLERANCE.in(Rotations)
        );
    }

    public boolean isDynamicAimedAt(Angle robotRelTarget) {
        return MathUtil.isNear(
            robotRelToTurretRel(robotRelTarget).in(Rotations), getAngleTurretRel().in(Rotations),
            DYNAMIC_TOLERANCE.in(Rotations)
        );
    }

    /**
     * {@link Command} to zero (find the definite position of) this subsystem. WIll
     * run the motor until the turret reaches its limit switch, then will reset its
     * position, and will finally set its control to hold an angle very close to
     * said limit.
     *
     * @return Command that performs the aforementioned task
     */
    public Command zeroSequence() {
        return run(coro -> {
                   kraken.getConfigurator().apply(
                       new SoftwareLimitSwitchConfigs()
                           .withForwardSoftLimitEnable(false)
                           .withForwardSoftLimitThreshold(TurretConstants.FORWARD_LIMIT_TUR_REL)
                           .withReverseSoftLimitEnable(true)
                           .withReverseSoftLimitThreshold(TurretConstants.REVERSE_LIMIT_TUR_REL)
                   );

                   kraken.setVoltage(0.75);
                   while (!limitTrigger.getAsBoolean())
                       coro.yield();
                   kraken.setVoltage(-0.75);
                   while (limitTrigger.getAsBoolean())
                       coro.yield();
                   kraken.setVoltage(0.65);
                   while (!limitTrigger.getAsBoolean())
                       coro.yield();
                   resetAngle(TurretConstants.HALL_LIMIT_POS_BOT_REL);

                   kraken.getConfigurator().apply(
                       new SoftwareLimitSwitchConfigs()
                           .withForwardSoftLimitEnable(true)
                           .withForwardSoftLimitThreshold(TurretConstants.FORWARD_LIMIT_TUR_REL)
                           .withReverseSoftLimitEnable(true)
                           .withReverseSoftLimitThreshold(TurretConstants.REVERSE_LIMIT_TUR_REL)
                   );
               }
        ).named("turret zero sequence");
    }

    public boolean isZeroed() { return isZeroed; }

    public boolean canTurnTo(Translation2d target) {
        return !isInDeadZone(target.minus(getMechanismPose().getTranslation())
                                 .getAngle()
                                 .get()
                                 .getMeasure()
                                 .minus(robotPoseSupplier.get().getRotation().getMeasure()));
    }

    @Utility
    public static class Tester implements OpMode {
        private final TurretMechanism turret = new TurretMechanism(() -> Pose2d.ZERO, () -> true, _ -> {});

        public Tester(Robot robot) {
            robot.port0.a().onTrue(turret.holdRobotRel(Rotations.of(0.00)));
            robot.port0.b().onTrue(turret.holdRobotRel(Rotations.of(0.75)));
            robot.port0.rightBumper().whileTrue(turret.trackRobotRel(() -> {
                double x = robot.port0.getRightX();
                double y = robot.port0.getRightY();
                return new Rotation2d(-y, -x).getMeasure();
            }));
            robot.port0.x().whileTrue(turret.zeroSequence());

            robot.port0.dpadUp().whileTrue(turret.sysId());
            robot.port0.dpadDown().onTrue(turret.run(_ -> turret.remakePID()).named("remake pid"));
        }

        @Override
        public void start() {
            if (!turret.isZeroed()) Scheduler.getDefault().schedule(turret.zeroSequence());
        }
    }
}
