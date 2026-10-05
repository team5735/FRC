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
import first.robot.util.NTable;
import first.robot.util.SysIdRoutine;
import frc.robot.PartialRobot;
import frc.robot.commands.LaunchCalculator;
import frc.robot.commands.LaunchCalculator.LaunchGoal;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.robot.CompbotConstants;
import frc.robot.constants.robot.CompbotTunerConstants;
import frc.robot.constants.robot.RobotConstants;
import frc.robot.util.TunableProfiledPIDController;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.hardware.hal.SimDevice.Direction;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

public class TurretMechanism implements Mechanism, TelemetryLoggable {
    private final TalonFX kraken = new TalonFX(Constants.TURRET_MOTOR_ID);
    private final DigitalInput hallLimit = new DigitalInput(Constants.TURRET_LIMIT_PIN);
    public final Trigger limitTrigger = new Trigger(() -> !hallLimit.get());
    public final Trigger zeroTrigger = limitTrigger.and(() -> DriverStation.isDisabled());
    public boolean isZeroed = false;

    private final ProfiledPIDController pid = new ProfiledPIDController(
        TurretConstants.KP, TurretConstants.KI, TurretConstants.KD,
        new TrapezoidProfile.Constraints(TurretConstants.MAX_VEL.in(RotationsPerSecond),
                                         TurretConstants.MAX_ACC.in(RotationsPerSecondPerSecond)));
    private final SimpleMotorFeedforward ff =
        new SimpleMotorFeedforward(TurretConstants.KS, TurretConstants.KV, TurretConstants.KA);

    private Supplier<Pose2d> robotPoseSupplier;
    private double prevVel = 0;
    private RobotConstants driveConstants;

    private Supplier<Boolean> turretEnabled;

    public TurretMechanism(Supplier<Pose2d> robotPoseSupplier, RobotConstants driveConstants,
                           Supplier<Boolean> turretEnabled) {
        super();
        this.driveConstants = driveConstants;
        kraken.getConfigurator().apply(new TalonFXConfiguration());
        kraken.getConfigurator().apply(new MotorOutputConfigs()
                                           .withNeutralMode(NeutralModeValue.Brake)
                                           .withInverted(InvertedValue.Clockwise_Positive));
        kraken.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(10));
        resetAngle(TurretConstants.START_POS_BOT_REL);
        kraken.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
                                           .withForwardSoftLimitEnable(true)
                                           .withForwardSoftLimitThreshold(TurretConstants.FORWARD_LIMIT_TUR_REL)
                                           .withReverseSoftLimitEnable(true)
                                           .withReverseSoftLimitThreshold(TurretConstants.REVERSE_LIMIT_TUR_REL));
        kraken.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(45)));
        pid.setup(robotRelToTurretRel(TurretConstants.START_POS_BOT_REL).in(Rotations));
        pid.reset(robotRelToTurretRel(TurretConstants.START_POS_BOT_REL).in(Rotations));
        pid.setTolerance(Units.degreesToRotations(1));
        this.robotPoseSupplier = robotPoseSupplier;
        this.turretEnabled = turretEnabled;
    }

    private void aimedAtHubTelemetry() {
        NTable table = NTable.root("turret");

        Translation2d turret = getMechanismPose().getTranslation();

        Translation2d blue = FieldConstants.BLUE_HUB_CENTER;
        Translation2d red = FieldConstants.redElement(FieldConstants.BLUE_HUB_CENTER);
        Translation2d closer = blue.getDistance(turret) < red.getDistance(turret) ? blue : red;
        table.set("closer hub", closer);

        Translation2d turretToHub = closer.minus(turret);
        table.set("turret to hub", turretToHub);

        Angle shouldBeAngle = turretToHub.getAngle().orElse(Rotation2d.fromDegrees(-1)).getMeasure();
        table.set("should be (deg)", shouldBeAngle.in(Degrees));

        Angle isAngle = this.getMechanismPose().getRotation().getMeasure();
        table.set("is (deg)", isAngle.in(Degrees));

        double delta = MathUtil.angleModulus(shouldBeAngle.in(Radians)) - MathUtil.angleModulus(isAngle.in(Radians));
        table.set("aimed", Math.abs(delta) < Degrees.of(4).in(Radians));
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("turret/posRots", getAngle().in(Rotations));
        table.log("turret/velRPS", kraken.getVelocity().getValue().in(RotationsPerSecond));
        table.log("turret/setpointPosRots",
                  turretRelToRobotRel(Rotations.of(pid.getSetpoint().position)).in(Rotations));
        table.log("turret/setpointVelRPS", pid.getSetpoint().velocity);
        table.log("turret/volts", kraken.getMotorVoltage().getValueAsDouble());
        table.log("turret/posErrorRots", pid.getPositionError());
        table.log("turret/limitEngaged", !hallLimit.get());
        table.log("turret/isZeroed", isZeroed);
        table.log("turret/atForwardSoftwareLimit", isAtForwardLim.getAsBoolean());
        table.log("turret/atReverseSoftwareLimit", isAtReverseLim.getAsBoolean());
        table.log("turret/isAtGoalPos", isAtGoalPos());
        table.log("turret/distance to hub", FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER)
                                                .getDistance(this.getMechanismPose().getTranslation()));
        table.log("turret/canTurnTo", canTurnTo(FieldConstants.alliance(FieldConstants.BLUE_HUB_CENTER)));
        Telemetry.field.getObject("turret_pose").setPose(getMechanismPose());
        table.log("turret/statorCurrent", kraken.getStatorCurrent().getValueAsDouble());
        aimedAtHubTelemetry();
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

    private SysIdRoutine routine =
        new SysIdRoutine(new SysIdRoutine.Config(Volts.of(0.15).per(Second), Volts.of(1), null, null,
                                                 v
                                                 -> kraken.setVoltage(v.in(Volts)),
                                                 log
                                                 -> {
                                                     log.motor("turret_motor")
                                                         .voltage(kraken.getMotorVoltage().getValue())
                                                         .angularVelocity(kraken.getVelocity().getValue())
                                                         .angularPosition(getAngleTurretRel());
                                                 },
                                                 "turret"),
                         this);

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
               })
            .named("turret sysid");
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
        return startRun(
                   ()
                       -> {
                       pid.reset(new State(getAngleTurretRel().in(Rotations),
                                           kraken.getVelocity().getValue().in(RotationsPerSecond)));
                       pid.setGoal(goalSupplier.get());
                       prevVel = kraken.getVelocity().getValue().in(RotationsPerSecond);
                   },
                   () -> {
                       if (!turretEnabled.get()) {
                           return;
                       }
                       double pidOut = pid.calculate(getAngleTurretRel().in(Rotations), goalSupplier.get());
                       SmartDashboard.putNumber("turret/pidOut", pidOut);
                       double newVel = pid.getController().getSetpoint().velocity;
                       double ffOut = (!MathUtil.isNear(0, newVel, 0.075))
                                          ? ff.calculate(newVel, (newVel - prevVel) / 0.05)
                                      : (!isAtGoalPos()) ? Math.copySign(KS, pidOut)
                                                         : 0;
                       SmartDashboard.putNumber("turret/ffOut", ffOut);
                       double voltsToSet = (!isAtGoalPos()) ? pidOut + ffOut : 0;
                       kraken.setVoltage(voltsToSet);
                       prevVel = newVel;
                   })
            .finallyDo(() -> kraken.setVoltage(0));
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
        return startRun(
                   ()
                       -> {
                       pid.reset(new State(getAngleTurretRel().in(Rotations),
                                           kraken.getVelocity().getValue().in(RotationsPerSecond)));
                       pid.setGoal(new State(formatInputPosRobotRel(goal).in(Rotations), 0));
                       prevVel = kraken.getVelocity().getValue().in(RotationsPerSecond);
                   },
                   () -> {
                       if (!turretEnabled.get()) {
                           return;
                       }
                       var _T = new frc.robot.util.Timer("");
                       double pidOut = pid.calculate(getAngleTurretRel().in(Rotations));
                       SmartDashboard.putNumber("turret/pidOut", pidOut);
                       double newVel = pid.getController().getSetpoint().velocity;
                       double ffOut = (!MathUtil.isNear(0, newVel, 0.075))
                                          ? ff.calculate(newVel, (newVel - prevVel) / 0.05)
                                      : (MathUtil.isNear(0, pidOut, 0.75 * KS)) ? 0
                                                                                : Math.copySign(0.5 * KS, pidOut);
                       SmartDashboard.putNumber("turret/ffOut", ffOut);
                       double voltsToSet = (!isAtGoalPos()) ? pidOut + ffOut : 0;
                       kraken.setVoltage(voltsToSet);
                       prevVel = newVel;
                       _T.toc();
                   })
            .finallyDo(() -> kraken.setVoltage(0))
            .withName("hold robot relative");
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
    public Command trackRobotRelWithVelocity(Supplier<Angle> angleSupplier,
                                             Supplier<AngularVelocity> velocitySupplier) {
        return trackStateTurretRel(
            ()
                -> formatInputStateRobotRel(
                    new State(angleSupplier.get().in(Rotations), velocitySupplier.get().in(RotationsPerSecond))));
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
            Angle fieldAngle = positionToTrack.minus(mechanismInField).getAngle().getMeasure();
            return fieldAngle.minus(robotPoseInField.getRotation().getMeasure());
        });
    }

    public Command trackFieldPosDynamic(Supplier<Translation2d> positionSupplier) {
        return trackRobotRel(() -> {
            Pose2d robotPoseInField = robotPoseSupplier.get();
            Translation2d mechanismInField = getMechanismPose().getTranslation();
            Angle fieldAngle = positionSupplier.get().minus(mechanismInField).getAngle().getMeasure();
            return fieldAngle.minus(robotPoseInField.getRotation().getMeasure());
        });
    }

    /**
     * Testing method that resets the Turret's {@link TunableProfiledPIDController}
     * to the constants set in NT
     */
    public void remakePID() {
        State currentState = new State(getAngleTurretRel().in(Rotations), 0);
        pid.setup(currentState, 0);
        pid.reset(currentState);
    }

    /** {@return whether we're at the setpoint} */
    public boolean atGoal() { return this.pid.getController().atGoal(); }

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
        return new Pose2d(driveConstants.getRobotToTurretCenter()
                              .rotateBy(robotPoseInField.getRotation())
                              .plus(robotPoseInField.getTranslation()),
                          getRotation().plus(robotPoseInField.getRotation()));
    }

    public boolean isAtGoalPos() {
        return MathUtil.isNear(pid.getController().getGoal().position, getAngleTurretRel().in(Rotations),
                               TOLERANCE.in(Rotations));
    }

    public boolean isDynamicAimed() {
        return MathUtil.isNear(pid.getController().getGoal().position, getAngleTurretRel().in(Rotations),
                               DYNAMIC_TOLERANCE.in(Rotations));
    }

    public boolean isDynamicAimedAt(Angle robotRelTarget) {
        return MathUtil.isNear(robotRelToTurretRel(robotRelTarget).in(Rotations), getAngleTurretRel().in(Rotations),
                               DYNAMIC_TOLERANCE.in(Rotations));
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
        return hardRunForward()
            .until(limitTrigger::getAsBoolean)
            .andThen(softRunReverse().until(() -> !limitTrigger.getAsBoolean()))
            .andThen(softRunForward().until(limitTrigger::getAsBoolean))
            .andThen(zeroCommand())
            .withName("zero sequence")
            .beforeStarting(
                ()
                    -> kraken.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
                                                          .withForwardSoftLimitEnable(false)
                                                          .withForwardSoftLimitThreshold(FORWARD_LIMIT_TUR_REL)
                                                          .withReverseSoftLimitEnable(true)
                                                          .withReverseSoftLimitThreshold(REVERSE_LIMIT_TUR_REL)))
            .finallyDo(
                ()
                    -> kraken.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
                                                          .withForwardSoftLimitEnable(true)
                                                          .withForwardSoftLimitThreshold(FORWARD_LIMIT_TUR_REL)
                                                          .withReverseSoftLimitEnable(true)
                                                          .withReverseSoftLimitThreshold(REVERSE_LIMIT_TUR_REL)));
    }

    /**
     * Non-requiring {@link Command} that simply zeroes the position of this
     * subsytem to that of its Hall-Effect limit switch being engaged
     * <p>
     * This is intended to be bound to {@link TurretMechanism#limitTrigger} and used
     * by very little else.
     *
     * @return a Command generated with {@link Commands#runOnce()} that sets this
     *         subsystem's known angle to its limit
     */
    public Command zeroCommand() {
        return Commands
            .runOnce(() -> {
                resetAngle(HALL_LIMIT_POS_BOT_REL);
                isZeroed = true;
            })
            .ignoringDisable(true);
    }

    public boolean getZeroStatus() { return isZeroed; }

    public boolean canTurnTo(Translation2d target) {
        return !isInDeadZone(target.minus(getMechanismPose().getTranslation())
                                 .getAngle()
                                 .getMeasure()
                                 .minus(robotPoseSupplier.get().getRotation().getMeasure()));
    }

    public static class Tester extends PartialRobot {
        private final TurretMechanism turret =
            new TurretMechanism(() -> Pose2d.kZero, new CompbotConstants(), () -> true);

        public Tester() {
            super();
            // turret.setDefaultCommand(turret.holdRobotRel(Rotations.of(0)));
            turret.zeroTrigger.onTrue(turret.zeroCommand()); // resets the turrets position when it engages the
                                                             // Hall-Effect
                                                             // sensor and the robot is Disabled

            controller.a().onTrue(turret.holdRobotRel(Rotations.of(0.00)));
            controller.b().onTrue(turret.holdRobotRel(Rotations.of(0.75)));
            controller.rightBumper().whileTrue(turret.trackRobotRel(() -> {
                double x = controller.getRightX();
                double y = controller.getRightY();
                return new Rotation2d(-y, -x).getMeasure();
            }));
            controller.x().whileTrue(turret.zeroSequence());

            controller.povUp().whileTrue(turret.sysId());
            controller.povDown().onTrue(Commands.runOnce(turret::remakePID, turret));
        }

        @Override
        public void teleopInit() {
            if (!turret.getZeroStatus()) {
                CommandScheduler.getInstance().schedule(turret.zeroSequence());
            }
        }

        @Override
        public void autonomousInit() {
            if (!turret.getZeroStatus()) {
                CommandScheduler.getInstance().schedule(turret.zeroSequence());
            }
        }
    }
}
