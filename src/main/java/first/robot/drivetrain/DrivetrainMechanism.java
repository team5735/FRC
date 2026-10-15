package first.robot.drivetrain;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;

import java.util.function.Consumer;
import java.util.function.Supplier;

import first.robot.FieldConstants;
import first.robot.util.NTable;
import first.robot.util.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DrivetrainMechanism
    extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Mechanism, TelemetryLoggable {
    public RobotConstants constants;

    private static final double simLoopPeriod = 0.005;  // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    private NTable table = NTable.root("drivetrain");

    /* Swerve requests to apply during SysId characterization */
    public final SwerveRequest.SysIdSwerveTranslation translationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();
    public final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    public final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public final SwerveRequest.FieldCentric fieldCentricRequest;
    public final SwerveRequest.FieldCentric pidRequest;
    public final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    public final SwerveRequest.ApplyRobotVelocity autoRequest =
        new SwerveRequest.ApplyRobotVelocity().withDriveRequestType(DriveRequestType.Velocity);

    private Pose2d lastVisionUpdatePose = new Pose2d();
    private double lastVisionUpdateTime = Double.MIN_VALUE;

    @SuppressWarnings("unused")
    private final Consumer<SysIdRoutineLog> openTranslationLogConsumer = (log) -> {
        log.motor("FL_drive")
            .linearPosition(Meters.of(getState().Pose.getY()))
            .linearVelocity(MetersPerSecond.of(getState().ModuleVelocities[0].velocity))
            .voltage(getModule(0).getDriveMotor().getMotorVoltage().getValue());
        log.motor("FR_drive")
            .linearPosition(Meters.of(getState().Pose.getY()))
            .linearVelocity(MetersPerSecond.of(getState().ModuleVelocities[1].velocity))
            .voltage(getModule(1).getDriveMotor().getMotorVoltage().getValue());
        log.motor("BL_drive")
            .linearPosition(Meters.of(getState().Pose.getY()))
            .linearVelocity(MetersPerSecond.of(getState().ModuleVelocities[2].velocity))
            .voltage(getModule(2).getDriveMotor().getMotorVoltage().getValue());
        log.motor("BR_drive")
            .linearPosition(Meters.of(getState().Pose.getY()))
            .linearVelocity(MetersPerSecond.of(getState().ModuleVelocities[3].velocity))
            .voltage(getModule(3).getDriveMotor().getMotorVoltage().getValue());
    };

    private final Consumer<SysIdRoutineLog> closedTranslationLogConsumer = (log) -> {
        log.motor("FL_drive")
            .angularPosition(getModule(0).getDriveMotor().getPosition().getValue())
            .angularVelocity(getModule(0).getDriveMotor().getVelocity().getValue())
            .voltage(getModule(0).getDriveMotor().getMotorVoltage().getValue());
        log.motor("FR_drive")
            .angularPosition(getModule(1).getDriveMotor().getPosition().getValue())
            .angularVelocity(getModule(1).getDriveMotor().getVelocity().getValue())
            .voltage(getModule(1).getDriveMotor().getMotorVoltage().getValue());
        log.motor("BL_drive")
            .angularPosition(getModule(2).getDriveMotor().getPosition().getValue())
            .angularVelocity(getModule(2).getDriveMotor().getVelocity().getValue())
            .voltage(getModule(2).getDriveMotor().getMotorVoltage().getValue());
        log.motor("BR_drive")
            .angularPosition(getModule(3).getDriveMotor().getPosition().getValue())
            .angularVelocity(getModule(3).getDriveMotor().getVelocity().getValue())
            .voltage(getModule(3).getDriveMotor().getMotorVoltage().getValue());
    };

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second), Volts.of(2.5),
            null,  // Use default timeout (10 s)
            // Log state by default
            null,
            output
            -> setControl(translationCharacterization.withVolts(output)),
            closedTranslationLogConsumer, "drivetrain translation"
        ),
        this
    );

    private final Consumer<SysIdRoutineLog> steerLogConsumer = (log) -> {
        log.motor("FL_steer")
            .angularPosition(getModule(0).getSteerMotor().getPosition().getValue())
            .angularVelocity(getModule(0).getSteerMotor().getVelocity().getValue())
            .voltage(getModule(0).getSteerMotor().getMotorVoltage().getValue());
        log.motor("FR_steer")
            .angularPosition(getModule(1).getSteerMotor().getPosition().getValue())
            .angularVelocity(getModule(1).getSteerMotor().getVelocity().getValue())
            .voltage(getModule(1).getSteerMotor().getMotorVoltage().getValue());
        log.motor("BL_steer")
            .angularPosition(getModule(2).getSteerMotor().getPosition().getValue())
            .angularVelocity(getModule(2).getSteerMotor().getVelocity().getValue())
            .voltage(getModule(2).getSteerMotor().getMotorVoltage().getValue());
        log.motor("BR_steer")
            .angularPosition(getModule(3).getSteerMotor().getPosition().getValue())
            .angularVelocity(getModule(3).getSteerMotor().getVelocity().getValue())
            .voltage(getModule(3).getSteerMotor().getMotorVoltage().getValue());
    };

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Use default ramp rate (1 V/s)
            Volts.of(7),  // Use dynamic voltage of 7 V
            null,         // Use default timeout (10 s)
            // Log state by default
            null, volts -> setControl(steerCharacterization.withVolts(volts)), steerLogConsumer, "drivetrain steer"
        ),
        this
    );

    private final Consumer<SysIdRoutineLog> rotationLogConsumer = (log) -> {
        log.motor("robot_rot")
            .angularPosition(getPigeon2().getYaw().getValue())
            .angularVelocity(getPigeon2().getAngularVelocityZWorld().getValue())
            .voltage(getModule(0).getDriveMotor().getMotorVoltage().getValue());
    };
    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null,  // Use default timeout (10 s)
            // Log state by default
            null,
            output
            -> /* output is actually radians per second, but SysId only supports "volts" */
            setControl(rotationCharacterization.withRotationalRate(output.in(Volts))),
            rotationLogConsumer, "drivetrain rotation"
        ),
        this
    );

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct the devices themselves. If they
     * need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public DrivetrainMechanism(
        SwerveDrivetrainConstants drivetrainConstants, RobotConstants constants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.constants = constants;

        fieldCentricRequest = new SwerveRequest.FieldCentric()
                                  .withDriveRequestType(DriveRequestType.Velocity)
                                  .withCenterOfRotation(constants.pigeonToCenterOfRotation());

        pidRequest = new SwerveRequest.FieldCentric()
                         .withDriveRequestType(DriveRequestType.Velocity)
                         .withCenterOfRotation(constants.pigeonToCenterOfRotation())
                         .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

        setUpAuto();
    }

    private boolean hasAppliedOperatorPerspective = false;

    @Override
    public void logTo(TelemetryTable table) {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state. This
         * allows us to correct the perspective in case the robot code restarts mid-match. Otherwise, only check and
         * apply the operator perspective if the DS is disabled. This ensures driving behavior doesn't change until an
         * explicit disable event occurs during testing.
         */
        int qualityIndex = this.getVisionPoseAgreementQuality();
        String asString = switch (qualityIndex)
        {
            case 0 -> "whack";
            case 1 -> "poor";
            case 2 -> "fair";
            case 3 -> "great";
            default -> "???";
        };

        table.log("pose quality", asString);
        table.log("can shoot accurately?", qualityIndex >= 2);
        table.log(
            "rotational confidence", getEstimatedPosition().getRotation().getMeasure().isNear(
                                         lastVisionUpdatePose.getRotation().getMeasure(), Degrees.of(5)
                                     )
        );

        if (!hasAppliedOperatorPerspective || RobotState.isDisabled()) {
            setOperatorForwardDirection(
                FieldConstants.isRed() ? FieldConstants.RED_ALLIANCE_PERSPECTIVE_ROTATION
                                       : FieldConstants.BLUE_ALLIANCE_PERSPECTIVE_ROTATION
            );
            hasAppliedOperatorPerspective = true;
        }
    }

    public final Pose2d getEstimatedPosition() { return getState().Pose; }

    /** For use by PIDs. Speed limited for safety. */
    public void pidDrive(double vx, double vy, double omega) { pidDrive(new Translation2d(vx, vy), omega); }

    public void pidDrive(Translation2d trans, double omega) {
        if (trans.getNorm() > constants.defaultSpeed().in(MetersPerSecond)) {
            trans = trans.times(constants.defaultSpeed().in(MetersPerSecond) / trans.getNorm());
        }
        omega = Math.min(constants.defaultRotationalRate().in(RadiansPerSecond), omega);
        setControl(pidRequest.withVelocityX(trans.getX()).withVelocityY(trans.getY()).withRotationalRate(omega));
    }

    public Command joystickDriveCommand(
        Supplier<Double> stickX, Supplier<Double> stickY, Supplier<Double> leftTrigger, Supplier<Double> rightTrigger,
        Supplier<Boolean> isSlowMode, Supplier<Boolean> isTurboMode
    ) {
        return runRepeatedly(
                   () -> {
                       double speedMPS = (isSlowMode.get().booleanValue()) ? constants.slowSpeed().in(MetersPerSecond)
                                         : (isTurboMode.get().booleanValue())
                                             ? constants.turboSpeed().in(MetersPerSecond)
                                             : constants.defaultSpeed().in(MetersPerSecond);

                       double rotationMPS = (isSlowMode.get().booleanValue())
                                                ? constants.slowRotationalRate().in(RadiansPerSecond)
                                            : (isTurboMode.get().booleanValue())
                                                ? constants.turboRotationalRate().in(RadiansPerSecond)
                                                : constants.defaultRotationalRate().in(RadiansPerSecond);

                       var ret =
                           fieldCentricRequest.withVelocityX(-deadband(stickY.get()) * speedMPS)
                               .withVelocityY(-deadband(stickX.get()) * speedMPS)
                               .withRotationalRate(deadband(leftTrigger.get() - rightTrigger.get()) * rotationMPS);

                       setControl(ret);
                   }
        ).named("drivetrain: joystick drive");
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(simLoopPeriod);
    }

    public static double deadband(double input) {
        if (Math.abs(input) <= DrivetrainConstants.DEFAULT_DEADBAND) return 0;
        return input;
    }

    public ChassisVelocities getChassisVelocities() {
        var states = new SwerveModuleVelocity[4];
        for (int i = 0; i < 4; i++)
            states[i] = getModule(i).getCurrentVelocity();
        return constants.config().toChassisSpeeds(states);
    }

    public ChassisVelocities getChassisVelocitiesFieldRel() {
        return getChassisVelocities().toFieldRelative(getEstimatedPosition().getRotation());
    }

    public void autoDriveRobotRelative(ChassisVelocities robotChassisSpeeds) {
        var discrete = robotChassisSpeeds.discretize(0.005);
        setControl(autoRequest.withVelocity(discrete));
    }

    private void setUpAuto() {
        AutoBuilder.configure(
            this::getEstimatedPosition, this::resetPose, this::getChassisVelocities,
            (speeds, ff)
                -> autoDriveRobotRelative(speeds),
            new PPHolonomicDriveController(constants.autoPosConstants(), constants.autoRotConstants()),
            constants.config(), FieldConstants::isRed, this
        );
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * This method can be called as infrequently as you want.
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds. Note that you must use a timestamp with
     *                              an epoch since system startup (i.e., the epoch
     *                              of this timestamp is the same epoch as
     *                              {@link Utils#getCurrentTimeSeconds}). This means
     *                              that you should use
     *                              {@link Utils#getCurrentTimeSeconds} as your time
     *                              source or sync the epochs.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        table.set("timestamp", timestampSeconds);
        table.set("Utils.getCurrentTimeSeconds()=", Utils.getCurrentTimeSeconds());
        table.set("timestamp difference", timestampSeconds - Utils.getCurrentTimeSeconds());
        super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);

        lastVisionUpdatePose = visionRobotPoseMeters;
        lastVisionUpdateTime = Utils.getCurrentTimeSeconds();
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * This method can be called as infrequently as you want.
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds. Note that you must use a timestamp with
     *                                 an epoch since system startup (i.e., the epoch
     *                                 of this timestamp is the same epoch as
     *                                 {@link Utils#getCurrentTimeSeconds}). This means
     *                                 that you should use
     *                                 {@link Utils#getCurrentTimeSeconds} as your time
     *                                 source or sync the epochs.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position in meters, y position
     *                                 in meters, and heading in radians). Increase
     *                                 these numbers to trust the vision pose
     *                                 measurement less.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        table.set("timestamp", timestampSeconds);
        table.set("Utils.getCurrentTimeSeconds", Utils.getCurrentTimeSeconds());
        table.set("timestamp difference", timestampSeconds - Utils.getCurrentTimeSeconds());
        super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);

        lastVisionUpdatePose = visionRobotPoseMeters;
        lastVisionUpdateTime = Utils.getCurrentTimeSeconds();
    }

    public double getLastVisionUpdateTimeMS() {
        return (double) (Utils.getCurrentTimeSeconds() - lastVisionUpdateTime) / 1000.0;
    }

    public double getVisionPoseAgreementDistance() {
        return this.getEstimatedPosition().getTranslation().getDistance(this.lastVisionUpdatePose.getTranslation());
    }

    public int getVisionPoseAgreementQuality() {
        double cm = 100 * this.getVisionPoseAgreementDistance();
        double ms = this.getLastVisionUpdateTimeMS();

        int blocks = 0;
        if (cm <= 3) blocks = 3;
        else if (cm <= 100) blocks = 2;
        else if (cm <= 200) blocks = 1;
        else blocks = 0;

        blocks -= (int) (ms / 5000);
        if (blocks < 0) blocks = 0;
        return blocks;
    }
}
