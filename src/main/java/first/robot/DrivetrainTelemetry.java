package first.robot;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

import java.util.Arrays;

import first.robot.drivetrain.DrivetrainConstants;
import first.robot.drivetrain.DrivetrainMechanism;
import first.robot.turret.TurretMechanism;
import first.robot.util.NTable;

public class DrivetrainTelemetry {
    // Mechanisms to represent the swerve module states
    private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    // A direction and length changing ligament for speed representation
    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    // A direction changing and length constant ligament for module direction
    private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
        moduleMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        moduleMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        moduleMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        moduleMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
    };

    private final double[] poseArray = new double[3];
    private final double[] moduleStatesArray = new double[8];
    private final double[] moduleTargetsArray = new double[8];

    public static final Field2d field = new Field2d();

    private DrivetrainMechanism drivetrain;
    private TurretMechanism turret;

    private final NTable ntable = NTable.root().sub("telemetry");
    private final NTable stateNTable = ntable.sub("drive state");
    private final NTable moduleNTable = ntable.sub("modules");

    private final TelemetryTable stateTable = stateNTable.telemetryTable();

    public DrivetrainTelemetry(DrivetrainMechanism drivetrain, TurretMechanism turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        field.getRobotObject().setPose(new Pose2d());
        ntable.set("field", field);
    }

    public void telemeterize(SwerveDriveState state) {
        // Telemeterize the swerve drive state
        stateTable.log("pose", state.Pose);
        stateTable.log("speeds", state.Velocity);
        stateTable.log("module states", state.ModulePositions);
        stateTable.log("module states", state.ModuleVelocities);
        stateTable.log("module targets", state.ModuleTargets);
        stateTable.log("module positions", state.ModulePositions);
        stateTable.log("timestamp", state.Timestamp);
        stateTable.log("odometry period", state.OdometryPeriod);
        stateTable.log("odometry frequency", 1.0 / state.OdometryPeriod);

        // Also write to log file
        poseArray[0] = state.Pose.getX();
        poseArray[1] = state.Pose.getY();
        poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            moduleStatesArray[i * 2 + 0] = state.ModuleVelocities[i].angle.getRadians();
            moduleStatesArray[i * 2 + 1] = state.ModuleVelocities[i].velocity;
            moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
            moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].velocity;
        }

        field.setRobotPose(this.drivetrain.getEstimatedPosition());
        field.getObject("turret_pose").setPose(this.turret.getMechanismPose());

        var modules = this.drivetrain.getModules();
        NTable[] tables =
            Arrays.stream(new String[] {"FL", "FR", "BL", "BR"}).map(s -> moduleNTable.sub(s)).toArray(NTable[] ::new);
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            TelemetryTable table = tables[i].telemetryTable();

            table.log(
                "steer position", Units.radiansToRotations(
                                      MathUtil.angleModulus(module.getCurrentVelocity().angle.getMeasure().in(Radians))
                                  )
            );
            table.log(
                "steer setpoint", Units.radiansToRotations(
                                      MathUtil.angleModulus(module.getTargetVelocity().angle.getMeasure().in(Radians))
                                  )
            );
            table.log("drive velocity", Math.abs(state.ModuleVelocities[i].velocity));
            table.log("drive voltage", module.getDriveMotor().getMotorVoltage().getValueAsDouble());
            table.log("steer voltage", module.getSteerMotor().getMotorVoltage().getValueAsDouble());

            moduleSpeeds[i].setAngle(state.ModuleVelocities[i].angle);
            moduleDirections[i].setAngle(state.ModuleVelocities[i].angle);
            moduleSpeeds[i].setLength(
                state.ModuleVelocities[i].velocity / (2 * DrivetrainConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond))
            );

            table.log("mechanism", moduleMechanisms[i]);
        }
    }
}
