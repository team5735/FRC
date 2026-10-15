package first.robot.turret;

import static org.wpilib.units.Units.Rotations;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.XboxController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.telemetry.TelemetryTable;

import first.robot.DrivetrainTelemetry;
import first.robot.Robot;
import first.robot.drivetrain.DevbotTunerConstants;
import first.robot.drivetrain.DrivetrainMechanism;
import first.robot.launcher.LaunchCalculator;
import first.robot.limelight.LimelightMechanism;

@Utility(name = "Turret aiming test", group = "Testers")
public class TurretAimingTest implements OpMode {
    private final Field2d field = new Field2d();
    private final TelemetryTable table = TelemetryRegistry.getTable("turret aiming test");

    private final DrivetrainMechanism drivetrain = DevbotTunerConstants.createDrivetrain();
    private final TurretMechanism turret = new TurretMechanism(drivetrain::getEstimatedPosition, () -> true);

    public final DrivetrainTelemetry logger = new DrivetrainTelemetry(drivetrain, turret);

    private final LimelightMechanism[] limelights = {
        new LimelightMechanism(drivetrain, "limelight-fone"), new LimelightMechanism(drivetrain, "limelight-ftwo")
    };

    public TurretAimingTest(Robot robot) {
        drivetrain.registerTelemetry(logger::telemeterize);

        turret.limitTrigger.and(RobotState::isDisabled).onTrue(turret.zeroSequence());

        drivetrain.setDefaultCommand(drivetrain.joystickDriveCommand(
            ()
                -> robot.port0.getLeftX(),
            ()
                -> robot.port0.getLeftY(),
            ()
                -> robot.port0.getLeftTrigger(),
            ()
                -> robot.port0.getRightTrigger(),
            () -> robot.port0.y().getAsBoolean(), () -> robot.port0.menu().getAsBoolean()
        ));

        robot.port0.a().onTrue(turret.holdRobotRel(Rotations.of(0.00)));
        robot.port0.b().onTrue(turret.holdRobotRel(Rotations.of(0.75)));
        robot.port0.rightBumper().whileTrue(turret.trackRobotRel(() -> {
            double x = robot.port0.getRightX();
            double y = robot.port0.getRightY();
            return new Rotation2d(-y, -x).getMeasure();
        }));
        robot.port0.leftBumper().whileTrue(
            LaunchCalculator.dryAimTurret(LaunchCalculator.LaunchGoal.SCORE, turret, drivetrain)
        );

        robot.port0.x().whileTrue(turret.zeroSequence());
        robot.port0.dpadUp().whileTrue(turret.sysId());
        robot.port0.dpadDown().onTrue(Command.requiring(turret).executing(_ -> turret.remakePID()).named("remake PID"));

        for (LimelightMechanism limelight : limelights) {
            limelight.setIMUToPigeon();
        }
    }

    @Override
    public void start() {
        if (!turret.isZeroed()) {
            Scheduler.getDefault().schedule(turret.zeroSequence());
        }

        for (LimelightMechanism limelight : limelights) {
            limelight.setIMUMode(3);
        }
    }
}
