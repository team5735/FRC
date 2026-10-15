package first.robot.turret;

import static org.wpilib.units.Units.Rotations;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.telemetry.Telemetry;

import first.robot.Robot;
import first.robot.limelight.LimelightMechanism;

@Utility(name = "Turret aiming test", group = "Testers")
public class TurretAimingTest implements OpMode {
    private final TurretMechanism turret =
        new TurretMechanism(drivetrain::getEstimatedPosition, drivetrain.constants, () -> true);

    public final Telemetry logger = new Telemetry(drivetrain, turret);

    private final LimelightMechanism[] limelights = {
        new LimelightMechanism(drivetrain, "limelight-fone"), new LimelightMechanism(drivetrain, "limelight-ftwo")
    };
    private Robot robot;

    public TurretAimingTest(Robot robot) {
        this.robot = robot;

        drivetrain.registerTelemetry(logger::telemeterize);

        turret.zeroTrigger.onTrue(turret.zeroCommand());

        drivetrain.setDefaultCommand(drivetrain.joystickDriveCommand(
            ()
                -> robot.port0.getLeftX(),
            ()
                -> robot.port0.getLeftY(),
            ()
                -> robot.port0.getLeftTriggerAxis(),
            ()
                -> robot.port0.getRightTriggerAxis(),
            () -> robot.port0.getHID().getYButton(), () -> robot.port0.getHID().getStartButton()
        ));

        robot.port0.a().onTrue(turret.holdRobotRel(Rotations.of(0.00)));
        robot.port0.b().onTrue(turret.holdRobotRel(Rotations.of(0.75)));
        robot.port0.rightBumper().whileTrue(turret.trackRobotRel(() -> {
            double x = robot.port0.getRightX();
            double y = robot.port0.getRightY();
            return new Rotation2d(-y, -x).getMeasure();
        }));
        robot.port0.leftBumper().whileTrue(LaunchCalculator.dryAimTurret(LaunchGoal.SCORE, turret, drivetrain));

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
