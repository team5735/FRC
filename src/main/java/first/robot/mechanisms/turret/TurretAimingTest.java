package first.robot.mechanisms.turret;

import first.robot.Robot;
import first.robot.mechanisms.limelight.LimelightMechanism;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.telemetry.Telemetry;

@Utility
public class TurretAimingTest implements OpMode {
    private final TurretMechanism turret =
        new TurretMechanism(drivetrain::getEstimatedPosition, drivetrain.constants, () -> true);

    public final Telemetry logger = new Telemetry(drivetrain, turret);

    private final LimelightMechanism[] limelights = {new LimelightMechanism(drivetrain, "limelight-fone"),
                                                     new LimelightMechanism(drivetrain, "limelight-ftwo")};
    private Robot robot;

    public TurretAimingTest(Robot robot) {
        this.robot = robot;

        drivetrain.registerTelemetry(logger::telemeterize);

        turret.zeroTrigger.onTrue(turret.zeroCommand());

        drivetrain.setDefaultCommand(drivetrain.joystickDriveCommand(
            ()
                -> controller.getLeftX(),
            ()
                -> controller.getLeftY(),
            ()
                -> controller.getLeftTriggerAxis(),
            ()
                -> controller.getRightTriggerAxis(),
            () -> controller.getHID().getYButton(), () -> controller.getHID().getStartButton()));

        controller.a().onTrue(turret.holdRobotRel(Rotations.of(0.00)));
        controller.b().onTrue(turret.holdRobotRel(Rotations.of(0.75)));
        controller.rightBumper().whileTrue(turret.trackRobotRel(() -> {
            double x = controller.getRightX();
            double y = controller.getRightY();
            return new Rotation2d(-y, -x).getMeasure();
        }));
        controller.leftBumper().whileTrue(LaunchCalculator.dryAimTurret(LaunchGoal.SCORE, turret, drivetrain));

        controller.x().whileTrue(turret.zeroSequence());
        controller.povUp().whileTrue(turret.sysId());
        controller.povDown().onTrue(Commands.runOnce(turret::remakePID, turret));

        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUToPigeon();
        }
    }

    @Override
    public void teleopInit() {
        if (!turret.getZeroStatus()) {
            CommandScheduler.getInstance().schedule(turret.zeroSequence());
        }

        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUMode(3);
        }
    }

    @Override
    public void autonomousInit() {
        if (!turret.getZeroStatus()) {
            CommandScheduler.getInstance().schedule(turret.zeroSequence());
        }

        for (LimelightSubsystem limelight : limelights) {
            limelight.setIMUMode(3);
        }
    }
}
