package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import first.robot.Robot;
import first.robot.constants.CANIds;
import first.robot.util.NTable;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.drive.RobotDriveBase.MotorType;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;

public class SpinDexMechanism implements Mechanism {
    public final SparkFlex feedVortex = new SparkFlex(CANIds.SPINDEX_FEED_VORTEX_ID, MotorType.kBrushless);
    public final SparkFlex wheelVortex = new SparkFlex(CANIds.SPINDEX_WHEEL_VORTEX_ID, MotorType.kBrushless);
    private NTable table = NTable.root("tuning").sub("spindex");

    public SpinDexMechanism() {
        super();

        feedVortex.clearFaults();
        wheelVortex.clearFaults();

        // ensure these values are in NT and persistent
        // they can be changed for tuning purposes, but...
        table.ensure("feed", -6);
        table.ensure("wheel: fwd", -4);
        table.ensure("wheel: bck", 5);

        // they should default to these values on robot start
        table.set("feed", -6);
        table.set("wheel: fwd", -4);
        table.set("wheel: bck", 5);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("spindex/wheel_output", wheelVortex.getAppliedOutput());
    }

    public void runFeeder() { feedVortex.setVoltage(table.getDouble("feed")); }

    public void reverseFeeder() { feedVortex.setVoltage(-table.getDouble("feed")); }

    public void stopFeeder() { feedVortex.setVoltage(0); }

    public double getForwardVoltage() { return table.getDouble("wheel: fwd"); }

    public void runWheel() { wheelVortex.setVoltage(table.getDouble("wheel: fwd")); }

    public void stopWheel() { wheelVortex.setVoltage(0); }

    public void reverseWheel() { wheelVortex.setVoltage(table.getDouble("wheel: bck")); }

    public Command getRun() {
        return startEnd(
            ()
                -> {
                runWheel();
                runFeeder();
            },
            () -> {
                stopWheel();
                stopFeeder();
            });
    }

    public Command getRunSupplier(Supplier<Double> wheel, Supplier<Double> feeder) {
        return startEnd(
            ()
                -> {
                wheelVortex.setVoltage(wheel.get());
                feedVortex.setVoltage(feeder.get());
            },
            () -> {
                stopWheel();
                stopFeeder();
            });
    }

    public Command getInformedRun(BooleanSupplier isValid) {
        return runEnd(
                   ()
                       -> {
                       if (isValid.getAsBoolean()) {
                           runWheel();
                       } else {
                           stopWheel();
                       }
                   },
                   () -> {
                       stopWheel();
                       stopFeeder();
                   })
            .beforeStarting(this::runFeeder);
    }

    public Command getBackwards() {
        return startEnd(
            ()
                -> {
                reverseWheel();
                reverseFeeder();
            },
            () -> {
                stopWheel();
                stopFeeder();
            });
    }

    @Utility
    public static class Tester implements OpMode {
        private final SpinDexMechanism spindex = new SpinDexMechanism();
        private Robot robot;

        public Tester(Robot robot) {
            this.robot = robot;
            robot.port0.a().whileTrue(spindex.getRun());
        }
    }
}
