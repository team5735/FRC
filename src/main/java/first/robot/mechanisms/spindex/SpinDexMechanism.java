package first.robot.mechanisms.spindex;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import first.robot.IDs;
import first.robot.Robot;
import first.robot.util.NTable;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.TunableDouble;

public class SpinDexMechanism implements Mechanism, TelemetryLoggable {
    public final SparkFlex feedVortex = new SparkFlex(CANPort.CAN_D0, IDs.SPINDEX_FEED_VORTEX_ID, MotorType.kBrushless);
    public final SparkFlex wheelVortex =
        new SparkFlex(CANPort.CAN_D0, IDs.SPINDEX_WHEEL_VORTEX_ID, MotorType.kBrushless);

    private NTable table = NTable.root("tuning").sub("spindex");
    private TunableDouble feedVoltage = table.tunableDouble("feed", -6);
    private TunableDouble forwardVoltage = table.tunableDouble("wheel forward voltage", -4);
    private TunableDouble reverseVoltage = table.tunableDouble("wheel reverse voltage", 5);

    public SpinDexMechanism() {
        super();

        feedVortex.clearFaults();
        wheelVortex.clearFaults();
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("wheel_output", wheelVortex.getAppliedOutput());
    }

    public void runFeeder() { feedVortex.setVoltage(feedVoltage.get()); }

    public void reverseFeeder() { feedVortex.setVoltage(feedVoltage.get()); }

    public void stopFeeder() { feedVortex.setVoltage(0); }

    public void runWheel() { wheelVortex.setVoltage(forwardVoltage.get()); }

    public void stopWheel() { wheelVortex.setVoltage(0); }

    public void reverseWheel() { wheelVortex.setVoltage(reverseVoltage.get()); }

    public void run() {
        runWheel();
        runFeeder();
    }

    public void stop() {
        stopWheel();
        stopFeeder();
    }

    public Command getRun() {
        return run(_ -> {
                   runWheel();
                   runFeeder();
               })
            .whenCanceled(() -> {
                stopWheel();
                stopFeeder();
            })
            .named("run spindex");
    }

    public Command getRunSupplier(Supplier<Double> wheel, Supplier<Double> feeder) {
        return run(_ -> {
                   wheelVortex.setVoltage(wheel.get());
                   feedVortex.setVoltage(feeder.get());
               })
            .whenCanceled(() -> {
                stopWheel();
                stopFeeder();
            })
            .named("run spindex");
    }

    public Command getInformedRun(BooleanSupplier isValid) {
        return run(coro -> {
                   while (true) {
                       if (isValid.getAsBoolean())
                           runWheel();
                       else
                           stopWheel();
                       coro.yield();
                   }
               })
            .whenCanceled(() -> {
                stopWheel();
                stopFeeder();
            })
            .named("spindex informed run");
    }

    @Utility(name = "Spindex Tester", group = "Testers")
    public static class Tester implements OpMode {
        private final SpinDexMechanism spindex = new SpinDexMechanism();
        private Robot robot;

        public Tester(Robot robot) {
            this.robot = robot;
            robot.port0.a().whileTrue(spindex.getRun());
        }
    }
}
