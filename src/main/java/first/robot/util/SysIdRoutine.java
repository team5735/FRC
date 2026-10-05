package first.robot.util;

import static org.wpilib.sysid.SysIdRoutineLog.State;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.system.Timer;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Velocity;
import org.wpilib.units.measure.Voltage;

/** CommandsV3 version of SysIdRoutine */
public class SysIdRoutine {
    private Config config;
    private Mechanism mechanism;
    private SysIdRoutineLog logger;

    public SysIdRoutine(Config config, Mechanism mechanism, SysIdRoutineLog logger) {
        this.config = config;
        this.mechanism = mechanism;
        this.logger = logger;
    }

    public SysIdRoutine(Config config, Mechanism mechanism) {
        this(config, mechanism, new SysIdRoutineLog(config.name));
    }

    public static enum Direction {
        FORWARD(-1),
        REVERSE(1);

        int sign;

        Direction(int sign) { this.sign = sign; }
    }

    public static record Config(Velocity<VoltageUnit> rampRate, Voltage stepVoltage, Time timeout,
                                Consumer<State> recordState, Consumer<? super Voltage> setOutput,
                                Consumer<SysIdRoutineLog> logger, String name) {}

    public void quasistaticRun(Direction direction, Coroutine coro, BooleanSupplier until) {
        double outputSign = direction == Direction.FORWARD ? 1.0 : -1.0;
        State state = direction == Direction.FORWARD ? State.QUASISTATIC_FORWARD : State.QUASISTATIC_REVERSE;

        Timer timer = new Timer();
        timer.start();

        while (!until.getAsBoolean()) {
            Voltage voltage = (Voltage)(this.config.rampRate.times(Seconds.of(timer.get())).times(outputSign));
            this.config.setOutput.accept(voltage);
            this.config.logger.accept(this.logger);
            this.config.recordState.accept(state);

            if (timer.get() > this.config.timeout.in(Seconds)) {
                return;
            }

            coro.yield();
        }
    }

    public Command quasistatic(Direction direction, String name) {
        return this.mechanism.run(coro -> quasistaticRun(direction, coro, () -> false))
            .whenCanceled(() -> onCancel(direction))
            .named(name);
    }

    public void dynamicRun(Direction direction, Coroutine coro, BooleanSupplier until) {
        Voltage output = this.config.stepVoltage.times(direction.sign);
        State state = direction == Direction.FORWARD ? State.DYNAMIC_FORWARD : State.DYNAMIC_REVERSE;

        while (!until.getAsBoolean()) {
            this.config.setOutput.accept(output);
            this.config.logger.accept(this.logger);
            this.config.recordState.accept(state);

            coro.yield();
        }
    }

    // Same function for both dynamic and quasistatic.
    public void onCancel(Direction direction) {
        this.config.setOutput.accept(Volts.of(0));
        this.config.recordState.accept(State.NONE);
    }

    public Command dynamic(Direction direction, String name) {
        return this.mechanism.run(coro -> dynamicRun(direction, coro, () -> false))
            .whenCanceled(() -> onCancel(direction))
            .named(name);
    }
}
