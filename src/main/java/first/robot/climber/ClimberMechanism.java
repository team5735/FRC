package first.robot.climber;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;

import first.robot.IDs;
import first.robot.Robot;

public class ClimberMechanism implements Mechanism, TelemetryLoggable {
    public final TalonFX talon = new TalonFX(IDs.CLIMB_TALON_ID, IDs.DEFAULT_CANBUS);

    private final DigitalInput limitDown = new DigitalInput(IDs.CLIMB_UPPER_LIMIT_PIN);
    private final DigitalInput limitUp = new DigitalInput(IDs.CLIMB_LOWER_LIMIT_PIN);

    private boolean canMoveUp;
    private boolean canMoveDown;
    private boolean overridden;

    public ClimberMechanism() {
        talon.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public void retract() {
        if (canMoveUp) talon.setVoltage(ClimberConstants.RETRACT_VOLTS);
        else talon.setVoltage(0);
    }

    public void extend() {
        if (canMoveDown) talon.setVoltage(ClimberConstants.EXTEND_VOLTS);
        else talon.setVoltage(0);
    }

    public void setOverride(boolean newVal) { overridden = newVal; }

    public void stop() {
        talon.setVoltage(0);
        overridden = false;
    }

    public boolean isAtDownLimit() { return !limitDown.get(); }

    public boolean isAtUpLimit() { return !limitUp.get(); }

    public void checkLimitSwitches() {
        // DIO
        canMoveUp = limitDown.get();
        canMoveDown = limitUp.get();

        if (!canMoveDown && talon.getMotorVoltage().getValueAsDouble() < 0 && !overridden) {
            talon.setVoltage(0);
        } else if (!canMoveUp && talon.getMotorVoltage().getValueAsDouble() > 0 && !overridden) {
            talon.setVoltage(0);
        }
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("limit_up", limitUp.get());
        table.log("limit_down", limitDown.get());
        table.log("current", talon.getSupplyCurrent().getValueAsDouble());
        table.log("velocity", talon.getVelocity().getValueAsDouble());
        table.log("forward_limit", talon.getForwardLimit().getValueAsDouble());
        table.log("reverse_limit", talon.getReverseLimit().getValueAsDouble());
    }

    @Utility(name = "Climber Tester", group = "Testers")
    public static class Tester implements OpMode {
        private Robot robot;

        public Tester(Robot robot) {
            this.robot = robot;
            robot.port0.rightTrigger()
                .whileTrue(robot.climber.run(_ -> robot.climber.extend()).named("extend"))
                .onFalse(robot.climber.run(_ -> robot.climber.stop()).named("stop"));
            robot.port0.leftTrigger()
                .whileTrue(robot.climber.run(_ -> robot.climber.retract()).named("retract"))
                .onFalse(robot.climber.run(_ -> robot.climber.stop()).named("stop"));
        }

        @Override
        public void periodic() {
            robot.climber.checkLimitSwitches();
        }
    };
}
