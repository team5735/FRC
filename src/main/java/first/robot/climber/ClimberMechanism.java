package first.robot.climber;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;

import first.robot.IDs;
import first.robot.Robot;

public class ClimberMechanism implements Mechanism, TelemetryLoggable {
    public final TalonFX talon = new TalonFX(IDs.CLIMB_TALON_ID);

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

    @Utility
    public static class Tester implements OpMode {
        ClimberMechanism climber = new ClimberMechanism();

        public Tester(Robot robot) {
            robot.port0.rightTrigger()
                .whileTrue(climber.run(_ -> climber.extend()).named("extend"))
                .onFalse(climber.run(_ -> climber.stop()).named("stop"));
            robot.port0.leftTrigger()
                .whileTrue(climber.run(_ -> climber.retract()).named("retract"))
                .onFalse(climber.run(_ -> climber.stop()).named("stop"));
        }

        @Override
        public void periodic() {
            climber.checkLimitSwitches();
        }
    };
}
