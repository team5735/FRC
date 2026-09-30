// Currently, there is no 2027 library for CTRE devices.
/*
package frc.robot.subsystems;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.discrete.DigitalInput;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import first.robot.constants.CANIds;
import frc.robot.PartialRobot;
import frc.robot.constants.ClimberConstants;

public class ClimberMechanism implements Mechanism {
    public final TalonFX talon = new TalonFX(CANIds.CLIMB_TALON_ID);

    private final DigitalInput limitDown = new DigitalInput(Constants.CLIMB_UPPER_LIMIT_PIN);
    private final DigitalInput limitUp = new DigitalInput(Constants.CLIMB_LOWER_LIMIT_PIN);
    private boolean canMoveUp;
    private boolean canMoveDown;
    private boolean overriden;

    public ClimberMechanism() {
        super();
        talon.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public Command getExtendCommand() {
        return run(() -> extend()).finallyDo(a -> stop());
    }

    public Command getRetractCommand() {
        return run(() -> retract()).finallyDo(a -> stop());
    }

    public Command getExtendOverrideCommand() {
        return run(() -> extendOverride()).finallyDo(a -> stop());
    }

    public Command getRetractOverrideCommand() {
        return run(() -> retractOverride()).finallyDo(a -> stop());
    }

    public void retract() {
        if (canMoveUp) {
            talon.setVoltage(ClimberConstants.RETRACT_VOLTS);
        } else {
            talon.setVoltage(0);
        }
    }

    public void extend() {
        if (canMoveDown) {
            talon.setVoltage(ClimberConstants.EXTEND_VOLTS);
        } else {
            talon.setVoltage(0);
        }
    }

    public Command getFullyExtendCommand() {
        return run(() -> extend()).until(this::isAtUpLimit).finallyDo(() -> stop());
    }

    public Command getFullyDetractCommand() {
        return run(() -> retract()).until(this::isAtDownLimit).finallyDo(() -> stop());
    }

    public void retractOverride() {
        overriden = true;
        talon.setVoltage(ClimberConstants.RETRACT_VOLTS);
    }

    public void extendOverride() {
        overriden = true;
        talon.setVoltage(ClimberConstants.EXTEND_VOLTS);
    }

    public void stop() {
        talon.setVoltage(0);
        overriden = false;
    }

    public boolean isAtDownLimit() {
        return !limitDown.get();
    }

    public boolean isAtUpLimit() {
        return !limitUp.get();
    }

    @Override
    public void periodic() {
        // DIO
        canMoveUp = limitDown.get();
        canMoveDown = limitUp.get();

        if (!canMoveDown && talon.getMotorVoltage().getValueAsDouble() < 0 && !overriden) {
            talon.setVoltage(0);
        } else if (!canMoveUp && talon.getMotorVoltage().getValueAsDouble() > 0 && !overriden) {
            talon.setVoltage(0);
        }

        SmartDashboard.putBoolean("limitUp", limitUp.get());
        SmartDashboard.putBoolean("limitDown", limitDown.get());

        double current = talon.getSupplyCurrent().getValueAsDouble();
        SmartDashboard.putNumber("Climber/Current", current);

        double velocity = talon.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Climber/Velocity", velocity);

        double forwardLimit = talon.getForwardLimit().getValueAsDouble();
        SmartDashboard.putNumber("Climber/ForwardLimit", forwardLimit);

        double reverseLimit = talon.getReverseLimit().getValueAsDouble();
        SmartDashboard.putNumber("Climber/ReverseLimit", reverseLimit);
    }

    public static class Tester extends PartialRobot {

        ClimberMechanism climber = new ClimberMechanism();

        public Tester() {
            super();
            controller.rightTrigger().whileTrue(climber.getExtendCommand());
            controller.leftTrigger().whileTrue(climber.getRetractCommand());
        }
    };
}
*/
