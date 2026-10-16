// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.intake;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.robot.IDs;
import first.robot.Robot;
import first.robot.util.SysIdRoutine;

public class IntakeMechanism implements Mechanism, TelemetryLoggable {
    private final TalonFX intakeSlapdown = new TalonFX(IDs.INTAKE_SLAPDOWN_TALONFX_ID, IDs.DEFAULT_CANBUS);
    private final TalonFX intakeRoller = new TalonFX(IDs.INTAKE_ROLLER_TALONFX_ID, IDs.DEFAULT_CANBUS);
    private final ArmFeedforward ff = new ArmFeedforward(IntakeConstants.KS, IntakeConstants.KG, IntakeConstants.KV);
    private final DigitalInput hallLimit = new DigitalInput(IDs.INTAKE_LIMIT_PIN);

    public IntakeMechanism() {
        super();
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 10;
        intakeRoller.getConfigurator().apply(rollerConfig);

        TalonFXConfiguration slapdownConfig = new TalonFXConfiguration();
        slapdownConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        slapdownConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;  // TODO swap back to brake
        slapdownConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_REDUCTION;
        intakeSlapdown.getConfigurator().apply(slapdownConfig);

        intakeSlapdown.setPosition(IntakeConstants.START_POS);
    }

    public void forwardRoll() { intakeRoller.setVoltage(IntakeConstants.ROLL_IN_VOLTS); }

    public void reverseRoll() { intakeRoller.setVoltage(IntakeConstants.ROLL_OUT_VOLTS); }

    public void stopRoll() { intakeRoller.setVoltage(0); }

    private void runSlapdown(AngularVelocity velocity) {
        intakeSlapdown.setVoltage(ff.calculate(getSlapdownPosition().in(Radians), velocity.in(RotationsPerSecond)));
    }

    public boolean isAtPosition(Angle position) {
        return getSlapdownPosition().isNear(position, IntakeConstants.ANGULAR_TOLERANCE);
    }

    public Angle getSlapdownPosition() { return intakeSlapdown.getPosition().getValue(); }

    public Command getLiftCommand() {
        return run(
                   coro -> {
                       while (!getSlapdownPosition().gte(IntakeConstants.UPPER_RELEASE_POS)) {
                           runSlapdown(IntakeConstants.LIFT_VEL);
                           coro.yield();
                       }
                       intakeSlapdown.setVoltage(0);
                   }
        ).named("intake: lift");
    }

    public Command getSlapdownCommand() {
        return run(
                   coro -> {
                       while (!getSlapdownPosition().gte(IntakeConstants.LOWER_RELEASE_POS)) {
                           runSlapdown(IntakeConstants.SLAPDOWN_VEL);
                           coro.yield();
                       }
                       intakeSlapdown.setVoltage(0);
                   }
        ).named("intake: slapdown");
    }

    public SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.1).per(Second), Volts.of(0.5), null, null,
            v
            -> intakeSlapdown.setVoltage(v.in(Volts)),
            log
            -> {
                log.motor("slapdown")
                    .angularPosition(getSlapdownPosition())
                    .angularVelocity(intakeSlapdown.getVelocity().getValue())
                    .voltage(intakeSlapdown.getMotorVoltage().getValue());
            },
            "intake"
        ),
        this
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction, "intake quasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction, "intake dynamic");
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("limit_engaged", !hallLimit.get());
        table.log("slapdown_volts", intakeSlapdown.getMotorVoltage().getValueAsDouble());
        table.log("slapdown_pos_deg", getSlapdownPosition().in(Degrees));
        table.log("slapdown_vel_dps", intakeSlapdown.getVelocity().getValue().in(DegreesPerSecond));
    }

    public Trigger limitEngaged = new Trigger(() -> !hallLimit.get());

    @Utility
    public static class Tester implements OpMode {
        private final IntakeMechanism intake = new IntakeMechanism();

        public Tester(Robot robot) {
            intake.limitEngaged.onTrue(
                intake.run(_ -> intake.intakeSlapdown.setPosition(IntakeConstants.LIMIT_POS)).named("zero slapdown")
            );

            robot.port0.rightBumper().whileTrue(intake.getLiftCommand());
            robot.port0.leftBumper().whileTrue(intake.getSlapdownCommand());

            robot.port0.a().whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.FORWARD));  // forward = up
            robot.port0.b().whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.REVERSE));
            robot.port0.x().whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD));
            robot.port0.y().whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.REVERSE));
        }
    }
}
