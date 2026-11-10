// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.intake;

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
import org.wpilib.epilogue.Logged;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.robot.IDs;
import first.robot.Robot;
import first.robot.util.SysIdRoutine;

@Logged
public class IntakeMechanism implements Mechanism {
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
        return run(coro -> {
                   while (!getSlapdownPosition().gte(IntakeConstants.UPPER_RELEASE_POS)) {
                       runSlapdown(IntakeConstants.LIFT_VEL);
                       coro.yield();
                   }
                   intakeSlapdown.setVoltage(0);
               }
        ).named("intake: lift");
    }

    public Command getSlapdownCommand() {
        return run(coro -> {
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

    public Trigger limitEngaged = new Trigger(() -> !hallLimit.get());

    @Utility(name = "Intake Tester", group = "Testers")
    public static class Tester implements OpMode {
        public Tester(Robot robot) {
            robot.intake.limitEngaged.onTrue(
                robot.intake.run(_ -> robot.intake.intakeSlapdown.setPosition(IntakeConstants.LIMIT_POS))
                    .named("zero slapdown")
            );

            robot.port0.rightBumper().whileTrue(robot.intake.getLiftCommand());
            robot.port0.leftBumper().whileTrue(robot.intake.getSlapdownCommand());

            // forward is up
            robot.port0.a().whileTrue(robot.intake.sysIdDynamic(SysIdRoutine.Direction.FORWARD));
            robot.port0.b().whileTrue(robot.intake.sysIdDynamic(SysIdRoutine.Direction.REVERSE));
            robot.port0.x().whileTrue(robot.intake.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD));
            robot.port0.y().whileTrue(robot.intake.sysIdQuasistatic(SysIdRoutine.Direction.REVERSE));
        }
    }
}
