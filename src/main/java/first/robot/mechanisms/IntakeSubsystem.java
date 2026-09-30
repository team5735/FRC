// Currently, there is no 2027 library for CTRE devices.
/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.PartialRobot;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeSlapdown = new TalonFX(Constants.INTAKE_SLAPDOWN_TALONFX_ID);
    private final TalonFX intakeRoller = new TalonFX(Constants.INTAKE_ROLLER_TALONFX_ID);
    private final ArmFeedforward ff = new ArmFeedforward(IntakeConstants.KS, IntakeConstants.KG, IntakeConstants.KV);
    private final DigitalInput hallLimit = new DigitalInput(Constants.INTAKE_LIMIT_PIN);

    public IntakeSubsystem() {
        super();
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 10;
        intakeRoller.getConfigurator().apply(rollerConfig);

        TalonFXConfiguration slapdownConfig = new TalonFXConfiguration();
        slapdownConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        slapdownConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // TODO swap back to brake
        slapdownConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_REDUCTION;
        intakeSlapdown.getConfigurator().apply(slapdownConfig);

        intakeSlapdown.setPosition(IntakeConstants.START_POS);
    }

    public void forwardRoll() {
        intakeRoller.setVoltage(IntakeConstants.ROLL_IN_VOLTS);
    }

    public void reverseRoll() {
        intakeRoller.setVoltage(IntakeConstants.ROLL_OUT_VOLTS);
    }

    public void stopRoll() {
        intakeRoller.setVoltage(0);
    }

    private void runSlapdown(AngularVelocity velocity) {
        intakeSlapdown.setVoltage(ff.calculate(getSlapdownPosition().in(Radians), velocity.in(RotationsPerSecond)));
    }

    public boolean isAtPosition(Angle position) {
        return getSlapdownPosition().isNear(position, IntakeConstants.ANGULAR_TOLERANCE);
    }

    public Angle getSlapdownPosition() {
        return intakeSlapdown.getPosition().getValue();
    }

    public Command getIntakeForwardRollCommand() {
        return startEnd(() -> forwardRoll(), () -> stopRoll());
    }

    public Command getIntakeReverseRollCommand() {
        return startEnd(() -> reverseRoll(), () -> stopRoll());
    }

    public Command getLiftCommand() {
        return runEnd(() -> runSlapdown(IntakeConstants.LIFT_VEL), () -> intakeSlapdown.setVoltage(0))
                .until(() -> getSlapdownPosition().gte(IntakeConstants.UPPER_RELEASE_POS));
    }

    public Command getSlapdownCommand() {
        return runEnd(() -> runSlapdown(IntakeConstants.SLAPDOWN_VEL), () -> intakeSlapdown.setVoltage(0))
                .until(() -> getSlapdownPosition().lte(IntakeConstants.LOWER_RELEASE_POS));
    }
    public Command getStopRollCommand() {
        return Commands.runOnce(() -> stopRoll());
    }
    public Command zeroSlapdownPosition() {
        return Commands.runOnce(() -> intakeSlapdown.setPosition(IntakeConstants.LIMIT_POS)).ignoringDisable(true);
    }

    public SysIdRoutine routine = new SysIdRoutine(
            new Config(Volts.of(0.1).per(Second), Volts.of(0.5), null),
            new Mechanism(v -> intakeSlapdown.setVoltage(v.in(Volts)), log -> {
                log.motor("slapdown").angularPosition(getSlapdownPosition())
                        .angularVelocity(intakeSlapdown.getVelocity().getValue()).voltage(intakeSlapdown.getMotorVoltage().getValue());
            }, this));

    public Command sysIdQuasistatic(Direction direction){
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction){
        return routine.dynamic(direction);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake/limit_engaged", !hallLimit.get());
        SmartDashboard.putNumber("intake/slapdown_volts", intakeSlapdown.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("intake/slapdown_pos_deg", getSlapdownPosition().in(Degrees));
        SmartDashboard.putNumber("intake/slapdown_vel_dps", intakeSlapdown.getVelocity().getValue().in(DegreesPerSecond));
    }

    public Trigger limitEngaged = new Trigger(() -> !hallLimit.get());

    public static class Tester extends PartialRobot {
        private final IntakeSubsystem intake = new IntakeSubsystem();

        public Tester() {
            super();
            intake.limitEngaged.onTrue(intake.zeroSlapdownPosition());
            
            controller.rightBumper().whileTrue(intake.getLiftCommand());
            controller.leftBumper().whileTrue(intake.getSlapdownCommand());

            controller.a().whileTrue(intake.sysIdDynamic(Direction.kForward)); // forward = up
            controller.b().whileTrue(intake.sysIdDynamic(Direction.kReverse));
            controller.x().whileTrue(intake.sysIdQuasistatic(Direction.kForward));
            controller.y().whileTrue(intake.sysIdQuasistatic(Direction.kReverse));
        }
    }
}

 */   
