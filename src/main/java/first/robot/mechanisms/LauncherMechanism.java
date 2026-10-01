// Currently, there is no 2027 library for CTRE devices.
/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.math.controller.BangBangController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.wpilibj.smartdashboard.SmartDashboard;
import org.wpilib.wpilibj2.command.Command;
import org.wpilib.wpilibj2.command.SubsystemBase;
import org.wpilib.wpilibj2.command.sysid.SysIdRoutine;
import org.wpilib.wpilibj2.command.sysid.SysIdRoutine.Config;
import org.wpilib.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.wpilib.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.PartialRobot;
import frc.robot.constants.Constants;
import frc.robot.constants.LauncherConstants;
import frc.robot.util.NTable;

public class LauncherSubsystem extends SubsystemBase {
    private final TalonFX krakenLeft = new TalonFX(Constants.LAUNCHER_LEFT_KRAKEN_ID);
    private final TalonFX krakenRight = new TalonFX(Constants.LAUNCHER_RIGHT_KRAKEN_ID);

    private final BangBangController bangbang = new BangBangController();
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(LauncherConstants.KS,
            LauncherConstants.KV);

    private final NTable table = NTable.root("launcher");

    private double setpoint = 0;

    public LauncherSubsystem() {
        super();
        SmartDashboard.putNumber("shooter_volts", LauncherConstants.LAUNCHER_VOLTS);
        krakenLeft.getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));
        krakenLeft.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(30)));
        krakenRight.setControl(new Follower(Constants.LAUNCHER_LEFT_KRAKEN_ID, MotorAlignmentValue.Opposed));
        krakenRight.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(30)));

        table.ensure("!! threshold", LauncherConstants.BANGBANG_THRESHOLD);
        table.ensure("!! mult", 5);
        table.set("rpm", 3000.0);
    }

    public double getRPM() {
        SmartDashboard.putNumber("launcher/motorRPS", krakenLeft.getVelocity().getValueAsDouble());
        return krakenLeft.getVelocity().getValue().in(RPM) * LauncherConstants.GEARING;
    }

    public double getTargetRPM(){
        return this.setpoint;
    }
    
    private void setTargetRPM(double rpm) {
        this.setpoint = rpm;
        bangbang.setSetpoint(rpm * LauncherConstants.BANGBANG_THRESHOLD);
        bangbang.setTolerance(rpm * 0.01);
    }

    public void usePID() {
        double output = (bangbang.getSetpoint() != 0)
                ? bangbang.calculate(getRPM()) * table.getDouble("!! mult") + ff.calculate(setpoint)
                : 0;
        SmartDashboard.putNumber("launcher/output", output);
        krakenLeft.setVoltage(output);
    }

    @Override
    // Multiplying krakenMotor by 60 to turn rotations per second into rotations per
    // minute (RPM)
    public void periodic() {
        double currentRPM = getRPM();
        this.table.set("speed rpm", currentRPM);
        this.table.set("setpoint", setpoint);
        this.table.set("!! setpoint", bangbang.getSetpoint());
    }

    public Command getResting() {
        return runOnce(() -> krakenLeft.setVoltage(0)).withName("zero launcher voltage");
    }

    public void stop() {
        krakenLeft.setVoltage(0);
    }

    public void retunePID() {
        bangbang.setSetpoint(setpoint * table.getDouble("!! threshold"));
        bangbang.setTolerance(setpoint * 0.01);
    }

    public Command getLaunchFuel(AngularVelocity speed) {
        return runOnce(this::retunePID)
                .andThen(startRun(() -> setTargetRPM(speed.in(RPM)), this::usePID))
                .withName("Launch Fuel at " + speed);
    }

    public Command getLaunchFuelSupplier(Supplier<Double> supplier) {
        return runOnce(this::retunePID)
                .andThen(startRun(() -> setTargetRPM(supplier.get()),
                        this::usePID))
                .withName("Launch Fuel /tuning/rpm");
    }

    public Command getLaunchFuelNT() {
        return runOnce(this::retunePID)
                .andThen(startRun(() -> setTargetRPM(table.getDouble("rpm")),
                        this::usePID))
                .withName("Launch Fuel /tuning/rpm");
    }

    public Command getDynamicLaunch(Supplier<AngularVelocity> speedSupplier) {
        return startRun(this::retunePID, () -> {
            setTargetRPM(speedSupplier.get().in(RPM));
            usePID();
        }).withName("Dynamic Launching");
    }

    public Command getFedLaunch(SpinDexSubsystem spindex, AngularVelocity speed) {
        return this.getLaunchFuel(speed).until(bangbang::atSetpoint)
                .andThen(this.getLaunchFuel(speed).alongWith(spindex.getRun()));
    }

    private SysIdRoutine routine = new SysIdRoutine(
            new Config(Volts.of(0.5).per(Second), Volts.of(4), null, null),
            new Mechanism(v -> krakenLeft.setVoltage(v.in(Volts)), log -> {
                log.motor("launcher").voltage(krakenLeft.getMotorVoltage().getValue())
                        .angularVelocity(RPM.of(getRPM())).angularPosition(krakenLeft.getPosition().getValue());
            }, this));

    public Command sysIdDynamic(Direction direction) {
        return routine.dynamic(direction);
    }

    public Command sysIdQuasistatic(Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command runAtSpeed(AngularVelocity speed) {
        return startRun(() -> bangbang.setSetpoint(speed.in(RPM)), this::usePID);
    }

    public boolean atSetpoint() {
        return getRPM() > bangbang.getSetpoint();
    }

    public static class Tester extends PartialRobot {
        private final SpinDexSubsystem spindex = new SpinDexSubsystem();
        private final LauncherSubsystem launcher = new LauncherSubsystem();
        private final IntakeSubsystem intake = new IntakeSubsystem();

        public Tester() {
            super();
            launcher.setDefaultCommand(launcher.getLaunchFuel(RPM.of(0)));

            controller.a().whileTrue(launcher.getFedLaunch(spindex, RPM.of(3125)));
            controller.x().whileTrue(spindex.startEnd(spindex::reverseWheel, spindex::stopWheel));
            controller.rightBumper().whileTrue(intake.getIntakeForwardRollCommand());
        }
    }
}
*/
