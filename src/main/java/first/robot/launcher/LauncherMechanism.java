// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.launcher;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.BangBangController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.units.measure.AngularVelocity;

import java.util.function.Supplier;

import first.robot.IDs;
import first.robot.Robot;
import first.robot.intake.IntakeMechanism;
import first.robot.spindex.SpinDexMechanism;
import first.robot.util.NTable;
import first.robot.util.SysIdRoutine;

public class LauncherMechanism implements Mechanism, TelemetryLoggable {
    private final TalonFX krakenLeft = new TalonFX(IDs.LAUNCHER_LEFT_KRAKEN_ID, IDs.DEFAULT_CANBUS);
    private final TalonFX krakenRight = new TalonFX(IDs.LAUNCHER_RIGHT_KRAKEN_ID, IDs.DEFAULT_CANBUS);

    private final BangBangController bangbang = new BangBangController();
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(LauncherConstants.KS, LauncherConstants.KV);

    private final NTable table = NTable.root("launcher");

    private double setpoint = 0;

    public LauncherMechanism() {
        this.table.set("shooter_volts", LauncherConstants.LAUNCHER_VOLTS);
        krakenLeft.getConfigurator().apply(new MotorOutputConfigs()
                                               .withInverted(InvertedValue.CounterClockwise_Positive)
                                               .withNeutralMode(NeutralModeValue.Coast));
        krakenLeft.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(30)));
        krakenRight.setControl(new Follower(IDs.LAUNCHER_LEFT_KRAKEN_ID, MotorAlignmentValue.Opposed));
        krakenRight.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(30)));

        table.ensure("!! threshold", LauncherConstants.BANGBANG_THRESHOLD);
        table.ensure("!! mult", 5);
        table.set("rpm", 3000.0);
    }

    public double getRPM() {
        this.table.set("motorRPS", krakenLeft.getVelocity().getValueAsDouble());
        return krakenLeft.getVelocity().getValue().in(RPM) * LauncherConstants.GEARING;
    }

    public double getTargetRPM() { return this.setpoint; }

    private void setTargetRPM(double rpm) {
        this.setpoint = rpm;
        bangbang.setSetpoint(rpm * LauncherConstants.BANGBANG_THRESHOLD);
        bangbang.setTolerance(rpm * 0.01);
    }

    public void usePID() {
        double output = (bangbang.getSetpoint() != 0)
                            ? bangbang.calculate(getRPM()) * table.getDouble("!! mult") + ff.calculate(setpoint)
                            : 0;
        this.table.set("output", output);
        krakenLeft.setVoltage(output);
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("speed rpm", getRPM());
        table.log("setpoint", setpoint);
        table.log("!! setpoint", bangbang.getSetpoint());
    }

    public Command getResting() { return run(_ -> krakenLeft.setVoltage(0)).named("launcher: do not"); }

    public void stop() { krakenLeft.setVoltage(0); }

    public void retunePID() {
        bangbang.setSetpoint(setpoint * table.getDouble("!! threshold"));
        bangbang.setTolerance(setpoint * 0.01);
    }

    public Command getLaunchFuel(AngularVelocity speed) {
        return run(
                   coro -> {
                       retunePID();
                       setTargetRPM(speed.in(RPM));

                       while (true) {
                           usePID();
                           coro.yield();
                       }
                   }
        ).named("launcher: with rpm=" + speed);
    }

    private final TunableDouble rpmTunable = table.tunableDouble("rpm");

    public Command getLaunchFuelNT() {
        return run(
                   coro -> {
                       retunePID();
                       setTargetRPM(this.rpmTunable.get());

                       while (true) {
                           usePID();
                           coro.yield();
                       }
                   }
        ).named("launcher: with rpm=" + this.table.getPath() + "/rpm");
    }

    public Command getLaunchFuelSupplier(Supplier<AngularVelocity> rpmSupplier) {
        return run(
                   coro -> {
                       retunePID();
                       setTargetRPM(rpmSupplier.get().in(RPM));

                       while (true) {
                           usePID();
                           coro.yield();
                       }
                   }
        ).named("launcher: with rpm from supplier");
    }

    public Command getFedLaunch(SpinDexMechanism spindex, AngularVelocity speed) {
        return run(coro -> {
                   retunePID();
                   setTargetRPM(speed.in(RPM));

                   boolean hasHitSetpoint = false;
                   while (true) {
                       if (!hasHitSetpoint && this.atSetpoint()) hasHitSetpoint = true;

                       usePID();
                       if (hasHitSetpoint) spindex.run();
                       coro.yield();
                   }
               })
            .whenCanceled(spindex::stop)
            .named("launcher: fed launch");
    }

    @SuppressWarnings("unused")
    private SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second), Volts.of(4), null, null,
            v
            -> krakenLeft.setVoltage(v.in(Volts)),
            log
            -> {
                log.motor("launcher")
                    .voltage(krakenLeft.getMotorVoltage().getValue())
                    .angularVelocity(RPM.of(getRPM()))
                    .angularPosition(krakenLeft.getPosition().getValue());
            },
            "launcher"
        ),
        this
    );

    public boolean atSetpoint() { return getRPM() > bangbang.getSetpoint(); }

    @Utility
    public static class Tester implements OpMode {
        private final SpinDexMechanism spindex = new SpinDexMechanism();
        private final LauncherMechanism launcher = new LauncherMechanism();
        private final IntakeMechanism intake = new IntakeMechanism();

        public Tester(Robot robot) {
            launcher.setDefaultCommand(launcher.getLaunchFuel(RPM.of(0)));

            robot.port0.a().whileTrue(launcher.getFedLaunch(spindex, RPM.of(3125)));
            robot.port0.x()
                .onTrue(spindex.run(_ -> spindex.reverse()).named("spindex: reverse"))
                .onFalse(spindex.run(_ -> spindex.stop()).named("spindex: stop"));
            robot.port0.rightBumper()
                .onTrue(intake.run(_ -> intake.forwardRoll()).named("intake: roll"))
                .onFalse(intake.run(_ -> intake.stopRoll()).named("intake: stop"));
        }
    }
}
