// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.math.geometry.Pose2d;

import first.robot.climber.ClimberMechanism;
import first.robot.hood.HoodMechanism;
import first.robot.intake.IntakeMechanism;
import first.robot.launcher.LauncherMechanism;
import first.robot.spindex.SpinDexMechanism;
import first.robot.turret.TurretMechanism;

public class Robot extends OpModeRobot {
    public final CommandXboxController port0 = new CommandXboxController(0);

    public final SpinDexMechanism spindex = new SpinDexMechanism();
    public final LauncherMechanism launcher = new LauncherMechanism();
    public final IntakeMechanism intake = new IntakeMechanism();
    public final ClimberMechanism climber = new ClimberMechanism();
    public final HoodMechanism hood = new HoodMechanism(() -> new Pose2d(), FieldConstants.HOOD_EXCLUSION_ZONES);
    public final TurretMechanism turret = new TurretMechanism(() -> Pose2d.ZERO, () -> true);

    public Robot() {}

    @Override
    public void robotPeriodic() {
        Scheduler.getDefault().run();
    }
}
