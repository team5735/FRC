// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

import first.robot.drivetrain.DevbotTunerConstants;
import first.robot.drivetrain.DrivetrainMechanism;
import first.robot.turret.TurretMechanism;

@Teleop
public class DevbotTeleop implements OpMode {
    private Robot robot;

    private DrivetrainMechanism drivetrain = DevbotTunerConstants.createDrivetrain();

    private TurretMechanism turret = new TurretMechanism(drivetrain::getEstimatedPosition, () -> true);

    public DevbotTeleop(Robot robot) { this.robot = robot; }

    private void driverBindings() {}
}
