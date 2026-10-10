// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

import first.robot.mechanisms.turret.TurretMechanism;

@Teleop
public class TeleOp implements OpMode {
    private Robot robot;

    private TurretMechanism turret = new TurretMechanism(() -> null, () -> true);

    public TeleOp(Robot robot) { this.robot = robot; }

    private void driverBindings() {}
}
