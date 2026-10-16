package first.robot.drivetrain;

import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.units.measure.LinearVelocity;

public class DrivetrainConstants {
    public static final double DEFAULT_DEADBAND = 0.1;

    public static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(4);
}
