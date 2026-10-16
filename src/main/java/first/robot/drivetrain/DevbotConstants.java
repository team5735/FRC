package first.robot.drivetrain;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Mass;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

public class DevbotConstants implements RobotConstants {
    public PIDConstants autoPosConstants() { return new PIDConstants(15, 0); }  // TODO

    public PIDConstants autoRotConstants() { return new PIDConstants(15, 0); }  // TODO

    public double rotKs() { return 0.082104; }

    public double rotKv() { return 7.1488; }

    public double rotKa() { return 0.28448; }

    public double openDriveKv() { return 2.37; }

    public double openDriveKa() { return 0.09; }

    public Distance robotTotalWidth() { return Inches.of(30).plus(bumperWidth().times(2)); }

    public Distance robotTotalLength() { return Inches.of(30).plus(bumperWidth().times(2)); }

    public Distance pigeonToRobotFront() { return robotTotalLength().div(2); }

    public PathConstraints pathFollowConstraints() {
        return new PathConstraints(
            MetersPerSecond.of(2), MetersPerSecondPerSecond.of(0.5), RotationsPerSecond.of(0.25),
            RotationsPerSecondPerSecond.of(0.25)
        );
    }

    public LinearVelocity defaultSpeed() { return MetersPerSecond.of(2); }

    public LinearVelocity slowSpeed() { return MetersPerSecond.of(.5); }

    public LinearVelocity turboSpeed() { return MetersPerSecond.of(3); }

    public AngularVelocity defaultRotationalRate() { return RotationsPerSecond.of(0.25); }

    public AngularVelocity slowRotationalRate() { return RotationsPerSecond.of(0.0625); }

    public AngularVelocity turboRotationalRate() { return RotationsPerSecond.of(0.5); }

    public Mass robotMass() { return Kilograms.of(28.00); }

    public Distance drivetrainWidth() { return Inches.of(30); }

    public double robotMoiKgxMxM() {
        return robotMass().in(Kilograms) * drivetrainWidth().in(Meters) / 2 * rotKa() / openDriveKa();
    }

    public double coefficientOfFriction() { return 1.5; }

    private final RobotConfig config = new RobotConfig(
        robotMass().in(Kilograms), robotMoiKgxMxM(),
        new ModuleConfig(
            DevbotTunerConstants.WHEEL_RADIUS.in(Meters), DevbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond),
            coefficientOfFriction(), DCMotor.getKrakenX60(1).withReduction(DevbotTunerConstants.DRIVE_GEAR_RATIO), 60, 1
        ),
        new Translation2d(DevbotTunerConstants.FL_XPOS, DevbotTunerConstants.FL_YPOS),
        new Translation2d(DevbotTunerConstants.FR_XPOS, DevbotTunerConstants.FR_YPOS),
        new Translation2d(DevbotTunerConstants.BL_XPOS, DevbotTunerConstants.BL_YPOS),
        new Translation2d(DevbotTunerConstants.BR_XPOS, DevbotTunerConstants.BR_YPOS)
    );

    public RobotConfig config() { return config; }

    public Translation2d pigeonToCenterOfRotation() { return new Translation2d(Inches.of(0), Inches.of(0)); }

    public Translation2d robotToTurretCenter() { return new Translation2d(Inches.of(-7.75), Inches.of(-7.75)); }

    public Distance bumperWidth() { return Inches.of(3.5); }
}
