package first.robot.drivetrain;

import static org.wpilib.units.Units.DegreesPerSecond;
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

public class CompbotConstants implements RobotConstants {
    public PIDConstants autoPosConstants() { return new PIDConstants(12.5, 0); }  // TODO

    public PIDConstants autoRotConstants() { return new PIDConstants(10, 0); }  // TODO

    public double rotKs() { return 0.17285; }

    public double rotKv() { return 4.4734; }

    public double rotKa() { return 0.39386; }

    public double openDriveKv() { return 1.6983; }

    public double openDriveKa() { return 0.24867; }

    public Distance robotTotalWidth() { return Inches.of(25).plus(bumperWidth().times(2)); }

    public Distance robotTotalLength() { return Inches.of(29.5).plus(bumperWidth().times(2)); }

    public Distance pigeonToRobotFront() { return robotTotalLength().div(2); }

    public PathConstraints pathFollowConstraints() {
        return new PathConstraints(
            MetersPerSecond.of(2), MetersPerSecondPerSecond.of(0.5), RotationsPerSecond.of(0.25),
            RotationsPerSecondPerSecond.of(0.25)
        );
    }

    public LinearVelocity defaultSpeed() { return MetersPerSecond.of(3); }

    public LinearVelocity slowSpeed() { return MetersPerSecond.of(0.6); }

    public LinearVelocity turboSpeed() { return MetersPerSecond.of(5); }

    public AngularVelocity defaultRotationalRate() { return DegreesPerSecond.of(180); }

    public AngularVelocity slowRotationalRate() { return DegreesPerSecond.of(27); }

    public AngularVelocity turboRotationalRate() { return DegreesPerSecond.of(270); }

    public Mass robotMass() { return Kilograms.of(56.85); }

    public Distance drivetrainWidth() { return Inches.of(29.5); }

    public double robotMoiKgxMxM() {
        return robotMass().in(Kilograms) * drivetrainWidth().in(Meters) / 2 * rotKa() /
            CompbotTunerConstants.DEFAULT_DRIVE_CONSTANTS.kA;
    }

    public double coefficientOfFriction() { return 1.5; }

    private final RobotConfig config = new RobotConfig(
        robotMass().in(Kilograms), robotMoiKgxMxM(),
        new ModuleConfig(
            CompbotTunerConstants.WHEEL_RADIUS.in(Meters), CompbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond),
            coefficientOfFriction(), DCMotor.getKrakenX60(1).withReduction(CompbotTunerConstants.DRIVE_GEAR_RATIO), 60,
            1
        ),
        new Translation2d(CompbotTunerConstants.FRONT_LEFT_XPOS, CompbotTunerConstants.FRONT_LEFT_YPOS),
        new Translation2d(CompbotTunerConstants.FRONT_RIGHT_XPOS, CompbotTunerConstants.FRONT_RIGHT_YPOS),
        new Translation2d(CompbotTunerConstants.BACK_LEFT_XPOS, CompbotTunerConstants.BACK_LEFT_YPOS),
        new Translation2d(CompbotTunerConstants.BACK_RIGHT_XPOS, CompbotTunerConstants.BACK_RIGHT_YPOS)
    );

    public RobotConfig config() { return config; }

    public Translation2d pigeonToCenterOfRotation() { return new Translation2d(Inches.of(0), Inches.of(0)); }

    public Translation2d robotToTurretCenter() { return new Translation2d(Inches.of(-4.245), Inches.of(6.5)); }

    public Distance bumperWidth() {
        return Inches.of(3.5);  // TODO
    }
}
