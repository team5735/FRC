package first.robot;

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

public record RobotConstants(
    PIDConstants autoPosConstants,

    PIDConstants autoRotConstants,

    double rotKs,

    double rotKv,

    double rotKa,

    double openDriveKv,

    double openDriveKa,

    Distance robotTotalWidth,

    Distance robotTotalLength,

    Distance pigeonToRobotFront,

    PathConstraints pathFollowConstraints,

    LinearVelocity defaultSpeed,

    LinearVelocity slowSpeed,

    LinearVelocity turboSpeed,

    AngularVelocity defaultRotationalRate,

    AngularVelocity slowRotationalRate,

    AngularVelocity turboRotationalRate,

    Mass robotMass,

    Distance drivetrainWidth,

    double robotMoiKgxMxM,

    double coefficientOfFriction,

    RobotConfig config,

    Translation2d pigeonToCenterOfRotation,

    Translation2d robotToTurretCenter,

    Distance bumperWidth
) {
}
