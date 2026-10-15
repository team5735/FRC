package first.robot.drivetrain;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Mass;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

public interface RobotConstants {
    public PIDConstants autoPosConstants();
    public PIDConstants autoRotConstants();
    public double rotKs();
    public double rotKv();
    public double rotKa();
    public double openDriveKv();
    public double openDriveKa();
    public Distance robotTotalWidth();
    public Distance robotTotalLength();
    public Distance pigeonToRobotFront();
    public PathConstraints pathFollowConstraints();
    public LinearVelocity defaultSpeed();
    public LinearVelocity slowSpeed();
    public LinearVelocity turboSpeed();
    public AngularVelocity defaultRotationalRate();
    public AngularVelocity slowRotationalRate();
    public AngularVelocity turboRotationalRate();
    public Mass robotMass();
    public Distance drivetrainWidth();
    public double robotMoiKgxMxM();
    public double coefficientOfFriction();
    public RobotConfig config();
    public Translation2d pigeonToCenterOfRotation();
    public Translation2d robotToTurretCenter();
    public Distance bumperWidth();
}
