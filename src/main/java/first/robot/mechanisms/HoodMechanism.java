package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.discrete.AnalogInput;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.shape.Rectangle2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import first.robot.constants.PinIds;
import frc.robot.PartialRobot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HoodConstants;

public class HoodMechanism implements Mechanism {
    private final Servo servo = new Servo(PinIds.HOOD_SERVO_PIN);
    private final AnalogInput feedback = new AnalogInput(PinIds.HOOD_FEEDBACK_PIN);

    private Supplier<Pose2d> turretPoseSupplier;
    private Rectangle2d[] exclusionZones;
    private double exclusionZoneSavedServoPosition;

    /**
     * Linear interpolation. Returns the interpolated value of query point xq
     * against the line
     * defined by (x0, y0) and (x1, y1).
     * Requirements:
     * x0 does not have to be less than x1
     * x0 must not equal x1
     */

    public static double interp1(double x0, double x1, double y0, double y1, double xq) {
        if (x0 == x1) {
            throw new IllegalArgumentException("x1 and x2 cannot be equal for interpolation");
        }
        return y0 + (xq - x0) * (y1 - y0) / (x1 - x0);
    }

    public HoodMechanism(Supplier<Pose2d> turretPoseSupplier, Rectangle2d[] exclusionZones) {
        super();
        this.turretPoseSupplier = turretPoseSupplier;
        this.exclusionZones = exclusionZones;

        // makes the feedback more stable by averaging 2^4=16 samples
        // this.feedback.setAverageBits(4);
    }

    public double getServoSetpoint() {
        return servo.get();
    }

    public void setServoPosition(double pos) {
        // todo: should log warning if incoming pos is out of range
        pos = MathUtil.clamp(pos, 0.0, 1.0);
        this.servo.set(pos);
        this.sendTelemetry();
    }

    public double getHoodPosition() {
        return interp1(
                HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.HIGHEST_SERVO_POSITION,
                0.0, 1.0,
                this.getServoSetpoint());
    }

    public void setHoodPosition(double hoodPosition) {
        double servoPosition = interp1(
                0, 1,
                HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.HIGHEST_SERVO_POSITION,
                hoodPosition);
        this.setServoPosition(servoPosition);
    }

    public double getHoodAngle() {
        return interp1(
                HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.HIGHEST_SERVO_POSITION,
                HoodConstants.LOWEST_ANGLE_DEGREES, HoodConstants.HIGHEST_ANGLE_DEGREES,
                this.getServoSetpoint());
    }

    public void setHoodAngle(double hoodAngleDegrees) {
        double servoPosition = interp1(
                HoodConstants.LOWEST_ANGLE_DEGREES, HoodConstants.HIGHEST_ANGLE_DEGREES,
                HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.HIGHEST_SERVO_POSITION,
                hoodAngleDegrees);

        this.setServoPosition(servoPosition);
    }

    public void exzSaveServoPosition() {
        this.exclusionZoneSavedServoPosition = this.servo.get();
    }

    public double exzGetSavedServoPosition() {
        return exclusionZoneSavedServoPosition;
    }

    // Returns raw voltage from analog feedback wire
    public double getVoltage() {
        return feedback.getVoltage();
    }

    // Converts voltage (0-5V) into 0.0–1.0 normalized position
    public double getNormalizedPosition() {
        double v = this.getVoltage();
        return interp1(
                HoodConstants.SERVO_VOLTAGE_AT_REF0, HoodConstants.SERVO_VOLTAGE_AT_REF1,
                HoodConstants.SERVO_VOLTAGE_REF0, HoodConstants.SERVO_VOLTAGE_REF1,
                v);
    }

    public double getNormalizedAngle() {
        return interp1(HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.HIGHEST_SERVO_POSITION,
                HoodConstants.LOWEST_ANGLE_DEGREES, HoodConstants.HIGHEST_ANGLE_DEGREES,
                getNormalizedPosition());
    }

    public void sendTelemetry() {
        SmartDashboard.putNumber("hood/hood_position", this.getHoodPosition());
        SmartDashboard.putNumber("hood/servo_position", this.getServoSetpoint());
        SmartDashboard.putNumber("hood/servo_feedback_voltage", this.getVoltage());
        SmartDashboard.putNumber("hood/servo_feedback_position", getNormalizedPosition());
        SmartDashboard.putNumber("hood/hood_angle_degrees", this.getHoodAngle());
    }

    public boolean isInExclusionZone() {
        for (Rectangle2d r : exclusionZones) {
            if (r.contains(turretPoseSupplier.get().getTranslation()))
                return true;
        }
        return false;
    }

    public Command getDynamicTracking(Supplier<Angle> angleSupplier) {
        return run(() -> setHoodAngle(angleSupplier.get().in(Degrees)));
    }
    public Command setHoodPosition180(double hoodPosition) {
        return runOnce(() -> setHoodPosition(hoodPosition));
    }

    @Override
    public void periodic() {
        this.sendTelemetry();
    }

    // This is a full robot config for testing the hood subsystem
    public static class Tester extends PartialRobot {
        private final HoodMechanism hood = new HoodMechanism(() -> new Pose2d(),
                FieldConstants.HOOD_EXCLUSION_ZONES);

        private double lastPos = 0.6;

        public Tester() {
            super();

            controller.y().onTrue(hood.runOnce(() -> hood.setHoodPosition(1.0)));
            controller.b().onTrue(hood.runOnce(() -> hood.setHoodPosition(0.0)));

            controller.x().onTrue(hood.runOnce(() -> {
                lastPos += 0.025;
                lastPos = MathUtil.clamp(lastPos, 0.0, 1.0);
                hood.setServoPosition(lastPos);
            }));
            controller.a().onTrue(hood.runOnce(() -> {
                lastPos -= 0.025;
                lastPos = MathUtil.clamp(lastPos, 0.0, 1.0);
                hood.setServoPosition(lastPos);
            }));
        }

        @Override
        public void robotPeriodic() {
            this.hood.sendTelemetry();
            super.robotPeriodic();
        }

    };
}
