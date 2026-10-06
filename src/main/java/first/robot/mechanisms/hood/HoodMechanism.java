package first.robot.mechanisms.hood;

import first.robot.IDs;
import first.robot.Robot;
import first.robot.mechanisms.hood.HoodConstants;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.discrete.AnalogInput;
import org.wpilib.hardware.discrete.PWM;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.shape.Rectangle2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;

public class HoodMechanism implements Mechanism, TelemetryLoggable {
    private final PWM servo = new PWM(IDs.HOOD_SERVO_PIN);
    private final AnalogInput feedback = new AnalogInput(IDs.HOOD_FEEDBACK_PIN);

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

    public double getServoSetpoint() { return servo.getPulseTimeMicroseconds(); }

    public void setServoPosition(double pos) {
        // todo: should log warning if incoming pos is out of range
        pos = Math.clamp(pos, 0.0, 1.0);
        this.servo.setPulseTimeMicroseconds((int)(pos * 4096));
    }

    public double getHoodPosition() {
        return interp1(HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.HIGHEST_SERVO_POSITION, 0.0, 1.0,
                       this.getServoSetpoint());
    }

    public void setHoodPosition(double hoodPosition) {
        double servoPosition =
            interp1(0, 1, HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.HIGHEST_SERVO_POSITION, hoodPosition);
        this.setServoPosition(servoPosition);
    }

    public double getHoodAngle() {
        return interp1(HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.HIGHEST_SERVO_POSITION,
                       HoodConstants.LOWEST_ANGLE_DEGREES, HoodConstants.HIGHEST_ANGLE_DEGREES,
                       this.getServoSetpoint());
    }

    public void setHoodAngle(double hoodAngleDegrees) {
        double servoPosition =
            interp1(HoodConstants.LOWEST_ANGLE_DEGREES, HoodConstants.HIGHEST_ANGLE_DEGREES,
                    HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.HIGHEST_SERVO_POSITION, hoodAngleDegrees);

        this.setServoPosition(servoPosition);
    }

    public void exzSaveServoPosition() {
        this.exclusionZoneSavedServoPosition =
            this.servo.getPulseTimeMicroseconds() / (double)HoodConstants.PWM_US_RANGE;
    }

    public double exzGetSavedServoPosition() { return exclusionZoneSavedServoPosition; }

    // Returns raw voltage from analog feedback wire
    public double getVoltage() { return feedback.getVoltage(); }

    // Converts voltage (0-5V) into 0.0–1.0 normalized position
    public double getNormalizedPosition() {
        double v = this.getVoltage();
        return interp1(HoodConstants.SERVO_VOLTAGE_AT_REF0, HoodConstants.SERVO_VOLTAGE_AT_REF1,
                       HoodConstants.SERVO_VOLTAGE_REF0, HoodConstants.SERVO_VOLTAGE_REF1, v);
    }

    public double getNormalizedAngle() {
        return interp1(HoodConstants.LOWEST_SERVO_POSITION, HoodConstants.HIGHEST_SERVO_POSITION,
                       HoodConstants.LOWEST_ANGLE_DEGREES, HoodConstants.HIGHEST_ANGLE_DEGREES,
                       getNormalizedPosition());
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("position", this.getHoodPosition());
        table.log("angle_degrees", this.getHoodAngle());
        table.log("servo/position", this.getServoSetpoint());
        table.log("servo/feedback_voltage", this.getVoltage());
        table.log("servo/feedback_position", getNormalizedPosition());
    }

    public boolean isInExclusionZone() {
        for (Rectangle2d r : exclusionZones) {
            if (r.contains(turretPoseSupplier.get().getTranslation()))
                return true;
        }
        return false;
    }

    @Utility(name = "Hood Tester", group = "Testers")
    public static class Tester implements OpMode {
        private final HoodMechanism hood = new HoodMechanism(() -> new Pose2d(), FieldConstants.HOOD_EXCLUSION_ZONES);

        private double lastPos = 0.6;

        private Robot robot;

        public Tester(Robot robot) {
            super();
            this.robot = robot;

            robot.port0.y().onTrue(hood.run(_ -> hood.setHoodPosition(1.0)).named("set hood position to 1.0"));
            robot.port0.b().onTrue(hood.run(_ -> hood.setHoodPosition(0.0)).named("set hood position to 0.0"));

            robot.port0.x().onTrue(hood.run(_ -> {
                                           lastPos += 0.025;
                                           lastPos = Math.clamp(lastPos, 0.0, 1.0);
                                           hood.setServoPosition(lastPos);
                                       })
                                       .named("nudge up"));
            robot.port0.a().onTrue(hood.run(_ -> {
                                           lastPos -= 0.025;
                                           lastPos = Math.clamp(lastPos, 0.0, 1.0);
                                           hood.setServoPosition(lastPos);
                                       })
                                       .named("nudge down"));
        }
    };
}
