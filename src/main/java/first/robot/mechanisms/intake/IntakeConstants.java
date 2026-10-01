package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
    public static final double ROLL_IN_VOLTS = 8.0;
    public static final double ROLL_OUT_VOLTS = -6.0;

    public static final double KS = 0.12;
    public static final double KV = 6.584;
    public static final double KG = 0.2;

    public static final Angle START_POS = Degrees.of(110);
    public static final Angle LIMIT_POS = Degrees.of(105);
    public static final Angle UPPER_RELEASE_POS = Degrees.of(105);
    public static final Angle LOWER_RELEASE_POS = Degrees.of(15);

    public static final Angle ANGULAR_TOLERANCE = Degrees.of(1);

    public static final AngularVelocity LIFT_VEL = DegreesPerSecond.of(60);
    public static final AngularVelocity SLAPDOWN_VEL = DegreesPerSecond.of(-60);

    public static final double GEAR_REDUCTION = 61.25;
}
