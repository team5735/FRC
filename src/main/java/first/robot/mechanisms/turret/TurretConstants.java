package first.robot.mechanisms.turret;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;

import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.State;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;

public class TurretConstants {
    public static final double GEAR_REDUCTION = 200 / 20;  // initial gear 20T, secondary gear 200T

    // PID gains (with motion profiling)
    public static final double KP = 8.5;
    public static final double KI = 0;
    public static final double KD = 0;

    // Simple Feedforward gains, all roughly tuned due to SysID misbehavior
    public static final double KS = 0.35;
    public static final double KV = 0.75;
    public static final double KA = 0.4;

    public static final AngularVelocity MAX_VEL = RotationsPerSecond.of(1.5);
    public static final AngularAcceleration MAX_ACC = RotationsPerSecondPerSecond.of(7.5);
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAX_VEL.in(RotationsPerSecond), MAX_ACC.in(RotationsPerSecondPerSecond));

    // Imaginary, robot-relative hard stops for the turret; all setpoints are to be
    // within a set padding of these

    // TEMP!!!
    public static final Angle REVERSE_LIMIT_BOT_REL = Rotations.of(0.58);    // limit the turret would hit whild driving
                                                                             // CW
    public static final Angle FORWARD_LIMIT_BOT_REL = Rotations.of(0.3045);  // limit the turret would hit while driving
                                                                             // CCW
    public static final Angle HALL_LIMIT_POS_BOT_REL = Rotations.of(0.302);
    public static final Angle ZERO_OFFSET =
        Rotations.of((REVERSE_LIMIT_BOT_REL.in(Rotations) + FORWARD_LIMIT_BOT_REL.in(Rotations)) / 2);

    public static final Angle REVERSE_LIMIT_TUR_REL = REVERSE_LIMIT_BOT_REL.minus(ZERO_OFFSET);
    public static final Angle FORWARD_LIMIT_TUR_REL = Rotations.of(1).minus(REVERSE_LIMIT_TUR_REL);

    public static final Angle SOFT_PADDING = Rotations.of(0.05);
    public static final Angle MAX_DECEL_PADDING = Rotations.of(
        MAX_VEL.in(RotationsPerSecond) *MAX_VEL.in(RotationsPerSecond) / (MAX_ACC.in(RotationsPerSecondPerSecond) * 2)
    );

    public static final Angle TOLERANCE = Degrees.of(2);  // Maximum tolerance of +-2º (based on field geometry)
    public static final Angle DYNAMIC_TOLERANCE = Degrees.of(5);

    // Ideal robot-relative starting angle
    public static final Angle START_POS_BOT_REL = Rotations.of(0.25);

    public static final Angle CLIMB_POS_BOT_REL = Rotations.of(0.92);
}
