package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class LauncherConstants {
    public static final double RPM_TOLERANCE = 35; // 35 is 1% of expected RPM

    public static final double KP = 0.003;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double KS = 0.2;
    public static final double KV = 0.00084853;

    public static final double GEARING = 36. / 15; // input 36 tooth pulley, output 15

    public static final int LAUNCHER_VOLTS = 4;

    public static final AngularVelocity DEFAULT_SETPOINT = RPM.of(3000);

    public static final double BANGBANG_THRESHOLD = 0.99;
}
