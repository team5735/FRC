package first.robot.mechanisms.hood;

public class HoodConstants {
    // To tune the hood:
    // Necessary to do any time the hood is disengaged from the servo
    // Run the Tester bot in HoodSubsystem.java
    //   In Constants.java, set CURRENT_ROBOT = BotConfiguration.HOOD;
    // With hood engaged to servo, set hood to lower position
    // look at servo_feedback_position. This is the servo setpoint as
    // estimated via the feedback wire
    // plug that value into SERVO_SETPOINT_AT_HOOD_0
    public static final double SERVO_SETPOINT_AT_HOOD_0 = 0.283;

    // This is the range of setpoints that correspond to the range of hood positions
    // from fully retracted to fully extended.
    // This is determined once and should be left alone, so long as the gear ratios
    // on the hood servo don't change.
    public static final double SERVO_SETPOINT_TO_HOOD_POS_RATIO = 0.526; // 0.75-0.224;

    // The range of servo setpoints that correspond to hood lowest
    // and highest positions. The servo can physically be set
    // between 0 and 1. But these limit that by the physical reality
    // of the hood
    // These are used in the HoodSubsystem
    // They should not be changed directly here
    public static final double LOWEST_SERVO_POSITION = SERVO_SETPOINT_AT_HOOD_0;
    public static final double HIGHEST_SERVO_POSITION = SERVO_SETPOINT_AT_HOOD_0 + SERVO_SETPOINT_TO_HOOD_POS_RATIO;

    // The range of angles (in degrees) that the physcial shooter hood can
    // move to. While tuning the hood servo, these don't change
    public static final double LOWEST_ANGLE_DEGREES = 8.0;
    public static final double HIGHEST_ANGLE_DEGREES = 45.0;
    public static final double FERRY_ANGLE = HIGHEST_ANGLE_DEGREES;

    // TODO: test
    // public static final double ANGLE_AT_ARC = 15.0; // degrees, this is current angle to shoot from arc
    public static final double ANGLE_AT_ARC = 25.0; // degrees

    // 0.45 = 1.75v
    // do this twice and you get a linear interpolation between feedback
    // voltage and servo setpoint
    public static final double SERVO_VOLTAGE_REF0 = 0.45;    // This is setpoint 0
    public static final double SERVO_VOLTAGE_AT_REF0 = 1.75; // This is feedback voltage at setpoint 0
    public static final double SERVO_VOLTAGE_REF1 = 0.775;   // This is setpoint 1
    public static final double SERVO_VOLTAGE_AT_REF1 = 1.00; // This is feedback voltage at setpoint 1

    public static final double DYNAMIC_TOLERANCE_DEGREES = 5;

    public static int PWM_US_RANGE = 4096;
}
