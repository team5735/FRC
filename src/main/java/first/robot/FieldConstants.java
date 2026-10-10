package first.robot;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import java.util.Optional;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.shape.Rectangle2d;
import org.wpilib.units.measure.Distance;

// This file provides positions, orientations, distances, and poses to field elements for
// the REBUILT 2026 competition playing field.
// These are based on the AndyMark field specifications

public class FieldConstants {
    public static boolean isRed() {
        Optional<Alliance> alliance = MatchState.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.RED) {
            return true;
        }
        return false;
    }

    private static Distance inch(double inches) {
        return Inches.of(inches);
    }

    /**
     * Gets the red-alliance equivalent of the given blue field element.
     * The returned dimensions are STILL in wpiBlue coords
     *
     * @return red-alliance field element{@link Translation2d}
     */
    public static Translation2d redElement(Translation2d blueElement) {
        return FAR_CORNER.minus(blueElement);
    }

    /**
     * Gets the red-alliance equivalent of the given blue field element.
     * The returned dimensions are STILL in wpiBlue coords
     *
     * @return red-alliance field element{@link Translation2d}
     */
    public static Pose2d redElement(Pose2d blueElement) {
        return new Pose2d(redElement(blueElement.getTranslation()),
                blueElement.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    public static Rectangle2d redElement(Rectangle2d blueElement) {
        return new Rectangle2d(redElement(blueElement.getCenter()),
                blueElement.getXWidth(), blueElement.getYWidth());
    }

    public static Translation2d alliance(Translation2d element) {
        if (isRed()) {
            return redElement(element);
        }
        return element;
    }

    public static Pose2d alliance(Pose2d element) {
        if (isRed()) {
            return redElement(element);
        }
        return element;
    }

    public static Rectangle2d alliance(Rectangle2d element) {
        if (isRed()) return redElement(element);
        else return element;
    }

    /**
     * Contains all 32 AprilTags in wpiBlue coordinates.
     *
     * <p>
     * Access the tags with {@link AprilTagFieldLayout#getTags} and
     * {@link AprilTagFieldLayout#getTagPose}
     */
    public static final Field FIELD = Fields.DEFAULT_FIELD.loadField();

    // Dimensions of the arena, got from running FieldMapTest in the tests folder,
    // obtained from the official AprilTagFieldLayout k2026RebuiltAndymark built
    // into wpilib
    // The terms length and width (and height and depth) are ambiguous.
    // Instead of using those, we say the long side "length" of the field is
    // length_x and the short side "width" of the field is length_y.
    // All of our dimensions will be length_x, length_y, and length_z in wpiblue
    // field coordinates meaning downfield is in the x- direction, across the short
    // side of the field is y, and up in the air is z.
    public static final Distance FIELD_LENGTH_X = Meters.of(FIELD.getFieldLength());
    public static final Distance FIELD_LENGTH_Y = Meters.of(FIELD.getFieldWidth());

    public static final Translation2d FAR_CORNER = new Translation2d(FIELD_LENGTH_X, FIELD_LENGTH_Y);
    public static final Translation2d FIELD_CENTER = FAR_CORNER.div(2);

    // from
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    // NOTE: these are in the wpiBlue coordinate frame.
    // Use the function redElement() to get the corresponding red alliance element
    // the coordinates for all elements are ALWAYS in the wpiBlue coordinate frame
    // Red "RIGHT" elements are red alliance right ie viewed from the red alliance
    // side
    // So, when viewed from the blue side, the red element will be on the left
    // 4.61 ; 4.02
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(inch(181.56), inch(158.32));
    public static final Translation2d BLUE_OUTPOST_CENTER = new Translation2d(inch(0), inch(25.62));

    //////////////////////
    // TRENCH CONSTANTS //
    //////////////////////
    public static final Translation2d BLUE_TRENCH_RIGHT_CENTER = new Translation2d(inch(181.56), inch(24.97));
    public static final Translation2d BLUE_TRENCH_LEFT_CENTER = new Translation2d(inch(181.56),
            inch(FIELD_LENGTH_Y.in(Inches) - 24.97));
    public static final Translation3d TRENCH_DIMENSION = new Translation3d(
            inch(44.4), // TODO: incorrect, setting to RAMP X length, because the trench is just a bar
            inch(2 * 24.97), // how wide it is to fit a bot through
            inch(22.25)); // height to the ceiling of the tunnel

    ///////////////////////////////////////
    // TRENCH / HOOD DOWN EXLUSION ZONES //
    ///////////////////////////////////////
    // The trench centers (above) are the centers of our rectangle exclusion zone
    // we simply define the extent of the rectangle as being some
    // percent bigger than the trench itself
    public static final Rectangle2d HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT = new Rectangle2d(
            new Pose2d(BLUE_TRENCH_LEFT_CENTER, Rotation2d.ZERO),
            2.25 * TRENCH_DIMENSION.getX(), 1.75 * TRENCH_DIMENSION.getY());

    public static final Rectangle2d HOOD_DOWN_EXCLUSION_BLUE_TRENCH_RIGHT = new Rectangle2d(
            new Pose2d(BLUE_TRENCH_RIGHT_CENTER, Rotation2d.ZERO),
            2.25 * TRENCH_DIMENSION.getX(), 1.75 * TRENCH_DIMENSION.getY());

    public static final Rectangle2d[] HOOD_EXCLUSION_ZONES = new Rectangle2d[] {
            FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT,
            FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_RIGHT,
            FieldConstants.redElement(FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_LEFT),
            FieldConstants.redElement(FieldConstants.HOOD_DOWN_EXCLUSION_BLUE_TRENCH_RIGHT),
    };

    ////////////////////
    // RAMP CONSTANTS //
    ////////////////////
    public static final Translation2d BLUE_RAMP_RIGHT_CENTER = new Translation2d(inch(181.56),
            inch(24.97 * 2 + 12 + 73 / 2.0));
    public static final Translation2d BLUE_RAMP_LEFT_CENTER = new Translation2d(inch(181.56),
            inch(FIELD_LENGTH_Y.in(Inches) - (24.97 * 2 + 12 + 73 / 2.0)));

    public static final Translation3d RAMP_DIMENSION = new Translation3d(
            inch(44.4), // the up-then-down total length of the ramp
            inch(73), // how wide it is to fit a bot through
            inch(6.513)); // ramp starts at 0 height and goes to this height in the middle

    //////////////////////
    // TOWER CONSTANTS //
    //////////////////////

    // Field location of the center-front of the climbing tower
    // This is to the front of the uprights, which are a little further ahead than
    // the rungs, and a little further back than the climbing structure floor (see
    // pdf for details)
    public static final Translation2d BLUE_TOWER_FRONT_CENTER = new Translation2d(inch(43.8), inch(146.86));
    public static final Translation3d TOWER_DIMENSION = new Translation3d(
            inch(43.8), // how much does tower stick out onto field
            inch(47.0), // from outside end-to-end of the rungs
            inch(27)); // height to center of first rung
    public static final Distance TOWER_RUNG_OD = inch(1.66); // outer diameter of each rung of the tower
    public static final int BLUE_TOWER_TAG_1 = 31; // april tag IDs associated with the towers
    public static final int BLUE_TOWER_TAG_2 = 32;
    public static final int RED_TOWER_TAG_1 = 15;
    public static final int RED_TOWER_TAG_2 = 16;

    /////////////////////////////
    // CLIMBING AUTO WAYPOINTS //
    /////////////////////////////
    // This is the height we need to get our hook over to latch onto the lowest rung
    public static final Distance CLIMBER_HOOK_MIN_Z = TOWER_DIMENSION.getMeasureZ().plus(TOWER_RUNG_OD.div(2.0))
            .plus(inch(0.25));

    // Right in front of tower, 1' away from it
    public static final Pose2d CLIMBER_BLUE_LOCALIZATION_POSE = new Pose2d(
            BLUE_TOWER_FRONT_CENTER.plus(new Translation2d(inch(30 / 2.0 + 12), inch(0))),
            Rotation2d.fromDegrees(180));

    // Center bot on rung, position 1' to left of tower
    public static final Pose2d CLIMBER_BLUE_LEFT_CLIMB_ALIGN_POSE = new Pose2d(
            BLUE_TOWER_FRONT_CENTER.plus(new Translation2d(inch(-3.51 / 2.0), // half the thickness of the
                                                                              // uprights
                    TOWER_DIMENSION.getMeasureY().div(-2.0).minus(inch(30 / 2.0 + 12)))),
            Rotation2d.fromDegrees(180));

    // Where we should be to actually do the climb
    // TODO: right now this is to center of bot, need to add offset to hook
    public static final Pose2d CLIMBER_BLUE_LEFT_CLIMB_POSE = new Pose2d(
            CLIMBER_BLUE_LEFT_CLIMB_ALIGN_POSE.getX(), // center bot on rungs
            BLUE_TOWER_FRONT_CENTER.getY() // robot Y position calculated here is relative to center of
                                           // tower
                    - TOWER_DIMENSION.getY() / 2.0 // left side of tower
                    + inch(5.875).in(Meters) / 2.0 // 5.875"=rung extent, get center of outside rung
                    - inch(30 / 2.0 + 3).in(Meters), // offset to edge of robot, and then assume
                                                     // climber is 3 inches
                                                     // inside robot
            CLIMBER_BLUE_LEFT_CLIMB_ALIGN_POSE.getRotation());

    public static final Translation2d FERRY_TARGET_1 = new Translation2d(1.25, 0.732);
    public static final Translation2d FERRY_TARGET_2 = new Translation2d(
            FERRY_TARGET_1.getX(), FIELD_LENGTH_Y.minus(FERRY_TARGET_1.getMeasureY()).in(Meters));

    public static Translation2d closestFerryTarget(Translation2d drivetrainPos) {
        Translation2d one = FERRY_TARGET_1;
        Translation2d two = FERRY_TARGET_2;
        if (isRed()) {
            one = redElement(one);
            two = redElement(two);
        }
        return drivetrainPos.getDistance(one) < drivetrainPos.getDistance(two) ? one : two;
    }
}
