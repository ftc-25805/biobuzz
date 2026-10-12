package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointOdometry;

/**
 * Drives the robot to a spot on the field and turns it to face a direction,
 * using the Pinpoint to know where the robot is.
 *
 * Positions are measured from where the robot started (the spot it was in
 * when the robot was initialized):
 *     x = inches FORWARD,  y = inches LEFT,  heading = degrees turned LEFT (counter-clockwise).
 *
 * How it works: every loop, look at how far we are from the target and push
 * the robot toward it. The farther away we are, the harder we push (this is
 * called "proportional control"), so the robot slows down as it arrives.
 *
 * Typical use:
 *     navigator.setTarget(24, 0, 90);          // 24 in forward, facing left
 *     while (opModeIsActive() && !navigator.isAtTarget()) {
 *         navigator.update();                  // call every loop
 *     }
 */
public class PointToPointNavigator {

    // ---------------------------------------------------------------------
    // Tuning values. TODO: TUNE ON OUR ROBOT. Start low and work up.
    // ---------------------------------------------------------------------

    /** Drive power per inch of distance left. 0.04 = full push from 25 inches away. */
    public static final double DRIVE_KP = 0.04;
    /** Turn power per radian of angle left. 1.0 = full push from about 57 degrees away. */
    public static final double TURN_KP = 1.0;

    /** Fastest the robot may drive or turn (0.0 to 1.0). Keep low while testing! */
    public static final double MAX_DRIVE_POWER = 0.5;
    public static final double MAX_TURN_POWER = 0.4;

    /**
     * Smallest push while still moving (0.0 to 1.0). Close to the target the proportional
     * push gets so small the wheels can't overcome friction and the robot stalls short.
     * This floor keeps it creeping in. Too high and it will jitter around the target.
     */
    public static final double MIN_DRIVE_POWER = 0.12;
    public static final double MIN_TURN_POWER = 0.10;

    /** How close counts as "arrived". */
    public static final double POSITION_TOLERANCE_INCHES = 1.0;
    public static final double HEADING_TOLERANCE_DEGREES = 2.0;

    /**
     * The robot must stay within tolerance this long before a move counts as finished,
     * so we don't stop while it is still coasting past the target.
     */
    public static final double SETTLE_TIME_SECONDS = 0.1;

    private final MecanumDrive drive;
    private final PinpointOdometry odometry;

    private double targetXInches;
    private double targetYInches;
    private double targetHeadingRadians;
    private double maxDrivePower = MAX_DRIVE_POWER;
    private double maxTurnPower = MAX_TURN_POWER;
    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean wasInTolerance = false;
    private boolean atTarget = true;

    public PointToPointNavigator(MecanumDrive drive, PinpointOdometry odometry) {
        this.drive = drive;
        this.odometry = odometry;
    }

    /**
     * Sets a new place to go.
     *
     * @param xInches        inches forward of the start
     * @param yInches        inches left of the start
     * @param headingDegrees direction to face, in degrees turned left of the start
     */
    public void setTarget(double xInches, double yInches, double headingDegrees) {
        setTarget(xInches, yInches, headingDegrees, MAX_DRIVE_POWER, MAX_TURN_POWER);
    }

    /**
     * Sets a new place to go, with speed limits just for this move.
     * Use a low limit for precise moves and a high one for long, fast travel.
     *
     * @param maxDrive fastest the robot may drive on this move (0.0 to 1.0)
     * @param maxTurn  fastest the robot may turn on this move (0.0 to 1.0)
     */
    public void setTarget(double xInches, double yInches, double headingDegrees,
                          double maxDrive, double maxTurn) {
        maxDrivePower = maxDrive;
        maxTurnPower = maxTurn;
        wasInTolerance = false;
        targetXInches = xInches;
        targetYInches = yInches;
        targetHeadingRadians = Math.toRadians(headingDegrees);
        atTarget = false;
    }

    /** Call every loop while moving. Reads the Pinpoint and drives toward the target. */
    public void update() {
        odometry.update();

        double heading = odometry.getHeadingRadians();

        // How far we still have to go, on the FIELD.
        double errorX = targetXInches - odometry.getXInches();
        double errorY = targetYInches - odometry.getYInches();
        double distance = Math.hypot(errorX, errorY);
        double headingError =
                AngleUnit.normalizeRadians(targetHeadingRadians - heading);

        boolean inTolerance = distance <= POSITION_TOLERANCE_INCHES
                && Math.abs(Math.toDegrees(headingError)) <= HEADING_TOLERANCE_DEGREES;
        if (inTolerance) {
            // Close enough. Stop pushing and wait to make sure we STAY close enough.
            if (!wasInTolerance) {
                settleTimer.reset();
            }
            wasInTolerance = true;
            drive.stop();
            atTarget = settleTimer.seconds() >= SETTLE_TIME_SECONDS;
            return;
        }
        wasInTolerance = false;
        atTarget = false;

        // Push toward the target: harder when far, softer when close, but never under the
        // minimum (so we keep creeping in) or over the max. If we are already close enough
        // in position, don't push at all. We only need to finish turning.
        boolean positionDone = distance <= POSITION_TOLERANCE_INCHES;
        double power = positionDone
                ? 0.0 : clamp(DRIVE_KP * distance, MIN_DRIVE_POWER, maxDrivePower);

        // The error is measured on the field, but the wheels need it measured from the
        // robot's point of view, so rotate it by the direction the robot is facing.
        double forwardAmount = errorX * Math.cos(heading) + errorY * Math.sin(heading);
        double leftAmount = -errorX * Math.sin(heading) + errorY * Math.cos(heading);

        // Turn the amounts into a direction. (Skip this when basically on the spot, so
        // we don't divide by zero.)
        double forward = 0.0;
        double right = 0.0;
        if (distance > 1e-6) {
            forward = power * forwardAmount / distance;
            right = -power * leftAmount / distance;   // drive() wants RIGHT, we measured LEFT
        }

        // Turn toward the target heading. The wheels' "rotate" is positive to the RIGHT
        // (clockwise), but our heading is positive to the LEFT, so flip the sign.
        boolean headingDone = Math.abs(Math.toDegrees(headingError)) <= HEADING_TOLERANCE_DEGREES;
        double minTurn = headingDone ? 0.0 : MIN_TURN_POWER;
        double rotate = -clampSigned(TURN_KP * headingError, minTurn, maxTurnPower);

        drive.drive(forward, right, rotate);
    }

    /** True once the robot is close enough to the target position AND heading. */
    public boolean isAtTarget() {
        return atTarget;
    }

    /** Stops the robot where it is. */
    public void stop() {
        drive.stop();
    }

    /** Keeps value between min and max. */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Keeps the SIZE of value between min and max, but keeps its sign (left/right).
     * A value of exactly 0 stays 0.
     */
    private static double clampSigned(double value, double min, double max) {
        if (value == 0) {
            return 0;
        }
        return Math.signum(value) * clamp(Math.abs(value), min, max);
    }
}
