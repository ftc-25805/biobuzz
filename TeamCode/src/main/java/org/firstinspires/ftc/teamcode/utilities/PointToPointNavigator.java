package org.firstinspires.ftc.teamcode.utilities;

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

    /** How close counts as "arrived". */
    public static final double POSITION_TOLERANCE_INCHES = 1.0;
    public static final double HEADING_TOLERANCE_DEGREES = 2.0;

    private final MecanumDrive drive;
    private final PinpointOdometry odometry;

    private double targetXInches;
    private double targetYInches;
    private double targetHeadingRadians;
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

        atTarget = distance <= POSITION_TOLERANCE_INCHES
                && Math.abs(Math.toDegrees(headingError)) <= HEADING_TOLERANCE_DEGREES;
        if (atTarget) {
            drive.stop();
            return;
        }

        // Push toward the target: harder when far, softer when close, never over the max.
        double power = Math.min(MAX_DRIVE_POWER, DRIVE_KP * distance);

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
        double rotate = -clamp(TURN_KP * headingError, MAX_TURN_POWER);

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

    private static double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
