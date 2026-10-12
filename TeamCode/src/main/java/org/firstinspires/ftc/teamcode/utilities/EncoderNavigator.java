package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.ImuSensor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * Drives a set distance straight, and turns to a set heading, using the wheel
 * encoders to measure distance and the IMU to measure direction.
 *
 * Use this when you don't have a Pinpoint. It does two simple moves:
 *     driveStraight(inches, headingDegrees)   roll forward/backward while holding a heading
 *     turnToHeading(headingDegrees)           spin in place to face a direction
 *
 * Headings are measured from where the IMU was last reset:
 *     0 = the way the robot started, positive = turned LEFT, negative = turned RIGHT.
 *
 * How it works: every loop we measure how far we still have to go (inches or degrees)
 * and push in proportion to it. The farther away, the harder the push, so the robot
 * slows down as it arrives ("proportional control").
 *
 * Typical use:
 *     navigator.driveStraight(24, 0);         // 24 inches forward, staying on heading 0
 *     while (opModeIsActive() && !navigator.isAtTarget()) {
 *         navigator.update();                 // call every loop
 *     }
 */
public class EncoderNavigator {

    // ---------------------------------------------------------------------
    // Tuning values. TODO: TUNE ON OUR ROBOT. Start low and work up.
    // ---------------------------------------------------------------------

    /** Drive power per inch left. 0.04 = full push from 25 inches away. */
    public static final double DRIVE_KP = 0.04;
    /** Steering power per degree of heading error while driving straight. */
    public static final double STEER_KP = 0.03;
    /** Turn power per degree of heading error while turning in place. */
    public static final double TURN_KP = 0.02;

    /** Fastest the robot may drive / turn (0.0 to 1.0). Keep low while testing! */
    public static final double MAX_DRIVE_POWER = 0.4;
    public static final double MAX_TURN_POWER = 0.3;

    /** How close counts as "arrived". */
    public static final double DISTANCE_TOLERANCE_INCHES = 0.5;
    public static final double HEADING_TOLERANCE_DEGREES = 1.0;

    private enum Move { NONE, DRIVE_STRAIGHT, TURN }

    private final MecanumDrive drive;
    private final ImuSensor imu;

    private Move move = Move.NONE;
    private double targetInches;
    private double targetHeadingDegrees;
    private boolean atTarget = true;

    public EncoderNavigator(MecanumDrive drive, ImuSensor imu) {
        this.drive = drive;
        this.imu = imu;
    }

    /**
     * Starts a straight move.
     *
     * @param inches         distance to roll from where we are now (negative = backward)
     * @param headingDegrees direction to keep the robot facing the whole way
     */
    public void driveStraight(double inches, double headingDegrees) {
        drive.resetDistance();
        targetInches = inches;
        targetHeadingDegrees = headingDegrees;
        move = Move.DRIVE_STRAIGHT;
        atTarget = false;
    }

    /** Starts a turn in place to face this heading, in degrees. */
    public void turnToHeading(double headingDegrees) {
        targetHeadingDegrees = headingDegrees;
        move = Move.TURN;
        atTarget = false;
    }

    /** Call every loop while moving. */
    public void update() {
        double headingError = headingErrorDegrees();

        if (move == Move.DRIVE_STRAIGHT) {
            double inchesLeft = targetInches - drive.getForwardInches();
            atTarget = Math.abs(inchesLeft) <= DISTANCE_TOLERANCE_INCHES;
            if (atTarget) {
                drive.stop();
                return;
            }
            // Push forward (or backward) in proportion to the distance left, and steer
            // a little to stay on heading. Steering turns the robot the same way
            // whether it is driving forward or backward, so no sign flip is needed.
            double forward = clamp(DRIVE_KP * inchesLeft, MAX_DRIVE_POWER);
            double rotate = -clamp(STEER_KP * headingError, MAX_TURN_POWER);
            drive.drive(forward, 0, rotate);

        } else if (move == Move.TURN) {
            atTarget = Math.abs(headingError) <= HEADING_TOLERANCE_DEGREES;
            if (atTarget) {
                drive.stop();
                return;
            }
            // The wheels' "rotate" is positive to the RIGHT (clockwise), but headings are
            // positive to the LEFT, so flip the sign.
            drive.drive(0, 0, -clamp(TURN_KP * headingError, MAX_TURN_POWER));
        }
    }

    /** True once the current move has finished. */
    public boolean isAtTarget() {
        return atTarget;
    }

    /** Degrees still to turn to reach the target heading (positive = turn left). */
    public double headingErrorDegrees() {
        double error = Math.toRadians(targetHeadingDegrees) - imu.getHeadingRadians();
        return Math.toDegrees(AngleUnit.normalizeRadians(error));
    }

    /** Stops the robot where it is and cancels the move. */
    public void stop() {
        move = Move.NONE;
        drive.stop();
    }

    private static double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
