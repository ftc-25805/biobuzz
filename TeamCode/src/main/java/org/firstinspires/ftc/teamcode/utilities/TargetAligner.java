package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.mechanisms.LimelightCamera;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * Turns the robot to face what the Limelight sees.
 *
 * The camera tells us "tx": how many degrees the target is to the right of the
 * camera's center. If tx is 0 the target is dead ahead. So we just turn in
 * proportion to tx until it is (about) zero. The bigger tx is, the harder we turn.
 *
 * Typical use (for example, while the driver holds a button):
 *     aligner.update();                       // robot spins to face the target
 *     if (aligner.isAligned()) { ... }        // pointed at it: shoot!
 *
 * Or let the driver keep driving while the robot aims itself:
 *     aligner.update(forward, right);         // driver moves, robot turns toward target
 *
 * Note: this aims the CAMERA at the target. If the camera is not mounted in the
 * middle of the robot, or the shooter does not point the same way as the camera,
 * the robot will be slightly off.
 */
public class TargetAligner {

    /** Aim at whichever target the camera sees. */
    public static final int ANY_TARGET = -1;

    // ---------------------------------------------------------------------
    // Tuning values. TODO: TUNE ON OUR ROBOT. Start low and work up.
    // ---------------------------------------------------------------------

    /** Turn power per degree of error. 0.02 = full push from 50 degrees away. */
    public static final double TURN_KP = 0.02;

    /** Smallest turn while still not aligned, so the robot does not stall just short. */
    public static final double MIN_TURN_POWER = 0.08;

    /** Fastest the robot may turn (0.0 to 1.0). */
    public static final double MAX_TURN_POWER = 0.4;

    /** How close to centered (in degrees) counts as "aligned". */
    public static final double TOLERANCE_DEGREES = 1.5;

    private final MecanumDrive drive;
    private final LimelightCamera camera;

    private int tagId = ANY_TARGET;
    private boolean aligned = false;

    public TargetAligner(MecanumDrive drive, LimelightCamera camera) {
        this.drive = drive;
        this.camera = camera;
    }

    /** Aim only at the AprilTag with this ID, ignoring everything else. */
    public void setTagId(int tagId) {
        this.tagId = tagId;
    }

    /** Aim at whatever target the camera sees (the default). */
    public void aimAtAnyTarget() {
        this.tagId = ANY_TARGET;
    }

    /** Call every loop. Turns in place toward the target. */
    public void update() {
        update(0.0, 0.0);
    }

    /**
     * Call every loop. The robot moves with the forward/right amounts you give it
     * (-1.0 to 1.0, from the driver's joystick) while turning on its own to face the target.
     */
    public void update(double forward, double right) {
        camera.update();

        if (!canSeeTarget()) {
            // Nothing to aim at. Don't turn, but still let the driver move.
            aligned = false;
            drive.drive(forward, right, 0.0);
            return;
        }

        double error = getErrorDegrees();   // positive = target is to the RIGHT
        aligned = Math.abs(error) <= TOLERANCE_DEGREES;

        double rotate = 0.0;
        if (!aligned) {
            // Target on the right (positive error) means turn right, and the wheels'
            // "rotate" is positive to the right, so no sign flip is needed.
            double power = Math.min(MAX_TURN_POWER,
                    Math.max(MIN_TURN_POWER, TURN_KP * Math.abs(error)));
            rotate = Math.signum(error) * power;
        }
        drive.drive(forward, right, rotate);
    }

    /** True if the camera can see the target we are aiming at. */
    public boolean canSeeTarget() {
        return tagId == ANY_TARGET ? camera.hasTarget() : camera.canSeeTag(tagId);
    }

    /** True once the robot is facing the target (and can still see it). */
    public boolean isAligned() {
        return aligned;
    }

    /** How many degrees the target is to the right of center (negative = left). */
    public double getErrorDegrees() {
        return tagId == ANY_TARGET ? camera.getTargetXDegrees() : camera.getTagXDegrees(tagId);
    }

    /** Stops the robot. */
    public void stop() {
        aligned = false;
        drive.stop();
    }
}
