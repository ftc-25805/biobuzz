package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.EncoderNavigator;

/**
 * Autonomous: drives a diamond-shaped path using the wheel encoders for distance
 * and the IMU for direction. It needs no Pinpoint.
 *
 * Headings are measured from the way the robot faces at Play:
 * 0 = forward, positive = turned left, negative = turned right.
 *
 * Before running: clear about 4 feet in front and to the sides, and test with the
 * robot up on blocks first!
 */
@Autonomous(name = "Auto: Encoder Drive and Turn", group = "Encoder")
public class AutoEncoderDriveAndTurn extends LinearOpMode {

    /** Give up on a move after this long so a stuck robot doesn't freeze the whole auto. */
    private static final double MOVE_TIMEOUT_SECONDS = 5.0;

    private RobotHardware robot;
    private EncoderNavigator navigator;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);
        navigator = new EncoderNavigator(robot.drive, robot.imu);

        // Show the heading while waiting, so you can check the IMU is working.
        while (opModeInInit()) {
            telemetry.addData(">", "Heading = %.0f deg", robot.imu.getHeadingDegrees());
            telemetry.update();
        }
        waitForStart();

        // Whichever way the robot faces now counts as heading 0.
        robot.imu.resetHeading();

        // The path. Backward moves use a negative distance, not a negative speed.
        driveStraight(24, 0);       // forward 24 inches
        turnTo(-45);                // turn right to -45 degrees
        holdHeading(-45, 0.5);      // pause half a second to settle
        driveStraight(17, -45);     // forward 17 inches while holding -45 degrees
        turnTo(45);                 // turn left to 45 degrees
        holdHeading(45, 0.5);
        driveStraight(17, 45);      // forward 17 inches while holding 45 degrees
        turnTo(0);                  // turn back to face forward
        holdHeading(0, 1.0);
        driveStraight(-48, 0);      // backward 48 inches, back near the start

        robot.stop();
    }

    /** Rolls a distance (negative = backward) while holding a heading. */
    private void driveStraight(double inches, double headingDegrees) {
        navigator.driveStraight(inches, headingDegrees);
        runUntilDone(MOVE_TIMEOUT_SECONDS, true);
    }

    /** Turns in place to face a heading. */
    private void turnTo(double headingDegrees) {
        navigator.turnToHeading(headingDegrees);
        runUntilDone(MOVE_TIMEOUT_SECONDS, true);
    }

    /** Keeps correcting toward a heading for a while, so the robot stops wobbling. */
    private void holdHeading(double headingDegrees, double seconds) {
        navigator.turnToHeading(headingDegrees);
        runUntilDone(seconds, false);
    }

    /**
     * Runs the navigator until the move finishes (if stopWhenArrived) or time runs out.
     * Holding keeps calling update() for the full time, even after arriving.
     */
    private void runUntilDone(double timeoutSeconds, boolean stopWhenArrived) {
        long startTime = System.nanoTime();

        while (opModeIsActive()
                && (System.nanoTime() - startTime) / 1e9 < timeoutSeconds
                && !(stopWhenArrived && navigator.isAtTarget())) {
            navigator.update();

            telemetry.addData("Heading error", "%.1f deg", navigator.headingErrorDegrees());
            telemetry.addData("Heading", "%.1f deg", robot.imu.getHeadingDegrees());
            telemetry.addData("Distance", "%.1f in", robot.drive.getForwardInches());
            telemetry.update();
        }
        navigator.stop();
    }
}
