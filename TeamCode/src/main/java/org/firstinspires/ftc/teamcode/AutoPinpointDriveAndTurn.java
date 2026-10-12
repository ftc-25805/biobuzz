package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.PointToPointNavigator;

/**
 * Autonomous: drives a square using the Pinpoint, turning as it goes.
 *
 * A timer counts up to the 30 seconds autonomous lasts. When only
 * PARK_TIME_REMAINING_SECONDS are left, the robot drops whatever it is doing,
 * skips the rest of the routine, and drives to the parking spot. If the routine
 * finishes early, the robot parks right away.
 *
 * Alliance: while waiting for Play, press B for red or X for blue. The routine below is
 * written for red; blue mirrors it left-to-right.
 *
 * Positions are measured from where the robot sits at init: x = inches forward,
 * y = inches left, heading = degrees turned left.
 *
 * Before running: put the robot on the field, hold it still until init finishes,
 * and clear 3 feet in front and to the left of it. Test with the robot up on blocks first!
 */
@Autonomous(name = "Auto: Pinpoint Drive and Turn", group = "Robot")
public class AutoPinpointDriveAndTurn extends LinearOpMode {

    /** How long autonomous lasts. */
    private static final double AUTO_DURATION_SECONDS = 30.0;

    /** When this many seconds are left, stop the routine and go park. */
    private static final double PARK_TIME_REMAINING_SECONDS = 5.0;

    /** Where to park. TODO: SET THIS TO THE REAL PARKING SPOT. */
    private static final double PARK_X_INCHES = 12;
    private static final double PARK_Y_INCHES = 36;
    private static final double PARK_HEADING_DEGREES = 0;

    /** Speed limits for the park move (0.0 to 1.0). Higher than normal so we make it in time. */
    private static final double PARK_MAX_DRIVE_POWER = 0.8;
    private static final double PARK_MAX_TURN_POWER = 0.6;

    /** Give up on a move after this long so a stuck robot doesn't freeze the whole auto. */
    private static final double MOVE_TIMEOUT_SECONDS = 5.0;

    private RobotHardware robot;
    private PointToPointNavigator navigator;
    private final ElapsedTime autoTimer = new ElapsedTime();

    /** Which side of the field we are on. The routine is written for RED; BLUE mirrors it. */
    private boolean blueAlliance = false;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);
        navigator = new PointToPointNavigator(robot.drive, robot.odometry);

        // While waiting for Play: choose the alliance and check the Pinpoint is working.
        while (opModeInInit()) {
            if (gamepad1.b) {
                blueAlliance = false;
            } else if (gamepad1.x) {
                blueAlliance = true;
            }

            telemetry.addData("Alliance (B = red, X = blue)", blueAlliance ? "BLUE" : "RED");
            telemetry.addData("Pinpoint", robot.odometry.isHealthy()
                    ? "OK" : "PROBLEM: " + robot.odometry.getStatusText());
            telemetry.addLine("Keep the robot still, then press Play.");
            telemetry.update();
        }
        waitForStart();
        autoTimer.reset();   // the 30 seconds starts now

        // The path: a square, turning left at each corner.
        // Once it is time to park, every moveTo below is skipped.
        //          x (in)  y (in)  heading (deg)
        moveTo(24, 0, 0);               // drive forward 24 inches
        moveTo(24, 24, 90, 0.3, 0.3);   // slide left 24 inches, turn to face left (slow and careful)
        moveTo(0, 24, 180);             // drive back, turn to face backward
        moveTo(0, 0, 0);                // return to start, facing forward

        park();

        robot.stop();
    }

    /**
     * Flips a position or heading to the other side of the field for the blue alliance.
     * The field is a mirror image left-to-right, so "left" becomes "right":
     * y changes sign, and so does heading (turning left becomes turning right).
     */
    private double mirror(double value) {
        return blueAlliance ? -value : value;
    }

    /** Seconds left in autonomous. */
    private double secondsRemaining() {
        return AUTO_DURATION_SECONDS - autoTimer.seconds();
    }

    /** True once there are only a few seconds left and it is time to park. */
    private boolean timeToPark() {
        return secondsRemaining() <= PARK_TIME_REMAINING_SECONDS;
    }

    /**
     * Goes to a spot and heading, then returns. Gives up after the timeout.
     * Does nothing if it is already time to park, and stops early if time to park arrives.
     */
    private void moveTo(double xInches, double yInches, double headingDegrees) {
        moveTo(xInches, yInches, headingDegrees,
                PointToPointNavigator.MAX_DRIVE_POWER, PointToPointNavigator.MAX_TURN_POWER);
    }

    /** Same as above, but with speed limits (0.0 to 1.0) for just this move. */
    private void moveTo(double xInches, double yInches, double headingDegrees,
                        double maxDrive, double maxTurn) {
        driveTo(xInches, yInches, headingDegrees, maxDrive, maxTurn,
                MOVE_TIMEOUT_SECONDS, true);
    }

    /** Drives to the parking spot, using whatever time is left. */
    private void park() {
        // Park at full speed. There is not much time left.
        driveTo(PARK_X_INCHES, PARK_Y_INCHES, PARK_HEADING_DEGREES,
                PARK_MAX_DRIVE_POWER, PARK_MAX_TURN_POWER,
                Math.max(secondsRemaining(), 0), false);
    }

    /**
     * Drives to a spot and heading. Stops when it arrives, when the timeout runs out,
     * or (if stopForPark) when it is time to park.
     */
    private void driveTo(double xInches, double yInches, double headingDegrees,
                         double maxDrive, double maxTurn,
                         double timeoutSeconds, boolean stopForPark) {
        navigator.setTarget(xInches, mirror(yInches), mirror(headingDegrees), maxDrive, maxTurn);
        long startTime = System.nanoTime();

        while (opModeIsActive() && !navigator.isAtTarget()
                && (System.nanoTime() - startTime) / 1e9 < timeoutSeconds
                && !(stopForPark && timeToPark())) {
            navigator.update();

            telemetry.addData("Time left", "%.1f s", secondsRemaining());
            telemetry.addData("Target", "x=%.1f y=%.1f heading=%.0f",
                    xInches, mirror(yInches), mirror(headingDegrees));
            telemetry.addData("Now", "x=%.1f y=%.1f heading=%.0f",
                    robot.odometry.getXInches(), robot.odometry.getYInches(),
                    robot.odometry.getHeadingDegrees());
            telemetry.update();
        }
        navigator.stop();
    }
}
