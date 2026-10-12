package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.PointToPointNavigator;

/**
 * Autonomous: drives a square using the Pinpoint, turning as it goes, and ends
 * back where it started.
 *
 * Positions are measured from where the robot sits at init: x = inches forward,
 * y = inches left, heading = degrees turned left.
 *
 * Before running: put the robot on the field, hold it still until init finishes,
 * and clear 3 feet in front and to the left of it. Test with the robot up on blocks first!
 */
@Autonomous(name = "Auto: Pinpoint Drive and Turn", group = "Robot")
public class AutoPinpointDriveAndTurn extends LinearOpMode {

    /** Give up on a move after this long so a stuck robot doesn't freeze the whole auto. */
    private static final double MOVE_TIMEOUT_SECONDS = 5.0;

    private RobotHardware robot;
    private PointToPointNavigator navigator;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);
        navigator = new PointToPointNavigator(robot.drive, robot.odometry);

        telemetry.addLine("Ready. Keep the robot still, then press Play.");
        telemetry.update();
        waitForStart();

        // The path: a square, turning left at each corner.
        //          x (in)  y (in)  heading (deg)
        moveTo(24, 0, 0);       // drive forward 24 inches
        moveTo(24, 24, 90);     // slide left 24 inches, turn to face left
        moveTo(0, 24, 180);     // drive back, turn to face backward
        moveTo(0, 0, 0);        // return to start, facing forward

        robot.stop();
    }

    /** Goes to a spot and heading, then returns. Gives up after the timeout. */
    private void moveTo(double xInches, double yInches, double headingDegrees) {
        navigator.setTarget(xInches, yInches, headingDegrees);
        long startTime = System.nanoTime();

        while (opModeIsActive() && !navigator.isAtTarget()
                && (System.nanoTime() - startTime) / 1e9 < MOVE_TIMEOUT_SECONDS) {
            navigator.update();

            telemetry.addData("Target", "x=%.1f y=%.1f heading=%.0f",
                    xInches, yInches, headingDegrees);
            telemetry.addData("Now", "x=%.1f y=%.1f heading=%.0f",
                    robot.odometry.getXInches(), robot.odometry.getYInches(),
                    robot.odometry.getHeadingDegrees());
            telemetry.update();
        }
        navigator.stop();
    }
}
