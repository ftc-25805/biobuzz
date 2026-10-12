package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Driver-controlled mecanum drive. This op mode only reads the gamepad.
 * All the motor and sensor details live in {@link RobotHardware}.
 */
@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
@Disabled
public class RobotTeleopMecanumDrive extends OpMode {
    private RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        // The Pinpoint and Limelight need to be updated once every loop.
        robot.odometry.update();
        robot.limelight.update();

        // Pressing A makes the way the robot is currently pointing count as "forward".
        if (gamepad1.a) {
            robot.imu.resetHeading();
        }

        // Pushing a joystick forward gives a NEGATIVE y value, so we flip it.
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // robot.drive.driveFieldRelative(forward, right, rotate, robot.imu.getHeadingRadians());

        // AUTO-AIM (optional): hold the right bumper and the robot turns by itself to face
        // what the Limelight sees, while you keep driving with the left joystick.
        // To use it:
        //   1) Add a field:   private TargetAligner aligner;
        //      (and import org.firstinspires.ftc.teamcode.utilities.TargetAligner)
        //   2) In init():     aligner = new TargetAligner(robot.drive, robot.limelight);
        //   3) Replace the drive line below with:
        //
        // if (gamepad1.right_bumper) {
        //     aligner.update(forward, right);   // you move, the robot aims
        // } else {
        //     robot.drive.drive(forward, right, rotate);
        // }
        robot.drive.drive(forward, right, rotate);
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
