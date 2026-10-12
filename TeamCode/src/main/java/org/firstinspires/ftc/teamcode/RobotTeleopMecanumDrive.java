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

        // Pressing A makes the way the robot is currently pointing count as "forward".
        if (gamepad1.a) {
            robot.resetHeading();
        }

        // Pushing a joystick forward gives a NEGATIVE y value, so we flip it.
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        robot.driveFieldRelative(forward, right, rotate);
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
