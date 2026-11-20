package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.oldmechanisms.Flywheel;

/**
 * Driver-controlled mecanum drive. This op mode only reads the gamepad.
 * All the motor and sensor details live in {@link RobotHardware}.
 */
@TeleOp(name = "Robot: Field Relative Mecanum Drive")
public class RobotTeleopMecanumDrive extends OpMode {
    private RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.odometry.resetPosition();  // robot must be sitting still when this runs
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        // The Pinpoint and Limelight need to be updated once every loop.
        robot.odometry.update();
        //robot.limelight.update();

        // Pressing A makes the way the robot is currently pointing count as "forward".
        if (gamepad1.a) {
            robot.imu.resetHeading();
            robot.odometry.resetPosition();
        }

        // Pushing a joystick forward gives a NEGATIVE y value, so we flip it.
        double forward = gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;


        if (gamepad2.dpad_down) {
            robot.intake.collect();
        } else {
            robot.intake.stop();
        }

        if (gamepad2.dpad_up || gamepad2.left_trigger > 0) {
            if(gamepad2.left_trigger > 0) {
                robot.transfer.reverse();
            } else {
                robot.transfer.feed();
            }

        } else {
            robot.transfer.stop();
        }

        if (gamepad2.b) {
            robot.flywheel.setTargetVelocity(1000);
        }

        if (gamepad2.x)  {
            robot.flywheel.setTargetVelocity(1100);
        }

        if (gamepad2.y) {
            robot.flywheel.setTargetVelocity(1200);
        }

        if(gamepad2.right_trigger>0) {
            robot.flywheel.setTargetVelocity(0);
        }

        if(gamepad2.a){
            robot.flywheel.setTargetVelocity(-1000);
        }

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

        telemetry.addData("flywheel velo: ", robot.flywheel.getVelocity());
        telemetry.addData("EncoderX", robot.odometry.getXInches());
        telemetry.addData("EncoderY", robot.odometry.getYInches());
        telemetry.addData("OdoHeading", robot.odometry.getHeadingDegrees());
        telemetry.addData("OdoHeading", robot.odometry.getStatusText());
        telemetry.addData("OdoHeading", robot.odometry.isHealthy());
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
