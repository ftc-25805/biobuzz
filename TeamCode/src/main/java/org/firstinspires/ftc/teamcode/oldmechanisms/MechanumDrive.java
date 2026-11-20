package org.firstinspires.ftc.teamcode.oldmechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * MechanumDrive - Mecanum wheel drivetrain control for both TeleOp and Autonomous
 *
 * MECANUM WHEEL MOVEMENT PATTERNS:
 * ================================
 *
 * Movement Type     | FL  | FR  | BL  | BR  | Description
 * ------------------|-----|-----|-----|-----|----------------------------------
 * Forward           |  +  |  +  |  +  |  +  | All wheels same direction
 * Backward          |  -  |  -  |  -  |  -  | All wheels reverse
 * Strafe Right      |  +  |  -  |  -  |  +  | Diagonal wheel pattern
 * Strafe Left       |  -  |  +  |  +  |  -  | Opposite diagonal
 * Rotate CCW        |  +  |  -  |  +  |  -  | Left forward, right backward
 * Rotate CW         |  -  |  +  |  -  |  +  | Left backward, right forward
 *
 * FEATURES:
 * - TeleOp: Robot-relative and field-relative driving with IMU
 * - Autonomous: Encoder-based movements (forward, strafe, rotate)
 * - State machine compatible: Functions return true when complete
 *
 * USAGE IN AUTONOMOUS:
 * 1. Call resetEncoders() before each movement
 * 2. Call movement function in loop (driveForward, strafe, rotate)
 * 3. When function returns true, movement is complete
 * 4. Repeat for next movement
 *
 * Based on code shared by FTC16072
 */
public class MechanumDrive {

    private Hardware hardware;

    // ================================
    // AUTONOMOUS DRIVING CONSTANTS
    // ================================
    // These values should be tuned to match your specific robot configuration
    private final double WHEEL_DIAMETER_MM = 96;              // goBILDA mecanum wheel diameter
    private final double ENCODER_TICKS_PER_REV = 537.7;       // For goBILDA 5203 series Yellow Jacket motors
    private final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    private final double TRACK_WIDTH_MM = 404;                // Distance between left and right wheels (measure your robot!)
    private final double WHEELBASE_MM = 404;                  // Distance between front and back wheels (measure your robot!)

    private ElapsedTime driveTimer = new ElapsedTime();

    public MechanumDrive(Hardware hardware){
        this.hardware = hardware;
    }

    // ================================
    // TELEOP DRIVING FUNCTIONS
    // ================================

    /**
     * Robot-relative mecanum drive control for TeleOp.
     * Uses the standard mecanum wheel formula to calculate power for each wheel.
     *
     * @param forward  Forward/backward movement (-1.0 to 1.0, positive = forward)
     * @param strafe   Left/right movement (-1.0 to 1.0, positive = right)
     * @param rotate   Rotation (-1.0 to 1.0, positive = counter-clockwise)
     *
     * Thanks to FTC16072 for sharing this code!
     */
    public void drive(double forward, double strafe, double rotate, double max_power) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;
        double backLeftPower = forward - strafe + rotate;

        double maxPower = 1.0;
        double maxSpeed = max_power;//max_power;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        hardware.frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        hardware.frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        hardware.backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        hardware.backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    /**
     * Field-relative mecanum drive control for TeleOp.
     * Driver's joystick input is relative to the field, not the robot.
     * Forward on joystick always moves robot toward opponent, regardless of robot rotation.
     *
     * @param forward  Forward/backward relative to field (-1.0 to 1.0)
     * @param strafe   Left/right relative to field (-1.0 to 1.0)
     * @param rotate   Rotation (-1.0 to 1.0, positive = counter-clockwise)
     */
    public void driveFieldRelative(double forward, double strafe, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                hardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate, 1.0);
    }

    /**
     * Set mode for all drive motors
     */
    public void setMode(DcMotor.RunMode mode) {
        hardware.frontLeftDrive.setMode(mode);
        hardware.frontRightDrive.setMode(mode);
        hardware.backLeftDrive.setMode(mode);
        hardware.backRightDrive.setMode(mode);
    }

    /**
     * Set mecanum drive power
     * @param forward Forward/backward power (-1 to 1)
     * @param strafe Left/right power (-1 to 1)
     * @param turn Rotation power (-1 to 1)
     */
    public void setMecanumPower(double forward, double strafe, double turn) {
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        // Normalize powers
        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))
        ));

        hardware.frontLeftDrive.setPower(flPower / maxPower);
        hardware.frontRightDrive.setPower(frPower / maxPower);
        hardware.backLeftDrive.setPower(blPower / maxPower);
        hardware.backRightDrive.setPower(brPower / maxPower);
    }
}