package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * The robot's four-wheel mecanum drivetrain.
 *
 * Mecanum wheels let the robot drive forward, strafe sideways, and turn,
 * all at the same time. This class turns those three moves into a power
 * for each of the four wheels.
 */
public class MecanumDrive {

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final double maxSpeed;

    /**
     * @param frontLeft  the four drive motors, already set up by RobotHardware
     * @param maxSpeed   top speed from 0.0 to 1.0 (lower it for outreach events)
     */
    public MecanumDrive(DcMotor frontLeft, DcMotor frontRight,
                        DcMotor backLeft, DcMotor backRight,
                        double maxSpeed) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.maxSpeed = maxSpeed;
    }

    /**
     * Drives relative to the ROBOT (like an RC car): "forward" is wherever
     * the robot's front is pointing.
     *
     * @param forward -1.0 (backward) to 1.0 (forward)
     * @param right   -1.0 (strafe left) to 1.0 (strafe right)
     * @param rotate  -1.0 (turn left) to 1.0 (turn right)
     */
    public void drive(double forward, double right, double rotate) {
        // Each wheel gets a different mix of the three moves.
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        // A motor can't go faster than 1.0. If any wheel asks for more, shrink
        // ALL the wheels by the same amount so the robot still goes the right way.
        double largest = 1.0;
        largest = Math.max(largest, Math.abs(frontLeftPower));
        largest = Math.max(largest, Math.abs(frontRightPower));
        largest = Math.max(largest, Math.abs(backLeftPower));
        largest = Math.max(largest, Math.abs(backRightPower));

        frontLeft.setPower(maxSpeed * (frontLeftPower / largest));
        frontRight.setPower(maxSpeed * (frontRightPower / largest));
        backLeft.setPower(maxSpeed * (backLeftPower / largest));
        backRight.setPower(maxSpeed * (backRightPower / largest));
    }

    /**
     * Drives relative to the FIELD: "forward" is always away from the driver,
     * no matter which way the robot is facing.
     *
     * @param headingRadians the direction the robot is facing (get it from {@link ImuSensor})
     */
    public void driveFieldRelative(double forward, double right, double rotate,
                                   double headingRadians) {
        // 1. Turn the joystick push into an angle and a strength.
        double angle = Math.atan2(forward, right);
        double strength = Math.hypot(right, forward);

        // 2. Subtract the way the robot is facing, so the push is
        //    measured from the robot's point of view.
        angle = AngleUnit.normalizeRadians(angle - headingRadians);

        // 3. Turn the angle and strength back into forward/right amounts.
        drive(strength * Math.sin(angle), strength * Math.cos(angle), rotate);
    }

    /** Stops all drive motors. */
    public void stop() {
        drive(0, 0, 0);
    }
}
