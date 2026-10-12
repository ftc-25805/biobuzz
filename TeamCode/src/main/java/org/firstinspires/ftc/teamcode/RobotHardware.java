package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * RobotHardware is the ONE place that knows about the robot's physical parts:
 * motors, sensors, and how they are wired up.
 *
 * Op modes (TeleOp and Autonomous) should never talk to motors directly.
 * Instead they create a RobotHardware and ask it to do things, like:
 *
 *     robot = new RobotHardware(hardwareMap);
 *     robot.drive(forward, right, rotate);
 *
 * Why? If we change a motor name or flip a direction, we fix it here once
 * and every op mode keeps working.
 *
 * To add a new part (arm, claw, camera...): add a name constant, add a
 * variable, set it up in the constructor, and add a method that uses it.
 */
public class RobotHardware {

    // ---------------------------------------------------------------------
    // Configuration names. These must match the names in the Driver Station's
    // robot configuration exactly.
    // ---------------------------------------------------------------------
    public static final String FRONT_LEFT_NAME = "front_left_drive";
    public static final String FRONT_RIGHT_NAME = "front_right_drive";
    public static final String BACK_LEFT_NAME = "back_left_drive";
    public static final String BACK_RIGHT_NAME = "back_right_drive";
    public static final String IMU_NAME = "imu";

    // ---------------------------------------------------------------------
    // Tuning values. Change these to fit the robot.
    // ---------------------------------------------------------------------

    /** Top speed from 0.0 to 1.0. Lower this for outreach events. */
    public static final double MAX_DRIVE_SPEED = 1.0;

    /** How the control hub is mounted on the robot. Change to match your robot. */
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ---------------------------------------------------------------------
    // The hardware itself. Private so op modes must go through our methods.
    // ---------------------------------------------------------------------
    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;
    private final IMU imu;

    /**
     * Finds and sets up all the robot hardware.
     * Call this once, from an op mode's init().
     *
     * @param hardwareMap the op mode's hardwareMap
     */
    public RobotHardware(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRightDrive = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeftDrive = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRightDrive = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        // The left motors face the opposite way from the right ones, so reverse
        // them. That way positive power means "forward" for every wheel.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // RUN_USING_ENCODER makes speed more accurate.
        // If you don't have the encoder wires plugged in, remove these.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // The IMU tells us which direction the robot is facing.
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(HUB_LOGO_DIRECTION, HUB_USB_DIRECTION);
        imu.initialize(new IMU.Parameters(orientation));
    }

    // ---------------------------------------------------------------------
    // Driving
    // ---------------------------------------------------------------------

    /**
     * Drives relative to the ROBOT (like an RC car): "forward" is wherever
     * the robot's front is pointing.
     *
     * @param forward -1.0 (backward) to 1.0 (forward)
     * @param right   -1.0 (strafe left) to 1.0 (strafe right)
     * @param rotate  -1.0 (turn left) to 1.0 (turn right)
     */
    public void drive(double forward, double right, double rotate) {
        // Mecanum wheels: each wheel gets a different mix of the three moves.
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

        frontLeftDrive.setPower(MAX_DRIVE_SPEED * (frontLeftPower / largest));
        frontRightDrive.setPower(MAX_DRIVE_SPEED * (frontRightPower / largest));
        backLeftDrive.setPower(MAX_DRIVE_SPEED * (backLeftPower / largest));
        backRightDrive.setPower(MAX_DRIVE_SPEED * (backRightPower / largest));
    }

    /**
     * Drives relative to the FIELD: "forward" is always away from the driver,
     * no matter which way the robot is facing.
     *
     * Same inputs as {@link #drive(double, double, double)}.
     */
    public void driveFieldRelative(double forward, double right, double rotate) {
        // 1. Turn the joystick push into an angle and a strength.
        double angle = Math.atan2(forward, right);
        double strength = Math.hypot(right, forward);

        // 2. Subtract the way the robot is facing, so the push is
        //    measured from the robot's point of view.
        angle = AngleUnit.normalizeRadians(angle - getHeadingRadians());

        // 3. Turn the angle and strength back into forward/right amounts.
        double robotForward = strength * Math.sin(angle);
        double robotRight = strength * Math.cos(angle);

        drive(robotForward, robotRight, rotate);
    }

    /** Stops all drive motors. */
    public void stop() {
        drive(0, 0, 0);
    }

    // ---------------------------------------------------------------------
    // Sensors
    // ---------------------------------------------------------------------

    /** The direction the robot is facing, in radians (0 = the way it faced at the last reset). */
    public double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Makes the robot's current direction count as "forward" for field-relative driving. */
    public void resetHeading() {
        imu.resetYaw();
    }
}
