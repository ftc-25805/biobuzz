package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.mechanisms.ImuSensor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointOdometry;

/**
 * RobotHardware is the ONE place that knows about the robot's physical parts:
 * which motors and sensors exist, what they are named, and how they are set up.
 *
 * It finds each piece of hardware, configures it, and hands it to a mechanism
 * class (in the "mechanisms" package) that knows how to use it. An op mode only needs:
 *
 *     robot = new RobotHardware(hardwareMap);
 *     robot.drive.drive(forward, right, rotate);
 *
 * Wire swapped? Motor spinning the wrong way? Change it here and nowhere else.
 *
 * To add a new mechanism: write a class in "mechanisms" that takes its motors
 * or sensors in its constructor, add its settings below, then add a field and
 * set it up in the constructor here.
 */
public class RobotHardware {

    // ---------------------------------------------------------------------
    // Names. These must match the Driver Station's robot configuration exactly.
    // ---------------------------------------------------------------------
    public static final String FRONT_LEFT_NAME = "front_left_drive";
    public static final String FRONT_RIGHT_NAME = "front_right_drive";
    public static final String BACK_LEFT_NAME = "back_left_drive";
    public static final String BACK_RIGHT_NAME = "back_right_drive";
    public static final String IMU_NAME = "imu";
    public static final String PINPOINT_NAME = "pinpoint";

    // ---------------------------------------------------------------------
    // Tuning values. Change these to fit the robot.
    // ---------------------------------------------------------------------

    /** Top drive speed from 0.0 to 1.0. Lower this for outreach events. */
    public static final double MAX_DRIVE_SPEED = 1.0;

    /**
     * Where the Pinpoint's odometry pods sit, measured in mm from the robot's center
     * of rotation. TODO: MEASURE ON OUR ROBOT. These are placeholders from the SDK sample.
     * X pod offset: sideways (left is +). Y pod offset: forward (forward is +).
     */
    public static final double PINPOINT_X_POD_OFFSET_MM = -84.0;
    public static final double PINPOINT_Y_POD_OFFSET_MM = -168.0;
    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    /** Flip to REVERSED if the numbers go DOWN when you push the robot forward (X) or left (Y). */
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    /** How the Control Hub is mounted on the robot. */
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ---------------------------------------------------------------------
    // The mechanisms
    // ---------------------------------------------------------------------
    public final MecanumDrive drive;
    public final ImuSensor imu;
    public final PinpointOdometry odometry;

    /**
     * Finds and sets up all the robot's hardware.
     * Call this once, from an op mode's init().
     */
    public RobotHardware(HardwareMap hardwareMap) {
        // --- Drive motors ---
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        DcMotor backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        DcMotor backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        // The left motors face the opposite way from the right ones, so reverse
        // them. That way positive power means "forward" for every wheel.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // RUN_USING_ENCODER makes speed more accurate.
        // If you don't have the encoder wires plugged in, remove these.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight, MAX_DRIVE_SPEED);

        // --- IMU (tells us which way the robot is facing) ---
        IMU imuHardware = hardwareMap.get(IMU.class, IMU_NAME);
        imuHardware.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(HUB_LOGO_DIRECTION, HUB_USB_DIRECTION)));

        imu = new ImuSensor(imuHardware);

        // --- Pinpoint odometry (tells us where the robot is on the field) ---
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        pinpoint.setOffsets(PINPOINT_X_POD_OFFSET_MM, PINPOINT_Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(PINPOINT_POD_TYPE);
        pinpoint.setEncoderDirections(PINPOINT_X_DIRECTION, PINPOINT_Y_DIRECTION);
        pinpoint.resetPosAndIMU();  // robot must be sitting still when this runs

        odometry = new PinpointOdometry(pinpoint);
    }

    /** Stops everything that moves. Add new mechanisms here as you create them. */
    public void stop() {
        drive.stop();
    }
}
