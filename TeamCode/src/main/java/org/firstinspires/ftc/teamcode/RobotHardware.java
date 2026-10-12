package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mechanisms.ImuSensor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

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

    // ---------------------------------------------------------------------
    // Tuning values. Change these to fit the robot.
    // ---------------------------------------------------------------------

    /** Top drive speed from 0.0 to 1.0. Lower this for outreach events. */
    public static final double MAX_DRIVE_SPEED = 1.0;

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
    }

    /** Stops everything that moves. Add new mechanisms here as you create them. */
    public void stop() {
        drive.stop();
    }
}
