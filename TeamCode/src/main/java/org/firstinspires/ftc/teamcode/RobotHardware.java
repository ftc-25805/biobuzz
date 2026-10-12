package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.ImuSensor;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightCamera;
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
    public static final String LIMELIGHT_NAME = "limelight";
    public static final String FLYWHEEL_NAME = "flywheel";
    public static final String INTAKE_NAME = "intake";

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

    /** Which Limelight pipeline to use when the robot starts (set up on the Limelight web page). */
    public static final int LIMELIGHT_START_PIPELINE = 0;

    /**
     * Encoder ticks per turn of the flywheel. TODO: CHECK OUR MOTOR'S SPEC SHEET (and any gearing).
     * 28 is a bare goBILDA 5000-series motor with no gearbox.
     */
    public static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    /** Intake motor power from 0.0 to 1.0, for pulling game elements in and pushing them out. */
    public static final double INTAKE_COLLECT_POWER = 1.0;
    public static final double INTAKE_EJECT_POWER = 0.6;

    /** The flywheel's top speed in RPM (motor free speed). Used to work out the starting F value below. */
    public static final double FLYWHEEL_MAX_RPM = 6000.0;

    /**
     * Flywheel PIDF coefficients, used to hold the target speed. TODO: TUNE ON OUR ROBOT.
     * These are a common starting point, not final answers:
     *   F (feedforward): the main push toward the target. 32767 divided by top speed in ticks per second.
     *   P (proportional): fixes the remaining error. Start at 10% of F.
     *   I (integral):     removes steady leftover error. Start at 1% of P, or leave 0.
     *   D (derivative):   calms oscillation. Usually leave 0.
     * Tune F first, then raise P until it reaches speed fast without wobbling.
     */
    public static final double FLYWHEEL_F = 32767.0 / (FLYWHEEL_MAX_RPM / 60.0 * FLYWHEEL_TICKS_PER_REV);
    public static final double FLYWHEEL_P = 0.1 * FLYWHEEL_F;
    public static final double FLYWHEEL_I = 0.01 * FLYWHEEL_P;
    public static final double FLYWHEEL_D = 0.0;

    /** How close (in RPM) the flywheel must be to its target to count as "at speed". */
    public static final double FLYWHEEL_TOLERANCE_RPM = 50.0;

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
    public final LimelightCamera limelight;
    public final Flywheel flywheel;
    public final Intake intake;

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

        // --- Limelight camera (finds AprilTags and other targets) ---
        Limelight3A limelightHardware = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelightHardware.pipelineSwitch(LIMELIGHT_START_PIPELINE);
        limelightHardware.start();

        limelight = new LimelightCamera(limelightHardware);

        // --- Flywheel (launches game pieces) ---
        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        // If the flywheel spins the wrong way, change this to REVERSE.
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        // FLOAT lets it coast down instead of braking hard, which is easier on the motor.
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // RUN_USING_ENCODER lets us set a speed instead of a power.
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel = new Flywheel(flywheelMotor, FLYWHEEL_TICKS_PER_REV, FLYWHEEL_TOLERANCE_RPM);
        flywheel.setPidf(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);

        // --- Intake (collects game elements) ---
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        // If the intake pushes elements OUT when collecting, change this to REVERSE.
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        // No encoder needed. We just set a power.
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = new Intake(intakeMotor, INTAKE_COLLECT_POWER, INTAKE_EJECT_POWER);
    }

    /** Stops everything that moves or runs. Add new mechanisms here as you create them. */
    public void stop() {
        drive.stop();
        flywheel.stop();
        intake.stop();
        limelight.stop();
    }
}
