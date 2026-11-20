package org.firstinspires.ftc.teamcode.oldmechanisms;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class Hardware {
    HardwareMap hardwareMap;

    //drive
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;

    public DcMotor intake;

    public CRServo transferServo;
    public DcMotor transfer;

    public DcMotorEx flywheel;

    public Limelight3A limelight;

    public IMU imu;
    
    public Limelight3A camera;
    
    //public GoBildaPinpointDriver odo; 

    public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //odo = hardwareMap.get(GoBildaPinpointDriver.class, "Odo");

        //backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

       intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       //transferServo = hardwareMap.get(CRServo.class, "intakeservo");

        flywheel = hardwareMap.get(DcMotorEx.class, "outtake");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //flywheel.setDirection(DcMotor.Direction.REVERSE);
        
         double F = 14;
       double P = 170;
       PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0.0,0.0,F);
        flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(1);
    }
}