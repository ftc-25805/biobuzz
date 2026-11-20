package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.oldmechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.oldmechanisms.Hardware;
import org.firstinspires.ftc.teamcode.oldmechanisms.MechanumDrive;


@TeleOp
public class RobotOldTeleopMechanumDrive extends OpMode {

    private Hardware hardware;
    private MechanumDrive drive;
    private Flywheel flywheel;
    private Camera camera;
    //private Limelight limelight;
    double maxPower = 1.0;
    

    double forward;
    double strafe;
    double rotate;

    @Override
    public void init(){
        hardware = new Hardware(hardwareMap);
        drive = new MechanumDrive(hardware);
        flywheel = new Flywheel(hardware);
        //limelight = new Limelight(hardware);
        
        //camera = new Camera(hardware);
        
       //hardware.odo.resetPosAndIMU();
       
    }

    @Override
    public void start(){
       //camera.start();
    }
    
    @Override
    public void loop() {
        //read input
        forward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;
        //autoalaign = gamepad1.left
        
        if(gamepad1.a){
            if(maxPower >= 0.9){
                maxPower = 0.1;
            }
            else {
                maxPower =1.0;
            }
        }

        
        if (gamepad2.dpad_down) {
            hardware.intake.setPower(1.0);

        } else {
            if (flywheel.getIntakePower() != 0){
                
            } else {
                hardware.intake.setPower(0.0);
            }

        }
        
        if (gamepad2.dpad_up || gamepad2.left_trigger > 0) {
            if(gamepad2.left_trigger > 0) {
                hardware.transfer.setPower(-0.5);
            } else {
                hardware.transfer.setPower(0.5);
            }

        } else {
            if (flywheel.getTransferPower() != 0){

            } else {
                hardware.transfer.setPower(0);
            }

        }

        if (gamepad2.b) {
            flywheel.setTarget(Flywheel.FlywheelTarget.SHORT);
            flywheel.turnOnFlywheel();
        }
        
        if (gamepad2.y) {
            flywheel.setTarget(Flywheel.FlywheelTarget.LONG);
             flywheel.turnOnFlywheel();
        }

        if (gamepad2.x)  {
            flywheel.setTarget(Flywheel.FlywheelTarget.MEDIUM);
             flywheel.turnOnFlywheel();
        }
        
        if(gamepad2.right_trigger>0) {
              flywheel.turnOffFlywheel();
        }
        
        if(gamepad2.a){
            flywheel.setTarget(Flywheel.FlywheelTarget.UNSTUCK);
             flywheel.turnOnFlywheel();
        }
        
        /*if(gamepad2.left_trigger>0) {
              flywheel.turnOnTransferReverse();
        }
        else {
            flywheel.turnOffTransfer();
        }*/
        
        /*if (gamepad2.a)  {
            flywheel.launchArtifact(1);
        }*/

        /*if (gamepad2.right_trigger > 0)  {
            flywheel.turnOnTFlywheel();
        } else {
          if(flywheel.getFlywheelPower() != 0) {
          } else {
              flywheel.turnOffFlywheel();
          }
        }*/
        /*if (gamepad2.left_trigger > 0)  {
            intakeservo.setPower(-1.0);

        } else {
            intakeservo.setPower(0.0);
        }*/

        
        
        if (gamepad1.right_trigger > 0) {
            //right trigger is pressed
            //if (limelight.getLLResult() != null){
            //    //have a target 
            //} else { // we don't have a target :(
            //}
        } else {
            //right trigger was not pressed
            
        if (gamepad1.a) {
            //hardware.odo.resetPosAndIMU();
        }
        }
        
        drive.drive(forward, strafe, rotate, maxPower);
        flywheel.update();
        //hardware.odo.update();
        //camera.loop();

        /*LLResult llResult = camera.getLLResult();
        if (llResult != null) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.getOrientation().getYaw());
            telemetry.addData("Td", camera.getDistanceFromTag(llResult.getTa()));
        }*/

        telemetry.addData("flywheel velo: ", flywheel.getFlywheelPower());
        telemetry.addData("flywheel timer: ", flywheel.getElapsedTime());
        telemetry.addData("flywheel state: ", flywheel.getState());
        telemetry.addData("flywheel target: ", flywheel.getTarget());
         //telemetry.addData("trans powert: ", flywheel.getTransferPower());
          telemetry.addData("Intake power: ", flywheel.getIntakePower());
          telemetry.addData("MaxPower: ", maxPower);
        //telemetry.addData("IMU: ", imu);
        //telemetry.addData("Limelight : ", limelight.getLLResult() != null);
        
        //telemetry.addData("EncoderX", hardware.odo.getPosX(DistanceUnit.INCH));
        //telemetry.addData("EncoderY", hardware.odo.getPosY(DistanceUnit.INCH));
        //telemetry.addData("OdoHeading", hardware.odo.getHeading(AngleUnit.DEGREES));
    
        
        
        
        telemetry.addData("FL: ", hardware.frontLeftDrive.getCurrentPosition());
       telemetry.addData("FR: ", hardware.frontRightDrive.getCurrentPosition());
        telemetry.addData("BL: ", hardware.backLeftDrive.getCurrentPosition());
         telemetry.addData("BR: ", hardware.backRightDrive.getCurrentPosition());
    }

}