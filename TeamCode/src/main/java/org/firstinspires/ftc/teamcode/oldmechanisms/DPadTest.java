package org.firstinspires.ftc.teamcode.oldmechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DPadTest {
   private CRServo transferServo;
   private DcMotor flywheel;

   private ElapsedTime stateTimer = new ElapsedTime();

   private enum FlywheelState {
       IDLE,
       SPIN_UP,
       LAUNCH,
   };
   private FlywheelState flywheelState;

   public enum FlywheelTarget{
       SHORT,
       MEDIUM(),
       LONG,
   }
    private FlywheelTarget flywheelTarget;

    /*
     * Transfer Constants
     */
    private double TRANSFER_ON = 1.0;
    private double TRANSFER_OFF = 0.0;
    private double TRANSFER_TIME_SEC = 2;

   /*
    * Flywheel Constants
    */
    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double FLYWHEEL_LONG = 1.0;
    private double FLYWHEEL_MEDIDUM = 0.7;
    private double FLYWHEEL_SHORT = 0.6;
    private double FLYWHEEL_OFF = 0.0;
    private double FLYWHEEL_MAX_SPINUP_TIME_SEC = 3;

   public void init(HardwareMap hardwareMap){
       transferServo = hardwareMap.get(CRServo.class, "intakeservo");
       flywheel = hardwareMap.get(DcMotor.class, "outtake");

       flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       transferServo.setDirection(DcMotorSimple.Direction.REVERSE);

       // TODO: tune PIDF

       flywheelState = FlywheelState.IDLE;
       flywheelTarget = FlywheelTarget.MEDIUM;
       flywheel.setPower(FLYWHEEL_OFF);
       transferServo.setPower(TRANSFER_OFF);
   }

    public void update() {
       switch (flywheelState) {
           case IDLE:
               if (shotsRemaining > 0){
                   switch (flywheelTarget) {
                       case SHORT:
                           flywheel.setPower(FLYWHEEL_SHORT);
                           break;
                       case MEDIUM:
                           flywheel.setPower(FLYWHEEL_MEDIDUM);
                           break;
                       case LONG:
                           flywheel.setPower(FLYWHEEL_LONG);
                           break;
                   }
                   transferServo.setPower(TRANSFER_OFF);
                   stateTimer.reset();
                   flywheelState = FlywheelState.SPIN_UP;
               }
               break;
           case SPIN_UP:
               if (flywheelVelocity > 1.0 || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME_SEC){
                   stateTimer.reset();
                   transferServo.setPower(TRANSFER_ON);
                   flywheelState = FlywheelState.LAUNCH;
               }
               break;
           case LAUNCH:
               if ( stateTimer.seconds() > TRANSFER_TIME_SEC){
                   shotsRemaining--;
                   stateTimer.reset();
                   if( shotsRemaining>0) {
                       flywheelState = FlywheelState.LAUNCH;
                   }else {
                       transferServo.setPower(TRANSFER_OFF);
                       flywheel.setPower(FLYWHEEL_OFF);
                       flywheelState = FlywheelState.IDLE;
                   }
               }
               break;

       }
    }

    public void launchArtifact(int numberOfShots){
       if (flywheelState == FlywheelState.IDLE){
           shotsRemaining = numberOfShots;
       }
    }

    public void setTarget(FlywheelTarget target){
       flywheelTarget = target;
    }

    public boolean isBusy(){
       return flywheelState != FlywheelState.IDLE;
    }

    public double getFlywheelPower(){
       return flywheel.getPower();
    }
    public double getElapsedTime(){
        return stateTimer.seconds();
    }

    public String getState(){
        return flywheelState.toString();
    }
    public String getTarget(){
        return flywheelTarget.toString();
    }
}