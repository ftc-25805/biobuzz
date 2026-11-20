package org.firstinspires.ftc.teamcode.oldmechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {
    private final Hardware hardware;

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
       UNSTUCK,
   }
    private FlywheelTarget flywheelTarget;

    /*
     * Transfer Constants
     */
    private double TRANSFER_ON = -1.0;
    private double TRANSFER_OFF = 0.0;
    private double TRANSFER_TIME_SEC = 0.7;


   /*
    * Flywheel Constants
    */
    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double FLYWHEEL_LONG = 0.1;
    private double FLYWHEEL_MEDIDUM = 0.05;
    private double FLYWHEEL_SHORT = 0.025;
    private double FLYWHEEL_OFF = 0.0;
    private double FLYWHEEL_MAX_SPINUP_TIME_SEC = 1.3;
    private double currentIntakePower = 0.0; 
    private double currentTransferPower = 0.0; 
     private double currentFlywheelPower = 0.0;


    public Flywheel(Hardware hardware) {
        this.hardware = hardware;

       // TODO: tune PIDF

       flywheelState = FlywheelState.IDLE;
       flywheelTarget = FlywheelTarget.MEDIUM;
       currentFlywheelPower = FLYWHEEL_OFF;

        hardware.flywheel.setPower(currentFlywheelPower);
       currentTransferPower  = TRANSFER_OFF;
       // hardware.transferServo.setPower(currentTransferPower);
       currentIntakePower = 0.;
        hardware.intake.setPower(currentIntakePower);
   }

    public void update() {
       switch (flywheelState) {
           case IDLE:
               if (shotsRemaining > 0){
                   FLYWHEEL_MAX_SPINUP_TIME_SEC = 2.0;
                   switch (flywheelTarget) {
                       case SHORT:
                           currentFlywheelPower = FLYWHEEL_SHORT;
                           break;
                       case MEDIUM:
                           currentFlywheelPower = FLYWHEEL_MEDIDUM;
                           break;
                       case LONG:
                           currentFlywheelPower = FLYWHEEL_LONG;
                           break;
                   }
                   hardware.flywheel.setPower(currentFlywheelPower);
                   
                   currentTransferPower = TRANSFER_OFF;
                   hardware.transferServo.setPower(currentTransferPower);
                   currentIntakePower = 0.;
                   hardware.intake.setPower(currentIntakePower);
                   stateTimer.reset();
                   flywheelState = FlywheelState.SPIN_UP;
               }
               break;
           case SPIN_UP:
               if (flywheelVelocity > 1.0 || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME_SEC){
                   FLYWHEEL_MAX_SPINUP_TIME_SEC = 1.0;
                   stateTimer.reset();
                   currentIntakePower = 1.;

                   hardware.intake.setPower(currentIntakePower);
                    currentTransferPower = TRANSFER_ON;
                   hardware.transferServo.setPower(currentTransferPower);
                   flywheelState = FlywheelState.LAUNCH;
               }
               break;
           case LAUNCH:
               if ( stateTimer.seconds() > TRANSFER_TIME_SEC){
                   shotsRemaining--;
                   stateTimer.reset();
                   if( shotsRemaining>0) {
                       currentTransferPower = TRANSFER_OFF;

                       hardware.transferServo.setPower(currentTransferPower);
                       hardware.intake.setPower(0.0);
                       flywheelState = FlywheelState.SPIN_UP;
                   }else {
                       currentTransferPower = TRANSFER_OFF;
                       hardware.transferServo.setPower(currentTransferPower);
                       currentIntakePower = 0.;
                       hardware.intake.setPower(currentIntakePower);
                       currentFlywheelPower = FLYWHEEL_OFF;
                       hardware.flywheel.setPower(currentFlywheelPower);
                       flywheelState = FlywheelState.IDLE;
                   }
               }
               break;

       }
    }

    public void launchArtifact(int numberOfShots){
        shotsRemaining += numberOfShots;
        if (shotsRemaining>3){
            shotsRemaining=3;
        }
    
       //if (flywheelState == FlywheelState.IDLE){
        //   shotsRemaining = numberOfShots;
       //}
    }

    public void setTarget(FlywheelTarget target){
       flywheelTarget = target;
    }

    public boolean isBusy(){
       return flywheelState != FlywheelState.IDLE;
    }

    public double getFlywheelPower(){
       return hardware.flywheel.getVelocity();
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
    
    public double getIntakePower(){
        return currentIntakePower;
    }
    
    public void turnOnTransfer(){
        hardware.transferServo.setPower(-1.0);
     }
     
     public void turnOnTransferReverse(){
         hardware.transferServo.setPower(1.0);
     }
     public void turnOffTransfer(){
         hardware.transferServo.setPower(0.0);
     }
     public double getTransferPower(){
        return currentTransferPower;
    }
    
    public void turnOnFlywheel(){
        switch (flywheelTarget) {
                       case SHORT:
                           //hardware.flywheel.setVelocity(1700);
                           hardware.flywheel.setVelocity(1000);
                           break;
                       case MEDIUM:
                           //hardware.flywheel.setVelocity(1850);
                           hardware.flywheel.setVelocity(1100);
                           break;
                       case LONG:
                           //hardware.flywheel.setVelocity(2350);
                           hardware.flywheel.setVelocity(1200);
                           break;
                           case UNSTUCK:
                           hardware.flywheel.setVelocity(-2000);
                           break;
                   }
        
     }
     public void turnOffFlywheel(){
         hardware.flywheel.setVelocity(0);
     }
    
    public int getShotsRemaining(){
        return shotsRemaining;
    }
}