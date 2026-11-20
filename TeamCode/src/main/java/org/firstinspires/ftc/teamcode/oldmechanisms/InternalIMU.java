package org.firstinspires.ftc.teamcode.oldmechanisms;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.oldmechanisms.Hardware;

public class InternalIMU {
    /*
    from coach pratt youtube channel
     */

    private Hardware hardware;

    public InternalIMU(Hardware hardware){
        this.hardware = hardware;
    }

    public double getYaw(){
        return hardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetYaw(){
        hardware.imu.resetYaw();
    }
}
