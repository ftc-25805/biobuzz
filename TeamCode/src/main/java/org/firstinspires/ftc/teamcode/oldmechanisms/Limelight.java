package org.firstinspires.ftc.teamcode.oldmechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Limelight {

    private Hardware hardware;

    /*
    Plug on Blue Port

   Pipeline Type: AprilTags
   Source: Camera
   Resolution: 1280x960 40fps
   Stream Orient:
   Black Level Offset: 0
   Sensor Gain: 15
   Flicker Correction: 60Hz
   Red/Blue Balance: do not change
   Exposer:  ~62 (test while moving)
   Family: AprilTags Classic 36h11
   Marker Size: 101.6
   Detector Downscale: 4
   Quality Thresold:2
   ID Filters: Add one tag
   Sort Mode: Largest
   Target Mode : Single

   Advanced:
   Full 3D Targeting: Yes

   MegaTag:
   Snap RObot to Field: Yes

     */

    public Limelight(Hardware hardware){
        this.hardware = hardware;
    }

    public void start(){
        //hardware.limelight.start();
    }

    public void loop(){
        //YawPitchRollAngles orients = hardware.imu.getRobotYawPitchRollAngles();
        //hardware.limelight.updateRobotOrientation(orients.getYaw());
    }

    public LLResult getLLResult() {
        LLResult llResult = hardware.limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
        return llResult;
        }
        return null;
    }

    public double getDistanceFromTag(double targetArea) {
        double scale = 30665.95;
        double distance = scale / targetArea;
        return distance;
    }

}
