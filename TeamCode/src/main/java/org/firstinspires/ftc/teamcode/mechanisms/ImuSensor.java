package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * The IMU is a sensor inside the Control Hub that knows which way the robot
 * is facing. We call that direction the "heading".
 */
public class ImuSensor {
    private final IMU imu;

    /** @param imu the IMU, already set up by RobotHardware */
    public ImuSensor(IMU imu) {
        this.imu = imu;
    }

    /** The direction the robot is facing, in radians (0 = the way it faced at the last reset). */
    public double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** The direction the robot is facing, in degrees (positive = turned left). */
    public double getHeadingDegrees() {
        return Math.toDegrees(getHeadingRadians());
    }

    /** Makes the robot's current direction count as "forward" (heading 0). */
    public void resetHeading() {
        imu.resetYaw();
    }
}
