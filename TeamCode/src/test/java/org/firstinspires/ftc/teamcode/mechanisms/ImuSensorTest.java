package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.junit.Test;

public class ImuSensorTest {
    private final IMU imu = mock(IMU.class);
    private final ImuSensor sensor = new ImuSensor(imu);

    /** Pretend the IMU says the robot is turned this many radians. */
    private void facing(double yawRadians) {
        when(imu.getRobotYawPitchRollAngles()).thenReturn(
                new YawPitchRollAngles(AngleUnit.RADIANS, yawRadians, 0, 0, 0));
    }

    @Test
    public void headingInRadiansIsTheImuYaw() {
        facing(Math.PI / 2);
        assertEquals(Math.PI / 2, sensor.getHeadingRadians(), 1e-9);
    }

    @Test
    public void headingInDegreesMatchesRadians() {
        facing(Math.PI / 2);
        assertEquals(90.0, sensor.getHeadingDegrees(), 1e-9);
    }

    @Test
    public void negativeHeadingMeansTurnedRight() {
        facing(-Math.PI / 4);
        assertEquals(-45.0, sensor.getHeadingDegrees(), 1e-9);
    }

    @Test
    public void resetHeadingResetsTheImuYaw() {
        sensor.resetHeading();
        verify(imu).resetYaw();
    }
}
