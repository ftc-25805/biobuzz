package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

public class PinpointOdometryTest {
    private final GoBildaPinpointDriver pinpoint = mock(GoBildaPinpointDriver.class);
    private final PinpointOdometry odometry = new PinpointOdometry(pinpoint);

    @Test
    public void positionIsReportedInInches() {
        // 25.4 mm is exactly 1 inch, and 50.8 mm is 2 inches.
        when(pinpoint.getPosition())
                .thenReturn(new Pose2D(DistanceUnit.MM, 25.4, 50.8, AngleUnit.DEGREES, 90));

        assertEquals(1.0, odometry.getXInches(), 1e-9);
        assertEquals(2.0, odometry.getYInches(), 1e-9);
    }

    @Test
    public void headingIsReportedInRadiansAndDegrees() {
        when(pinpoint.getPosition())
                .thenReturn(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));

        assertEquals(Math.PI / 2, odometry.getHeadingRadians(), 1e-9);
        assertEquals(90.0, odometry.getHeadingDegrees(), 1e-9);
    }

    @Test
    public void updateAsksThePinpointForFreshNumbers() {
        odometry.update();
        verify(pinpoint).update();
    }

    @Test
    public void setPositionSendsAPoseInTheRightUnits() {
        odometry.setPosition(10, -4, 2.0);

        ArgumentCaptor<Pose2D> captor = ArgumentCaptor.forClass(Pose2D.class);
        verify(pinpoint).setPosition(captor.capture());
        assertEquals(10.0, captor.getValue().getX(DistanceUnit.INCH), 1e-9);
        assertEquals(-4.0, captor.getValue().getY(DistanceUnit.INCH), 1e-9);
        assertEquals(2.0, captor.getValue().getHeading(AngleUnit.RADIANS), 1e-9);
    }

    @Test
    public void resetPositionSetsZeroZeroFacingForward() {
        odometry.resetPosition();

        ArgumentCaptor<Pose2D> captor = ArgumentCaptor.forClass(Pose2D.class);
        verify(pinpoint).setPosition(captor.capture());
        assertEquals(0.0, captor.getValue().getX(DistanceUnit.INCH), 1e-9);
        assertEquals(0.0, captor.getValue().getY(DistanceUnit.INCH), 1e-9);
        assertEquals(0.0, captor.getValue().getHeading(AngleUnit.RADIANS), 1e-9);
    }

    @Test
    public void healthyOnlyWhenPinpointIsReady() {
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        assertTrue(odometry.isHealthy());
    }

    @Test
    public void notHealthyWhenPodsAreMissing() {
        when(pinpoint.getDeviceStatus())
                .thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED);
        assertFalse(odometry.isHealthy());
        assertEquals("FAULT_NO_PODS_DETECTED", odometry.getStatusText());
    }
}
