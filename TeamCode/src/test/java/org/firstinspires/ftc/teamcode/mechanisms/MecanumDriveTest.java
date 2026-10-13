package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.mockito.AdditionalMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.junit.Test;

/**
 * Tests the wheel math in MecanumDrive using fake motors that just remember what power they were given.
 */
public class MecanumDriveTest {
    private static final double DELTA = 1e-9;

    private final DcMotor frontLeft = mock(DcMotor.class);
    private final DcMotor frontRight = mock(DcMotor.class);
    private final DcMotor backLeft = mock(DcMotor.class);
    private final DcMotor backRight = mock(DcMotor.class);

    private final MecanumDrive drive =
            new MecanumDrive(frontLeft, frontRight, backLeft, backRight, 1.0, 100.0);

    /** Checks the power each wheel was given. */
    private void assertPowers(double fl, double fr, double bl, double br) {
        verify(frontLeft).setPower(eq(fl, DELTA));
        verify(frontRight).setPower(eq(fr, DELTA));
        verify(backLeft).setPower(eq(bl, DELTA));
        verify(backRight).setPower(eq(br, DELTA));
    }

    @Test
    public void forwardSpinsAllWheelsForward() {
        drive.drive(1, 0, 0);
        assertPowers(1, 1, 1, 1);
    }

    @Test
    public void strafingRightSpinsDiagonalsOpposite() {
        drive.drive(0, 1, 0);
        assertPowers(1, -1, -1, 1);
    }

    @Test
    public void rotatingRightSpinsLeftWheelsForwardAndRightWheelsBackward() {
        drive.drive(0, 0, 1);
        assertPowers(1, -1, 1, -1);
    }

    @Test
    public void halfForwardGivesHalfPower() {
        drive.drive(0.5, 0, 0);
        assertPowers(0.5, 0.5, 0.5, 0.5);
    }

    @Test
    public void tooMuchInputShrinksAllWheelsTogether() {
        // forward + right would ask two wheels for 2.0. Everything is scaled by half instead,
        // so the robot still goes in the same direction.
        drive.drive(1, 1, 0);
        assertPowers(1, 0, 0, 1);
    }

    @Test
    public void maxSpeedLimitsPower() {
        MecanumDrive slowDrive =
                new MecanumDrive(frontLeft, frontRight, backLeft, backRight, 0.5, 100.0);
        slowDrive.drive(1, 0, 0);
        assertPowers(0.5, 0.5, 0.5, 0.5);
    }

    @Test
    public void stopSetsEveryWheelToZero() {
        drive.stop();
        assertPowers(0, 0, 0, 0);
    }

    // ---- Field-relative driving ----

    @Test
    public void fieldRelativeWithHeadingZeroActsLikeNormalDrive() {
        drive.driveFieldRelative(1, 0, 0, 0.0);
        assertPowers(1, 1, 1, 1);
    }

    @Test
    public void fieldRelativeWhenFacingLeftStrafesRightToGoFieldForward() {
        // Robot turned 90 degrees left, so "away from the driver" is the robot's right.
        drive.driveFieldRelative(1, 0, 0, Math.PI / 2);
        assertPowers(1, -1, -1, 1);
    }

    @Test
    public void fieldRelativeWhenFacingRightStrafesLeftToGoFieldForward() {
        drive.driveFieldRelative(1, 0, 0, -Math.PI / 2);
        assertPowers(-1, 1, 1, -1);
    }

    @Test
    public void fieldRelativeWhenFacingBackwardDrivesBackward() {
        drive.driveFieldRelative(1, 0, 0, Math.PI);
        assertPowers(-1, -1, -1, -1);
    }

    @Test
    public void fieldRelativeKeepsRotationInput() {
        drive.driveFieldRelative(0, 0, 1, Math.PI / 2);
        assertPowers(1, -1, 1, -1);
    }

    // ---- Measuring distance with the encoders ----

    @Test
    public void forwardInchesAveragesAllWheels() {
        drive.resetDistance();
        when(frontLeft.getCurrentPosition()).thenReturn(400);
        when(frontRight.getCurrentPosition()).thenReturn(400);
        when(backLeft.getCurrentPosition()).thenReturn(400);
        when(backRight.getCurrentPosition()).thenReturn(400);

        assertEquals(4.0, drive.getForwardInches(), DELTA);   // 400 counts / 100 per inch
    }

    @Test
    public void forwardInchesIsNegativeWhenGoingBackward() {
        drive.resetDistance();
        when(frontLeft.getCurrentPosition()).thenReturn(-200);
        when(frontRight.getCurrentPosition()).thenReturn(-200);
        when(backLeft.getCurrentPosition()).thenReturn(-200);
        when(backRight.getCurrentPosition()).thenReturn(-200);

        assertEquals(-2.0, drive.getForwardInches(), DELTA);
    }

    @Test
    public void strafingCountsAsZeroForwardDistance() {
        drive.resetDistance();
        when(frontLeft.getCurrentPosition()).thenReturn(400);
        when(frontRight.getCurrentPosition()).thenReturn(-400);
        when(backLeft.getCurrentPosition()).thenReturn(-400);
        when(backRight.getCurrentPosition()).thenReturn(400);

        assertEquals(0.0, drive.getForwardInches(), DELTA);
    }

    @Test
    public void turningCountsAsZeroForwardDistance() {
        drive.resetDistance();
        when(frontLeft.getCurrentPosition()).thenReturn(400);
        when(frontRight.getCurrentPosition()).thenReturn(-400);
        when(backLeft.getCurrentPosition()).thenReturn(400);
        when(backRight.getCurrentPosition()).thenReturn(-400);

        assertEquals(0.0, drive.getForwardInches(), DELTA);
    }

    @Test
    public void distanceIsMeasuredFromTheLastReset() {
        when(frontLeft.getCurrentPosition()).thenReturn(1000);
        when(frontRight.getCurrentPosition()).thenReturn(1000);
        when(backLeft.getCurrentPosition()).thenReturn(1000);
        when(backRight.getCurrentPosition()).thenReturn(1000);
        drive.resetDistance();   // the wheels have already rolled 1000 counts

        when(frontLeft.getCurrentPosition()).thenReturn(1300);
        when(frontRight.getCurrentPosition()).thenReturn(1300);
        when(backLeft.getCurrentPosition()).thenReturn(1300);
        when(backRight.getCurrentPosition()).thenReturn(1300);

        assertEquals(3.0, drive.getForwardInches(), DELTA);   // only the 300 since the reset
    }
}
