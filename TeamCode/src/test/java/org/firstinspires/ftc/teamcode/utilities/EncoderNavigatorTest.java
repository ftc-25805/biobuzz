package org.firstinspires.ftc.teamcode.utilities;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoInteractions;
import static org.mockito.Mockito.when;

import org.firstinspires.ftc.teamcode.mechanisms.ImuSensor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

/**
 * Tests the drive-a-distance and turn-to-a-heading moves.
 * As always: positive heading is LEFT, but drive's positive "rotate" is RIGHT.
 */
public class EncoderNavigatorTest {
    private static final double DELTA = 1e-9;

    private final MecanumDrive drive = mock(MecanumDrive.class);
    private final ImuSensor imu = mock(ImuSensor.class);
    private final EncoderNavigator navigator = new EncoderNavigator(drive, imu);

    /** Pretend the robot has rolled this far and is facing this way (degrees, positive = left). */
    private void robotHas(double rolledInches, double headingDegrees) {
        when(drive.getForwardInches()).thenReturn(rolledInches);
        when(imu.getHeadingRadians()).thenReturn(Math.toRadians(headingDegrees));
    }

    private double[] lastDriveCommand() {
        ArgumentCaptor<Double> forward = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> right = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> rotate = ArgumentCaptor.forClass(Double.class);
        verify(drive, atLeastOnce()).drive(forward.capture(), right.capture(), rotate.capture());
        return new double[]{forward.getValue(), right.getValue(), rotate.getValue()};
    }

    // ---- Driving straight ----

    @Test
    public void driveStraightStartsMeasuringFromZero() {
        robotHas(0, 0);
        navigator.driveStraight(24, 0);
        verify(drive).resetDistance();
    }

    @Test
    public void driveStraightRollsForwardWithoutStrafingOrTurning() {
        robotHas(0, 0);
        navigator.driveStraight(24, 0);
        navigator.update();

        double[] cmd = lastDriveCommand();
        assertTrue(cmd[0] > 0);
        assertEquals(0.0, cmd[1], DELTA);
        assertEquals(0.0, cmd[2], DELTA);
    }

    @Test
    public void negativeDistanceRollsBackward() {
        robotHas(0, 0);
        navigator.driveStraight(-24, 0);
        navigator.update();

        assertTrue(lastDriveCommand()[0] < 0);
    }

    @Test
    public void slowsDownWhenCloseToTheDistance() {
        robotHas(0, 0);
        navigator.driveStraight(24, 0);
        navigator.update();
        double farPower = lastDriveCommand()[0];

        robotHas(22, 0);   // 2 inches left
        navigator.update();
        double nearPower = lastDriveCommand()[0];

        assertTrue(nearPower < farPower);
    }

    @Test
    public void farDistanceIsLimitedToMaxDrivePower() {
        robotHas(0, 0);
        navigator.driveStraight(500, 0);
        navigator.update();

        assertEquals(EncoderNavigator.MAX_DRIVE_POWER, lastDriveCommand()[0], DELTA);
    }

    @Test
    public void finishesWhenTheDistanceIsReached() {
        robotHas(0, 0);
        navigator.driveStraight(24, 0);
        navigator.update();
        assertFalse(navigator.isAtTarget());

        robotHas(24, 0);
        navigator.update();
        assertTrue(navigator.isAtTarget());
        verify(drive, atLeastOnce()).stop();
    }

    @Test
    public void finishesWhenGoingBackwardAndReachingTheDistance() {
        robotHas(0, 0);
        navigator.driveStraight(-24, 0);
        robotHas(-24, 0);
        navigator.update();

        assertTrue(navigator.isAtTarget());
    }

    // ---- Holding a heading while driving ----

    @Test
    public void steersRightWhenThePointedTooFarLeft() {
        // Heading is 10 degrees left of where we want to face, so steer right (positive rotate).
        robotHas(0, 10);
        navigator.driveStraight(24, 0);
        navigator.update();

        assertTrue(lastDriveCommand()[2] > 0);
    }

    @Test
    public void steersLeftWhenPointedTooFarRight() {
        robotHas(0, -10);
        navigator.driveStraight(24, 0);
        navigator.update();

        assertTrue(lastDriveCommand()[2] < 0);
    }

    @Test
    public void steeringIsTheSameWhenDrivingBackward() {
        robotHas(0, 10);
        navigator.driveStraight(-24, 0);
        navigator.update();

        assertTrue("still needs to steer right to come back onto heading", lastDriveCommand()[2] > 0);
    }

    @Test
    public void holdsANonZeroHeading() {
        robotHas(0, -45);   // already on the -45 degree heading
        navigator.driveStraight(24, -45);
        navigator.update();

        assertEquals(0.0, lastDriveCommand()[2], DELTA);
    }

    // ---- Turning ----

    @Test
    public void turnToTheLeftHeadingTurnsLeftWithoutDriving() {
        robotHas(0, 0);
        navigator.turnToHeading(90);
        navigator.update();

        double[] cmd = lastDriveCommand();
        assertEquals(0.0, cmd[0], DELTA);
        assertEquals(0.0, cmd[1], DELTA);
        assertTrue("turning left means a negative 'rotate'", cmd[2] < 0);
    }

    @Test
    public void turnToTheRightHeadingTurnsRight() {
        robotHas(0, 0);
        navigator.turnToHeading(-90);
        navigator.update();

        assertTrue(lastDriveCommand()[2] > 0);
    }

    @Test
    public void bigTurnIsLimitedToMaxTurnPower() {
        robotHas(0, 0);
        navigator.turnToHeading(120);
        navigator.update();

        assertEquals(EncoderNavigator.MAX_TURN_POWER, Math.abs(lastDriveCommand()[2]), DELTA);
    }

    @Test
    public void turnsTheShortWayAroundTheCircle() {
        robotHas(0, 170);
        navigator.turnToHeading(-170);
        navigator.update();

        assertTrue("20 degrees left is shorter than 340 right", lastDriveCommand()[2] < 0);
    }

    @Test
    public void turnFinishesWhenOnHeading() {
        robotHas(0, 0);
        navigator.turnToHeading(90);
        navigator.update();
        assertFalse(navigator.isAtTarget());

        robotHas(0, 90);
        navigator.update();
        assertTrue(navigator.isAtTarget());
    }

    @Test
    public void headingErrorIsTheDegreesLeftToTurn() {
        robotHas(0, 30);
        navigator.turnToHeading(90);
        assertEquals(60.0, navigator.headingErrorDegrees(), 1e-6);
    }

    // ---- Idle ----

    @Test
    public void doesNothingBeforeAMoveIsStarted() {
        robotHas(0, 0);
        navigator.update();

        verifyNoInteractions(drive);
        assertTrue(navigator.isAtTarget());
    }

    @Test
    public void stopCancelsTheMove() {
        robotHas(0, 0);
        navigator.driveStraight(24, 0);
        navigator.stop();
        verify(drive).stop();
    }
}
