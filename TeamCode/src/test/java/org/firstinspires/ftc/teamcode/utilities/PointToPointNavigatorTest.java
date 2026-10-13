package org.firstinspires.ftc.teamcode.utilities;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointOdometry;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

/**
 * Tests which way the robot is told to move for different targets.
 *
 * Reminder of the conventions:
 *   x = inches forward, y = inches LEFT, heading positive = turned LEFT.
 *   drive(forward, right, rotate): "right" is positive to the RIGHT and "rotate" is
 *   positive to the RIGHT (clockwise). So going left means a NEGATIVE right/rotate.
 */
public class PointToPointNavigatorTest {
    private static final double DELTA = 1e-9;

    private final MecanumDrive drive = mock(MecanumDrive.class);
    private final PinpointOdometry odometry = mock(PinpointOdometry.class);
    private final PointToPointNavigator navigator = new PointToPointNavigator(drive, odometry);

    /** Pretend the robot is at this spot, facing this way (heading in radians). */
    private void robotAt(double xInches, double yInches, double headingRadians) {
        when(odometry.getXInches()).thenReturn(xInches);
        when(odometry.getYInches()).thenReturn(yInches);
        when(odometry.getHeadingRadians()).thenReturn(headingRadians);
    }

    /** The forward / right / rotate values the navigator most recently gave the drive. */
    private double[] lastDriveCommand() {
        ArgumentCaptor<Double> forward = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> right = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> rotate = ArgumentCaptor.forClass(Double.class);
        verify(drive, atLeastOnce()).drive(forward.capture(), right.capture(), rotate.capture());
        return new double[]{forward.getValue(), right.getValue(), rotate.getValue()};
    }

    @Test
    public void targetStraightAheadDrivesForward() {
        robotAt(0, 0, 0);
        navigator.setTarget(24, 0, 0);
        navigator.update();

        double[] cmd = lastDriveCommand();
        assertTrue(cmd[0] > 0);
        assertEquals(0.0, cmd[1], DELTA);
        assertEquals(0.0, cmd[2], DELTA);
    }

    @Test
    public void targetBehindDrivesBackward() {
        robotAt(0, 0, 0);
        navigator.setTarget(-24, 0, 0);
        navigator.update();

        assertTrue(lastDriveCommand()[0] < 0);
    }

    @Test
    public void targetToTheLeftStrafesLeft() {
        robotAt(0, 0, 0);
        navigator.setTarget(0, 24, 0);
        navigator.update();

        double[] cmd = lastDriveCommand();
        assertEquals(0.0, cmd[0], DELTA);
        assertTrue("strafing left means a negative 'right'", cmd[1] < 0);
    }

    @Test
    public void whenFacingLeftTheFieldForwardTargetIsToTheRobotsRight() {
        robotAt(0, 0, Math.PI / 2);
        navigator.setTarget(24, 0, 90);
        navigator.update();

        double[] cmd = lastDriveCommand();
        assertEquals(0.0, cmd[0], 1e-6);
        assertTrue(cmd[1] > 0);
    }

    @Test
    public void whenFacingBackwardTheFieldForwardTargetIsBehindTheRobot() {
        robotAt(0, 0, Math.PI);
        navigator.setTarget(24, 0, 180);
        navigator.update();

        assertTrue(lastDriveCommand()[0] < 0);
    }

    @Test
    public void targetHeadingToTheLeftTurnsLeft() {
        robotAt(0, 0, 0);
        navigator.setTarget(0, 0, 90);
        navigator.update();

        assertTrue("turning left means a negative 'rotate'", lastDriveCommand()[2] < 0);
    }

    @Test
    public void targetHeadingToTheRightTurnsRight() {
        robotAt(0, 0, 0);
        navigator.setTarget(0, 0, -90);
        navigator.update();

        assertTrue(lastDriveCommand()[2] > 0);
    }

    @Test
    public void turnsTheShortWayAroundTheCircle() {
        // Facing 170 degrees, target -170 degrees: the short way is 20 degrees LEFT, not 340 right.
        robotAt(0, 0, Math.toRadians(170));
        navigator.setTarget(0, 0, -170);
        navigator.update();

        assertTrue(lastDriveCommand()[2] < 0);
    }

    @Test
    public void farTargetIsLimitedToMaxDrivePower() {
        robotAt(0, 0, 0);
        navigator.setTarget(1000, 0, 0);
        navigator.update();

        double[] cmd = lastDriveCommand();
        assertEquals(PointToPointNavigator.MAX_DRIVE_POWER, Math.hypot(cmd[0], cmd[1]), 1e-6);
    }

    @Test
    public void closeTargetStillGetsTheMinimumPower() {
        // Just outside the tolerance, where the proportional push alone would be tiny.
        robotAt(0, 0, 0);
        navigator.setTarget(PointToPointNavigator.POSITION_TOLERANCE_INCHES + 0.2, 0, 0);
        navigator.update();

        double[] cmd = lastDriveCommand();
        assertTrue(Math.hypot(cmd[0], cmd[1]) >= PointToPointNavigator.MIN_DRIVE_POWER - 1e-9);
    }

    @Test
    public void slowsDownAsItGetsCloser() {
        robotAt(0, 0, 0);
        navigator.setTarget(1000, 0, 0);
        navigator.update();
        double farPower = lastDriveCommand()[0];

        robotAt(990, 0, 0);
        navigator.setTarget(1000, 0, 0);   // 10 inches left
        navigator.update();
        double nearPower = lastDriveCommand()[0];

        assertTrue(nearPower < farPower);
    }

    @Test
    public void perMoveSpeedLimitsAreUsed() {
        robotAt(0, 0, 0);
        navigator.setTarget(1000, 0, 0, 0.3, 0.2);
        navigator.update();

        double[] cmd = lastDriveCommand();
        assertEquals(0.3, Math.hypot(cmd[0], cmd[1]), 1e-6);
    }

    @Test
    public void perMoveTurnLimitIsUsed() {
        robotAt(0, 0, 0);
        navigator.setTarget(0, 0, 180, 0.5, 0.2);
        navigator.update();

        assertEquals(0.2, Math.abs(lastDriveCommand()[2]), 1e-6);
    }

    @Test
    public void inPositionButNotFacingTheRightWayOnlyTurns() {
        robotAt(10, 10, 0);
        navigator.setTarget(10, 10, 45);
        navigator.update();

        double[] cmd = lastDriveCommand();
        assertEquals(0.0, cmd[0], DELTA);
        assertEquals(0.0, cmd[1], DELTA);
        assertTrue(cmd[2] != 0.0);
    }

    // ---- Arriving ----

    @Test
    public void notFinishedUntilItHasSettled() throws InterruptedException {
        robotAt(24, 0, 0);
        navigator.setTarget(24, 0, 0);

        navigator.update();
        assertFalse("just arrived, still settling", navigator.isAtTarget());

        Thread.sleep((long) (PointToPointNavigator.SETTLE_TIME_SECONDS * 1000) + 100);
        navigator.update();
        assertTrue(navigator.isAtTarget());
    }

    @Test
    public void stopsTheRobotWhileSettling() {
        robotAt(24, 0, 0);
        navigator.setTarget(24, 0, 0);
        navigator.update();

        verify(drive, atLeastOnce()).stop();
    }

    @Test
    public void leavingToleranceRestartsTheSettleTimer() throws InterruptedException {
        robotAt(24, 0, 0);
        navigator.setTarget(24, 0, 0);
        navigator.update();                                   // arrives, starts settling
        Thread.sleep((long) (PointToPointNavigator.SETTLE_TIME_SECONDS * 1000) + 100);

        robotAt(30, 0, 0);                                    // drifts away before finishing
        navigator.update();
        assertFalse(navigator.isAtTarget());

        robotAt(24, 0, 0);                                    // back in tolerance
        navigator.update();
        assertFalse("the settle timer should have restarted", navigator.isAtTarget());
    }

    @Test
    public void newTargetIsNotAtTargetEvenIfAlreadyThere() {
        robotAt(24, 0, 0);
        navigator.setTarget(24, 0, 0);
        assertFalse(navigator.isAtTarget());
    }

    @Test
    public void stopStopsTheDrive() {
        navigator.stop();
        verify(drive).stop();
    }

    @Test
    public void neverCommandsMoreThanFullPower() {
        robotAt(0, 0, 0);
        navigator.setTarget(500, 500, 180);
        navigator.update();

        double[] cmd = lastDriveCommand();
        for (double value : cmd) {
            if (Math.abs(value) > 1.0) {
                fail("power out of range: " + value);
            }
        }
    }
}
