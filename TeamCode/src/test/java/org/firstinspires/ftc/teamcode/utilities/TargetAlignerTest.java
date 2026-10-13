package org.firstinspires.ftc.teamcode.utilities;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.AdditionalMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.firstinspires.ftc.teamcode.mechanisms.LimelightCamera;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.junit.Test;

/**
 * Tests turning to face a target. tx is how many degrees the target is to the RIGHT of center,
 * and drive's positive "rotate" turns RIGHT, so a target on the right means a positive rotate.
 */
public class TargetAlignerTest {
    private static final double DELTA = 1e-9;

    private final MecanumDrive drive = mock(MecanumDrive.class);
    private final LimelightCamera camera = mock(LimelightCamera.class);
    private final TargetAligner aligner = new TargetAligner(drive, camera);

    /** Pretend the camera sees a target this many degrees to the right of center. */
    private void seesTargetAt(double txDegrees) {
        when(camera.hasTarget()).thenReturn(true);
        when(camera.getTargetXDegrees()).thenReturn(txDegrees);
    }

    @Test
    public void updateReadsTheCameraFirst() {
        seesTargetAt(10);
        aligner.update();
        verify(camera).update();
    }

    @Test
    public void targetOnTheRightTurnsRight() {
        seesTargetAt(10);
        aligner.update();

        // 10 degrees * gain, as long as that is between the min and max power
        verify(drive).drive(eq(0.0, DELTA), eq(0.0, DELTA), eq(TargetAligner.TURN_KP * 10, DELTA));
        assertFalse(aligner.isAligned());
    }

    @Test
    public void targetOnTheLeftTurnsLeft() {
        seesTargetAt(-10);
        aligner.update();

        verify(drive).drive(eq(0.0, DELTA), eq(0.0, DELTA), eq(-TargetAligner.TURN_KP * 10, DELTA));
    }

    @Test
    public void centeredTargetMeansAlignedAndNoTurn() {
        seesTargetAt(TargetAligner.TOLERANCE_DEGREES - 0.5);
        aligner.update();

        verify(drive).drive(eq(0.0, DELTA), eq(0.0, DELTA), eq(0.0, DELTA));
        assertTrue(aligner.isAligned());
    }

    @Test
    public void justOutsideToleranceGetsTheMinimumTurn() {
        seesTargetAt(TargetAligner.TOLERANCE_DEGREES + 0.1);
        aligner.update();

        verify(drive).drive(eq(0.0, DELTA), eq(0.0, DELTA), eq(TargetAligner.MIN_TURN_POWER, DELTA));
    }

    @Test
    public void farOffTargetIsLimitedToMaxTurnPower() {
        seesTargetAt(60);
        aligner.update();

        verify(drive).drive(eq(0.0, DELTA), eq(0.0, DELTA), eq(TargetAligner.MAX_TURN_POWER, DELTA));
    }

    @Test
    public void farOffTargetOnTheLeftIsLimitedToMaxTurnPowerTheOtherWay() {
        seesTargetAt(-60);
        aligner.update();

        verify(drive).drive(eq(0.0, DELTA), eq(0.0, DELTA), eq(-TargetAligner.MAX_TURN_POWER, DELTA));
    }

    @Test
    public void noTargetMeansNoTurnAndNotAligned() {
        when(camera.hasTarget()).thenReturn(false);
        aligner.update();

        verify(drive).drive(eq(0.0, DELTA), eq(0.0, DELTA), eq(0.0, DELTA));
        assertFalse(aligner.isAligned());
        assertFalse(aligner.canSeeTarget());
    }

    @Test
    public void driverKeepsControlOfMovementWhileAiming() {
        seesTargetAt(10);
        aligner.update(0.5, -0.3);

        verify(drive).drive(eq(0.5, DELTA), eq(-0.3, DELTA), eq(TargetAligner.TURN_KP * 10, DELTA));
    }

    @Test
    public void driverStillMovesWhenTheTargetIsLost() {
        when(camera.hasTarget()).thenReturn(false);
        aligner.update(0.5, -0.3);

        verify(drive).drive(eq(0.5, DELTA), eq(-0.3, DELTA), eq(0.0, DELTA));
    }

    // ---- Aiming at one specific tag ----

    @Test
    public void aimsAtTheChosenTagInsteadOfWhateverElseIsVisible() {
        aligner.setTagId(24);
        when(camera.canSeeTag(24)).thenReturn(true);
        when(camera.getTagXDegrees(24)).thenReturn(-10.0);
        when(camera.hasTarget()).thenReturn(true);
        when(camera.getTargetXDegrees()).thenReturn(30.0);   // some OTHER target

        aligner.update();

        verify(drive).drive(eq(0.0, DELTA), eq(0.0, DELTA), eq(-TargetAligner.TURN_KP * 10, DELTA));
        assertEquals(-10.0, aligner.getErrorDegrees(), DELTA);
    }

    @Test
    public void doesNotTurnWhenOnlyOtherTagsAreVisible() {
        aligner.setTagId(24);
        when(camera.canSeeTag(24)).thenReturn(false);
        when(camera.hasTarget()).thenReturn(true);
        when(camera.getTargetXDegrees()).thenReturn(30.0);

        aligner.update();

        verify(drive).drive(eq(0.0, DELTA), eq(0.0, DELTA), eq(0.0, DELTA));
        assertFalse(aligner.canSeeTarget());
    }

    @Test
    public void aimAtAnyTargetGoesBackToUsingEverything() {
        aligner.setTagId(24);
        aligner.aimAtAnyTarget();
        seesTargetAt(10);

        aligner.update();

        assertEquals(10.0, aligner.getErrorDegrees(), DELTA);
        assertTrue(aligner.canSeeTarget());
    }

    @Test
    public void stopStopsTheDriveAndClearsAligned() {
        seesTargetAt(0);
        aligner.update();
        assertTrue(aligner.isAligned());

        aligner.stop();

        verify(drive).stop();
        assertFalse(aligner.isAligned());
    }
}
