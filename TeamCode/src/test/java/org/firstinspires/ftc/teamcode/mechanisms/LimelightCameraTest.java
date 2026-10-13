package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;

public class LimelightCameraTest {
    private static final double CAMERA_HEIGHT = 10.0;
    private static final double MOUNT_ANGLE = 20.0;

    private final Limelight3A limelight = mock(Limelight3A.class);
    private final LLResult result = mock(LLResult.class);
    private final LimelightCamera camera =
            new LimelightCamera(limelight, CAMERA_HEIGHT, MOUNT_ANGLE);

    /** Makes the fake camera report a valid result that sees a target at these angles. */
    private void seesTarget(double tx, double ty) {
        when(limelight.getLatestResult()).thenReturn(result);
        when(result.isValid()).thenReturn(true);
        when(result.getTx()).thenReturn(tx);
        when(result.getTy()).thenReturn(ty);
        camera.update();
    }

    /** A fake AprilTag with this ID. */
    private LLResultTypes.FiducialResult tag(int id, double xDegrees, Position positionInCameraSpace) {
        LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetXDegrees()).thenReturn(xDegrees);
        when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                positionInCameraSpace, new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)));
        return tag;
    }

    // ---- Seeing nothing ----

    @Test
    public void noTargetBeforeTheCameraHasSentAnything() {
        when(limelight.getLatestResult()).thenReturn(null);
        camera.update();

        assertFalse(camera.hasTarget());
        assertEquals(0.0, camera.getTargetXDegrees(), 1e-9);
        assertEquals(0.0, camera.getTargetYDegrees(), 1e-9);
        assertEquals(0.0, camera.getDistanceInches(30), 1e-9);
        assertTrue(camera.getVisibleTagIds().isEmpty());
    }

    @Test
    public void invalidResultMeansNoTarget() {
        when(limelight.getLatestResult()).thenReturn(result);
        when(result.isValid()).thenReturn(false);
        camera.update();

        assertFalse(camera.hasTarget());
    }

    // ---- Seeing a target ----

    @Test
    public void targetAnglesAreReported() {
        seesTarget(5.0, -3.0);
        when(result.getTa()).thenReturn(2.5);

        assertTrue(camera.hasTarget());
        assertEquals(5.0, camera.getTargetXDegrees(), 1e-9);
        assertEquals(-3.0, camera.getTargetYDegrees(), 1e-9);
        assertEquals(2.5, camera.getTargetAreaPercent(), 1e-9);
    }

    // ---- Distance from the camera angle ----

    @Test
    public void distanceUsesTheCameraTiltAndTargetHeight() {
        // Target dead level with the camera's aim (ty = 0), so the angle up is just the 20 degree tilt.
        // distance = (30 - 10) / tan(20 degrees)
        seesTarget(0.0, 0.0);
        assertEquals(20.0 / Math.tan(Math.toRadians(20)), camera.getDistanceInches(30), 1e-6);
    }

    @Test
    public void targetHigherInThePictureIsCloser() {
        seesTarget(0.0, 10.0);    // angle up = 20 + 10 = 30 degrees
        assertEquals(20.0 / Math.tan(Math.toRadians(30)), camera.getDistanceInches(30), 1e-6);
    }

    @Test
    public void targetBelowTheCameraWorksWhenSeenBelowLevel() {
        // Target 5 inches below the camera. It must appear 20 degrees below level: ty = -40 with a 20 degree tilt.
        seesTarget(0.0, -40.0);
        assertEquals(5.0 / Math.tan(Math.toRadians(20)), camera.getDistanceInches(5), 1e-6);
    }

    @Test
    public void nonsenseDistanceIsReportedAsZero() {
        // A target ABOVE the camera cannot appear below level.
        seesTarget(0.0, -40.0);
        assertEquals(0.0, camera.getDistanceInches(30), 1e-9);
    }

    // ---- AprilTags ----

    @Test
    public void visibleTagIdsListsEveryTagSeen() {
        seesTarget(0, 0);
        LLResultTypes.FiducialResult tag20 = tag(20, 0, new Position(DistanceUnit.INCH, 0, 0, 0, 0));
        LLResultTypes.FiducialResult tag24 = tag(24, 0, new Position(DistanceUnit.INCH, 0, 0, 0, 0));
        when(result.getFiducialResults()).thenReturn(Arrays.asList(tag20, tag24));

        assertEquals(Arrays.asList(20, 24), camera.getVisibleTagIds());
        assertTrue(camera.canSeeTag(24));
        assertFalse(camera.canSeeTag(21));
    }

    @Test
    public void tagXDegreesIsForTheRequestedTag() {
        seesTarget(0, 0);
        LLResultTypes.FiducialResult tag20 = tag(20, 4.0, new Position(DistanceUnit.INCH, 0, 0, 0, 0));
        LLResultTypes.FiducialResult tag24 = tag(24, -7.0, new Position(DistanceUnit.INCH, 0, 0, 0, 0));
        when(result.getFiducialResults()).thenReturn(Arrays.asList(tag20, tag24));

        assertEquals(-7.0, camera.getTagXDegrees(24), 1e-9);
        assertEquals(4.0, camera.getTagXDegrees(20), 1e-9);
        assertEquals(0.0, camera.getTagXDegrees(99), 1e-9);   // not visible
    }

    @Test
    public void tagDistanceIsTheStraightLineToTheTag() {
        seesTarget(0, 0);
        // 3 inches right, 4 inches down, 0 forward: a 3-4-5 triangle
        LLResultTypes.FiducialResult tag24 = tag(24, 0, new Position(DistanceUnit.INCH, 3, 4, 0, 0));
        when(result.getFiducialResults()).thenReturn(Collections.singletonList(tag24));

        assertEquals(5.0, camera.getTagDistanceInches(24), 1e-6);
        assertEquals(0.0, camera.getTagDistanceInches(99), 1e-9);
    }

    // ---- Field position ----

    @Test
    public void fieldPositionIsConvertedToInches() {
        seesTarget(0, 0);
        when(result.getBotpose()).thenReturn(new Pose3D(
                new Position(DistanceUnit.METER, 1.0, -0.5, 0, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)));

        assertTrue(camera.hasFieldPosition());
        assertEquals(39.3700787, camera.getFieldXInches(), 1e-5);
        assertEquals(-19.6850394, camera.getFieldYInches(), 1e-5);
    }

    @Test
    public void noFieldPositionWithoutATarget() {
        when(limelight.getLatestResult()).thenReturn(null);
        camera.update();

        assertFalse(camera.hasFieldPosition());
        assertEquals(0.0, camera.getFieldXInches(), 1e-9);
    }

    // ---- Control ----

    @Test
    public void setPipelineSwitchesThePipeline() {
        camera.setPipeline(3);
        verify(limelight).pipelineSwitch(3);
    }

    @Test
    public void stopTurnsTheCameraOff() {
        camera.stop();
        verify(limelight).stop();
    }
}
