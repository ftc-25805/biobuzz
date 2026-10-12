package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

/**
 * The Limelight is a smart camera that finds things for us, such as AprilTags
 * (the black-and-white square codes on the field), and tells us where they are.
 *
 * What the camera looks for is set by its "pipeline", which you set up in the
 * Limelight's web page. Each pipeline has a number.
 *
 * Angles: "tx" is how far the target is to the RIGHT of the camera's center
 * (negative = left), and "ty" is how far UP (negative = down). Both in degrees.
 * Turning until tx is 0 points the robot right at the target!
 *
 * IMPORTANT: call {@link #update()} once every loop, before reading anything.
 */
public class LimelightCamera {
    private final Limelight3A limelight;
    private final double cameraHeightInches;
    private final double cameraMountAngleDegrees;

    /** The newest answer from the camera. Can be null if the camera hasn't sent anything yet. */
    private LLResult result;

    /**
     * @param limelight                the Limelight, already set up and started by RobotHardware
     * @param cameraHeightInches        height of the camera lens above the floor
     * @param cameraMountAngleDegrees   how far the camera is tilted UP from level (0 = looking straight ahead)
     */
    public LimelightCamera(Limelight3A limelight,
                           double cameraHeightInches, double cameraMountAngleDegrees) {
        this.limelight = limelight;
        this.cameraHeightInches = cameraHeightInches;
        this.cameraMountAngleDegrees = cameraMountAngleDegrees;
    }

    /** Gets the newest answer from the camera. Call once per loop. */
    public void update() {
        result = limelight.getLatestResult();
    }

    /** Turns the camera off. RobotHardware calls this when the op mode ends. */
    public void stop() {
        limelight.stop();
    }

    /** Changes what the camera looks for. Pipeline numbers come from the Limelight web page. */
    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /** True if the camera can currently see something it is looking for. */
    public boolean hasTarget() {
        return result != null && result.isValid();
    }

    /** How far right of the camera's center the target is, in degrees (negative = left). */
    public double getTargetXDegrees() {
        return hasTarget() ? result.getTx() : 0.0;
    }

    /** How far above the camera's center the target is, in degrees (negative = below). */
    public double getTargetYDegrees() {
        return hasTarget() ? result.getTy() : 0.0;
    }

    /** How much of the camera's picture the target fills, in percent. Bigger means closer. */
    public double getTargetAreaPercent() {
        return hasTarget() ? result.getTa() : 0.0;
    }

    /** The ID numbers of every AprilTag the camera can see right now. */
    public List<Integer> getVisibleTagIds() {
        List<Integer> ids = new ArrayList<>();
        if (hasTarget()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                ids.add(tag.getFiducialId());
            }
        }
        return ids;
    }

    /** True if the AprilTag with this ID is in view. */
    public boolean canSeeTag(int tagId) {
        return getVisibleTagIds().contains(tagId);
    }

    /**
     * How far right of the camera's center a specific AprilTag is, in degrees.
     * Check {@link #canSeeTag(int)} first; returns 0 if the tag isn't visible.
     */
    public double getTagXDegrees(int tagId) {
        if (hasTarget()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == tagId) {
                    return tag.getTargetXDegrees();
                }
            }
        }
        return 0.0;
    }

    /**
     * How far away the target is, in inches, worked out from how high it appears
     * in the camera's picture. Works for any target (AprilTag, color blob, ...)
     * as long as you know how high the target is off the floor.
     *
     * The higher the target appears (bigger ty), the closer it is. This is just
     * trigonometry: we know the camera's height and tilt, the target's height,
     * and the angle up to the target.
     *
     * Returns 0 if there is no target or the answer doesn't make sense. If the distance
     * is always wrong, check the camera height and mount angle in RobotHardware.
     *
     * @param targetHeightInches height of the CENTER of the target above the floor
     */
    public double getDistanceInches(double targetHeightInches) {
        if (!hasTarget()) {
            return 0.0;
        }
        double angleToTarget = Math.toRadians(cameraMountAngleDegrees + result.getTy());
        double distance = (targetHeightInches - cameraHeightInches) / Math.tan(angleToTarget);
        return (Double.isFinite(distance) && distance > 0) ? distance : 0.0;
    }

    /**
     * How far away a specific AprilTag is, in inches, in a straight line from the camera.
     * AprilTags carry their own size, so the Limelight can measure this directly.
     * Check {@link #canSeeTag(int)} first; returns 0 if the tag isn't visible.
     */
    public double getTagDistanceInches(int tagId) {
        if (hasTarget()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == tagId) {
                    Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                    return Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                }
            }
        }
        return 0.0;
    }

    /**
     * True if the camera has worked out where the robot is on the field
     * (it needs to see AprilTags and be using an AprilTag pipeline).
     */
    public boolean hasFieldPosition() {
        if (!hasTarget()) {
            return false;
        }
        Pose3D pose = result.getBotpose();
        return pose != null;
    }

    /** Robot's X position on the field in inches (field center is 0), from AprilTags. */
    public double getFieldXInches() {
        return hasFieldPosition()
                ? result.getBotpose().getPosition().toUnit(DistanceUnit.INCH).x : 0.0;
    }

    /** Robot's Y position on the field in inches (field center is 0), from AprilTags. */
    public double getFieldYInches() {
        return hasFieldPosition()
                ? result.getBotpose().getPosition().toUnit(DistanceUnit.INCH).y : 0.0;
    }
}
