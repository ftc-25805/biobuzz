package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    /** The newest answer from the camera. Can be null if the camera hasn't sent anything yet. */
    private LLResult result;

    /** @param limelight the Limelight, already set up and started by RobotHardware */
    public LimelightCamera(Limelight3A limelight) {
        this.limelight = limelight;
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
