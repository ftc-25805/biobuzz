package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Odometry tells us WHERE the robot is on the field. The goBILDA Pinpoint
 * watches two wheels that roll freely on the ground (the "odometry pods") and
 * uses its own gyro to work out the robot's position and heading.
 *
 * Position is measured from wherever the robot was when we last reset it:
 *   x = how far FORWARD, y = how far LEFT (inches), heading = turn angle (radians).
 *
 * IMPORTANT: call {@link #update()} once every loop, before reading the position.
 */
public class PinpointOdometry {
    private final GoBildaPinpointDriver pinpoint;

    /** @param pinpoint the Pinpoint, already set up by RobotHardware */
    public PinpointOdometry(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    /** Asks the Pinpoint for fresh numbers. Call once per loop. */
    public void update() {
        pinpoint.update();
    }

    /** How far forward of the starting spot the robot is, in inches. */
    public double getXInches() {
        return pinpoint.getPosition().getX(DistanceUnit.INCH);
    }

    /** How far left of the starting spot the robot is, in inches. */
    public double getYInches() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    /** The direction the robot is facing in radians (0 = the way it faced at the last reset). */
    public double getHeadingRadians() {
        return pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }

    /** The direction the robot is facing in degrees, handy for telemetry. */
    public double getHeadingDegrees() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    /** Makes the robot's current spot and direction count as (0, 0) facing 0. */
    public void resetPosition() {
        setPosition(0, 0, 0);
    }

    /** Tells the Pinpoint where the robot really is, e.g. its starting spot in autonomous. */
    public void setPosition(double xInches, double yInches, double headingRadians) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, xInches, yInches, AngleUnit.RADIANS, headingRadians));
    }
}
