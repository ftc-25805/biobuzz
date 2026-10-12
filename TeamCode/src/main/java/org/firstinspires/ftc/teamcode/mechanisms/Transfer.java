package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * The transfer moves a game element from the intake to the flywheel.
 * It is a continuous-rotation servo: it spins like a motor instead of
 * moving to a position.
 *
 *     transfer.feed();      // push game elements toward the flywheel
 *     transfer.reverse();   // run backward (to clear a jam)
 *     transfer.stop();
 */
public class Transfer {
    private final CRServo servo;
    private final double feedPower;
    private final double reversePower;

    /**
     * @param servo        the transfer servo, already set up by RobotHardware
     * @param feedPower    power for moving elements toward the flywheel, 0.0 to 1.0
     * @param reversePower power for running backward, 0.0 to 1.0
     */
    public Transfer(CRServo servo, double feedPower, double reversePower) {
        this.servo = servo;
        this.feedPower = feedPower;
        this.reversePower = reversePower;
    }

    /** Moves game elements toward the flywheel. */
    public void feed() {
        servo.setPower(feedPower);
    }

    /** Runs backward. Useful for clearing a jam. */
    public void reverse() {
        servo.setPower(-reversePower);
    }

    /** Stops the transfer. */
    public void stop() {
        servo.setPower(0.0);
    }
}
