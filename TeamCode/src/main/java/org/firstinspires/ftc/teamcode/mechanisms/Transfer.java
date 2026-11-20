package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private final DcMotor motor;
    private final double feedPower;
    private final double reversePower;

    /**
     * @param motor        the transfer motor, already set up by RobotHardware
     * @param feedPower    power for moving elements toward the flywheel, 0.0 to 1.0
     * @param reversePower power for running backward, 0.0 to 1.0
     */
    public Transfer(DcMotor motor, double feedPower, double reversePower) {
        this.motor = motor;
        this.feedPower = feedPower;
        this.reversePower = reversePower;
    }

    /** Moves game elements toward the flywheel. */
    public void feed() {
        motor.setPower(feedPower);
    }

    /** Runs backward. Useful for clearing a jam. */
    public void reverse() {
        motor.setPower(-reversePower);
    }

    /** Stops the transfer. */
    public void stop() {
        motor.setPower(0.0);
    }
}
