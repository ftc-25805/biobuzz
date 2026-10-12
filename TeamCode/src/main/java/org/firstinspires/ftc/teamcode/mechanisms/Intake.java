package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The intake is a motor that pulls game elements into the robot.
 *
 * Typical use:
 *     intake.collect();   // pull game elements in
 *     intake.eject();     // push them back out (for a jam)
 *     intake.stop();
 */
public class Intake {
    private final DcMotor motor;
    private final double collectPower;
    private final double ejectPower;

    /**
     * @param motor        the intake motor, already set up by RobotHardware
     * @param collectPower power for pulling elements in, 0.0 to 1.0
     * @param ejectPower   power for pushing elements out, 0.0 to 1.0
     */
    public Intake(DcMotor motor, double collectPower, double ejectPower) {
        this.motor = motor;
        this.collectPower = collectPower;
        this.ejectPower = ejectPower;
    }

    /** Pulls game elements in. */
    public void collect() {
        motor.setPower(collectPower);
    }

    /** Pushes game elements back out. Useful for clearing a jam. */
    public void eject() {
        motor.setPower(-ejectPower);
    }

    /** Stops the intake. */
    public void stop() {
        motor.setPower(0.0);
    }
}
