package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * A flywheel is a heavy wheel that spins fast to launch game pieces.
 *
 * Instead of setting a motor POWER (which gets weaker as the battery drains),
 * we set a target SPEED in RPM (revolutions per minute) and the motor
 * controller adjusts the power to hold that speed.
 *
 * Typical use:
 *     flywheel.setTargetRpm(3000);          // start spinning up
 *     if (flywheel.isAtSpeed()) { ... }     // ready to launch!
 *     flywheel.stop();                      // spin down
 */
public class Flywheel {
    private final DcMotorEx motor;
    private final double ticksPerRevolution;
    private final double toleranceRpm;

    private double targetRpm = 0.0;

    /**
     * @param motor              the flywheel motor, already set up by RobotHardware
     * @param ticksPerRevolution encoder ticks for one turn of the flywheel (see the motor's spec sheet)
     * @param toleranceRpm       how close to the target counts as "at speed"
     */
    public Flywheel(DcMotorEx motor, double ticksPerRevolution, double toleranceRpm) {
        this.motor = motor;
        this.ticksPerRevolution = ticksPerRevolution;
        this.toleranceRpm = toleranceRpm;
    }

    /**
     * Sets the PIDF numbers the motor uses to hold its target speed.
     * Call this from RobotHardware, or from a tuning op mode to try values live.
     * Does nothing to the target; the new numbers apply right away.
     */
    public void setPidf(double p, double i, double d, double f) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }

    /** The PIDF numbers the motor is using right now (handy to show in telemetry while tuning). */
    public PIDFCoefficients getPidf() {
        return motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Spins the flywheel up to this speed and holds it. */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        // The motor wants ticks per SECOND, so convert: RPM / 60 = revs per second.
        motor.setVelocity(rpm / 60.0 * ticksPerRevolution);
    }

    /** Lets the flywheel spin down. */
    public void stop() {
        setTargetRpm(0.0);
    }

    /** The speed we are trying to reach, in RPM. */
    public double getTargetRpm() {
        return targetRpm;
    }

    /** How fast the flywheel is actually spinning right now, in RPM. */
    public double getRpm() {
        return motor.getVelocity() / ticksPerRevolution * 60.0;
    }

    /** True once the flywheel is spinning close enough to the target to launch. */
    public boolean isAtSpeed() {
        return targetRpm > 0 && Math.abs(getRpm() - targetRpm) <= toleranceRpm;
    }
}
