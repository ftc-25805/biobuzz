package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.junit.Test;
import org.mockito.ArgumentCaptor;

public class FlywheelTest {
    private static final double TICKS_PER_REV = 28.0;
    private static final double TOLERANCE_RPM = 50.0;

    private final DcMotorEx motor = mock(DcMotorEx.class);
    private final Flywheel flywheel = new Flywheel(motor, TICKS_PER_REV, TOLERANCE_RPM);

    /** Pretend the motor is spinning at this many RPM. */
    private void spinningAt(double rpm) {
        when(motor.getVelocity()).thenReturn(rpm / 60.0 * TICKS_PER_REV);
    }

    @Test
    public void targetRpmIsSentToTheMotorAsTicksPerSecond() {
        flywheel.setTargetRpm(3000);
        // 3000 rev/min = 50 rev/sec, and 50 * 28 ticks = 1400 ticks/sec
        verify(motor).setVelocity(1400.0);
    }

    @Test
    public void getTargetRpmReturnsWhatWasSet() {
        flywheel.setTargetRpm(2500);
        assertEquals(2500, flywheel.getTargetRpm(), 1e-9);
    }

    @Test
    public void getRpmConvertsTicksPerSecondBackToRpm() {
        spinningAt(3000);
        assertEquals(3000, flywheel.getRpm(), 1e-9);
    }

    @Test
    public void stopSetsTargetToZero() {
        flywheel.setTargetRpm(3000);
        flywheel.stop();

        verify(motor).setVelocity(0.0);
        assertEquals(0, flywheel.getTargetRpm(), 1e-9);
    }

    @Test
    public void isAtSpeedWhenWithinTolerance() {
        flywheel.setTargetRpm(3000);

        spinningAt(3000);
        assertTrue(flywheel.isAtSpeed());
        spinningAt(2960);
        assertTrue(flywheel.isAtSpeed());
        spinningAt(3040);
        assertTrue(flywheel.isAtSpeed());
    }

    @Test
    public void isNotAtSpeedWhenOutsideTolerance() {
        flywheel.setTargetRpm(3000);

        spinningAt(2900);
        assertFalse(flywheel.isAtSpeed());
        spinningAt(3100);
        assertFalse(flywheel.isAtSpeed());
    }

    @Test
    public void isNeverAtSpeedWhenTargetIsZero() {
        spinningAt(0);
        flywheel.setTargetRpm(0);
        assertFalse(flywheel.isAtSpeed());
    }

    @Test
    public void setPidfSendsTheNumbersToTheMotor() {
        flywheel.setPidf(1.0, 2.0, 3.0, 4.0);

        ArgumentCaptor<PIDFCoefficients> captor = ArgumentCaptor.forClass(PIDFCoefficients.class);
        verify(motor).setPIDFCoefficients(
                org.mockito.ArgumentMatchers.eq(DcMotor.RunMode.RUN_USING_ENCODER),
                captor.capture());
        assertEquals(1.0, captor.getValue().p, 1e-9);
        assertEquals(2.0, captor.getValue().i, 1e-9);
        assertEquals(3.0, captor.getValue().d, 1e-9);
        assertEquals(4.0, captor.getValue().f, 1e-9);
    }
}
