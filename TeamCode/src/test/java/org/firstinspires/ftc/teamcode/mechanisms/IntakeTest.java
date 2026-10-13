package org.firstinspires.ftc.teamcode.mechanisms;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.junit.Test;

public class IntakeTest {
    private final DcMotor motor = mock(DcMotor.class);
    private final Intake intake = new Intake(motor, 1.0, 0.6);

    @Test
    public void collectRunsMotorAtCollectPower() {
        intake.collect();
        verify(motor).setPower(1.0);
    }

    @Test
    public void ejectRunsMotorBackwardAtEjectPower() {
        intake.eject();
        verify(motor).setPower(-0.6);
    }

    @Test
    public void stopSetsPowerToZero() {
        intake.stop();
        verify(motor).setPower(0.0);
    }
}
