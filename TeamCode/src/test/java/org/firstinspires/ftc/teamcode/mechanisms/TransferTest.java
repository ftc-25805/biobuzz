package org.firstinspires.ftc.teamcode.mechanisms;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import com.qualcomm.robotcore.hardware.CRServo;

import org.junit.Test;

public class TransferTest {
    private final CRServo servo = mock(CRServo.class);
    private final Transfer transfer = new Transfer(servo, 1.0, 0.6);

    @Test
    public void feedRunsServoAtFeedPower() {
        transfer.feed();
        verify(servo).setPower(1.0);
    }

    @Test
    public void reverseRunsServoBackwardAtReversePower() {
        transfer.reverse();
        verify(servo).setPower(-0.6);
    }

    @Test
    public void stopSetsPowerToZero() {
        transfer.stop();
        verify(servo).setPower(0.0);
    }
}
