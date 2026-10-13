package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

import java.util.HashMap;
import java.util.Map;

/**
 * Tests that RobotHardware sets up each piece of hardware the way we expect:
 * the right device names, motor directions and modes.
 *
 * The hardware map is a fake. Asking it for a device by name hands back a fake
 * device, and the test remembers which fake went with which name.
 */
public class RobotHardwareTest {
    private final Map<String, Object> devices = new HashMap<>();
    private RobotHardware robot;

    @Before
    public void setUp() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        when(hardwareMap.get(any(Class.class), anyString())).thenAnswer(invocation -> {
            Class<?> type = invocation.getArgument(0);
            String name = invocation.getArgument(1);
            Object device = mock(type);
            devices.put(name, device);
            return device;
        });

        robot = new RobotHardware(hardwareMap);
    }

    @SuppressWarnings("unchecked")
    private <T> T device(String name) {
        Object device = devices.get(name);
        assertNotNull("RobotHardware never asked for a device named " + name, device);
        return (T) device;
    }

    @Test
    public void createsEveryMechanism() {
        assertNotNull(robot.drive);
        assertNotNull(robot.imu);
        assertNotNull(robot.odometry);
        assertNotNull(robot.limelight);
        assertNotNull(robot.flywheel);
        assertNotNull(robot.intake);
        assertNotNull(robot.transfer);
    }

    @Test
    public void usesTheNamesFromTheDriverStationConfiguration() {
        // If someone renames a device in RobotHardware, the configuration on the
        // Driver Station must change too. This test makes that deliberate.
        assertNotNull(devices.get("front_left_drive"));
        assertNotNull(devices.get("front_right_drive"));
        assertNotNull(devices.get("back_left_drive"));
        assertNotNull(devices.get("back_right_drive"));
        assertNotNull(devices.get("imu"));
        assertNotNull(devices.get("pinpoint"));
        assertNotNull(devices.get("limelight"));
        assertNotNull(devices.get("flywheel"));
        assertNotNull(devices.get("intake"));
        assertNotNull(devices.get("transfer"));
    }

    @Test
    public void leftDriveMotorsAreReversedSoForwardIsForwardOnEveryWheel() {
        DcMotor frontLeft = device(RobotHardware.FRONT_LEFT_NAME);
        DcMotor backLeft = device(RobotHardware.BACK_LEFT_NAME);
        DcMotor frontRight = device(RobotHardware.FRONT_RIGHT_NAME);
        DcMotor backRight = device(RobotHardware.BACK_RIGHT_NAME);

        verify(frontLeft).setDirection(DcMotor.Direction.REVERSE);
        verify(backLeft).setDirection(DcMotor.Direction.REVERSE);
        verify(frontRight, never()).setDirection(DcMotor.Direction.REVERSE);
        verify(backRight, never()).setDirection(DcMotor.Direction.REVERSE);
    }

    @Test
    public void driveMotorsUseEncoders() {
        for (String name : new String[]{RobotHardware.FRONT_LEFT_NAME, RobotHardware.FRONT_RIGHT_NAME,
                RobotHardware.BACK_LEFT_NAME, RobotHardware.BACK_RIGHT_NAME}) {
            DcMotor motor = device(name);
            verify(motor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Test
    public void imuIsInitialized() {
        IMU imu = device(RobotHardware.IMU_NAME);
        verify(imu).initialize(any(IMU.Parameters.class));
    }

    @Test
    public void pinpointIsConfiguredAndReset() {
        GoBildaPinpointDriver pinpoint = device(RobotHardware.PINPOINT_NAME);
        verify(pinpoint).setEncoderResolution(RobotHardware.PINPOINT_POD_TYPE);
        verify(pinpoint).setEncoderDirections(
                RobotHardware.PINPOINT_X_DIRECTION, RobotHardware.PINPOINT_Y_DIRECTION);
        verify(pinpoint).resetPosAndIMU();
    }

    @Test
    public void limelightStartsOnTheStartPipeline() {
        Limelight3A limelight = device(RobotHardware.LIMELIGHT_NAME);
        verify(limelight).pipelineSwitch(RobotHardware.LIMELIGHT_START_PIPELINE);
        verify(limelight).start();
    }

    @Test
    public void flywheelUsesVelocityControlAndCoastsToAStop() {
        DcMotorEx flywheel = device(RobotHardware.FLYWHEEL_NAME);
        verify(flywheel).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verify(flywheel).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Test
    public void flywheelGetsTheConfiguredPidfNumbers() {
        DcMotorEx flywheel = device(RobotHardware.FLYWHEEL_NAME);
        ArgumentCaptor<PIDFCoefficients> captor = ArgumentCaptor.forClass(PIDFCoefficients.class);
        verify(flywheel).setPIDFCoefficients(
                eq(DcMotor.RunMode.RUN_USING_ENCODER), captor.capture());

        assertEquals(RobotHardware.FLYWHEEL_P, captor.getValue().p, 1e-9);
        assertEquals(RobotHardware.FLYWHEEL_I, captor.getValue().i, 1e-9);
        assertEquals(RobotHardware.FLYWHEEL_D, captor.getValue().d, 1e-9);
        assertEquals(RobotHardware.FLYWHEEL_F, captor.getValue().f, 1e-9);
    }

    @Test
    public void flywheelFeedforwardIsPositive() {
        // A bad ticks-per-rev or max RPM would make F zero, infinite, or negative.
        assertTrue(RobotHardware.FLYWHEEL_F > 0 && Double.isFinite(RobotHardware.FLYWHEEL_F));
    }

    @Test
    public void countsPerInchIsPositiveAndFinite() {
        assertTrue(RobotHardware.DRIVE_COUNTS_PER_INCH > 0
                && Double.isFinite(RobotHardware.DRIVE_COUNTS_PER_INCH));
    }

    @Test
    public void stopStopsTheMovingMechanisms() {
        DcMotor frontLeft = device(RobotHardware.FRONT_LEFT_NAME);
        DcMotor intake = device(RobotHardware.INTAKE_NAME);
        CRServo transfer = device(RobotHardware.TRANSFER_NAME);
        DcMotorEx flywheel = device(RobotHardware.FLYWHEEL_NAME);
        Limelight3A limelight = device(RobotHardware.LIMELIGHT_NAME);

        robot.stop();

        verify(frontLeft).setPower(0.0);
        verify(intake).setPower(0.0);
        verify(transfer).setPower(0.0);
        verify(flywheel).setVelocity(0.0);
        verify(limelight).stop();
    }
}
