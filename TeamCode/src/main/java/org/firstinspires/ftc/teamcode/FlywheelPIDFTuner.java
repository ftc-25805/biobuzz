package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tunes the flywheel's PIDF numbers live with the gamepad, so you don't have to
 * redeploy the code after every guess.
 *
 * Idea from Coach Pratt's video "This trick makes your flywheel 30x faster [PIDF Tuning]".
 * Only P and F are tuned. I and D stay 0.
 *
 * How to tune:
 *   1) Start with P = 0 and raise F until the flywheel reaches about the target speed.
 *      F does most of the work, so it should get you close.
 *   2) Then raise P until it gets to speed quickly and holds it, without wobbling.
 *   3) Press Y to switch between the long and short targets and check both.
 *   4) Copy the final P and F into FLYWHEEL_P and FLYWHEEL_F in RobotHardware.
 *
 * Controls:
 *   Y          switch between the long and short target speeds
 *   B          change the step size (how much each button press changes a number)
 *   D-pad up/down      raise / lower P
 *   D-pad right/left   raise / lower F
 */
@TeleOp(name = "Flywheel PIDF Tuner", group = "Utils")
public class FlywheelPIDFTuner extends OpMode {

    private RobotHardware robot;

    // Target speeds in RPM. TODO: set these to the speeds your shots actually need.
    private static final double LONG_RPM = 5000;
    private static final double SHORT_RPM = 3600;
    private double targetRpm = LONG_RPM;

    // Start from the values in RobotHardware, but with I and D off.
    private double F = RobotHardware.FLYWHEEL_F;
    private double P = RobotHardware.FLYWHEEL_P;

    private final double[] stepSizes = {10.0, 1.0, 0.01, 0.001, 0.0001};
    private int stepIndex = 1;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.flywheel.setPidf(P, 0.0, 0.0, F);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        // Read the buttons.
        if (gamepad1.yWasPressed()) {
            targetRpm = (targetRpm == LONG_RPM) ? SHORT_RPM : LONG_RPM;
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        boolean changed = false;
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
            changed = true;
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
            changed = true;
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
            changed = true;
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
            changed = true;
        }

        // Only send new numbers to the motor when they actually changed.
        if (changed) {
            robot.flywheel.setPidf(P, 0.0, 0.0, F);
        }

        robot.flywheel.setTargetRpm(targetRpm);

        double currentRpm = robot.flywheel.getRpm();

        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("Current RPM", "%.0f", currentRpm);
        telemetry.addData("Error", "%.0f", targetRpm - currentRpm);
        telemetry.addLine("----------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
