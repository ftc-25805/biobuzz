package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Finds the flywheel speed (in RPM) that makes a good shot.
 *
 * Idea from Coach Pratt's video "This trick makes your flywheel 30x faster [PIDF Tuning]".
 *
 * How to use:
 *   1) Tune the PIDF first with the "Flywheel PIDF Tuner", so the flywheel holds its speed.
 *   2) Run this op mode and raise the target speed until shots score from where you stand.
 *   3) Write down that RPM for each shooting distance.
 *
 * Controls:
 *   D-pad up / down    raise / lower the target speed by one step
 *   B                  change the step size
 *
 * It uses the PIDF numbers set in RobotHardware.
 */
@TeleOp(name = "Flywheel Velocity Tuner", group = "Utils")
public class FlywheelVelocityTuner extends OpMode {

    private RobotHardware robot;

    private double targetRpm = 0.0;

    // How much each D-pad press changes the target, in RPM.
    private final double[] stepSizes = {100.0, 50.0, 10.0};
    private int stepIndex = 1;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetRpm += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetRpm = Math.max(0.0, targetRpm - stepSizes[stepIndex]);
        }

        robot.flywheel.setTargetRpm(targetRpm);

        double currentRpm = robot.flywheel.getRpm();
        PIDFCoefficients pidf = robot.flywheel.getPidf();

        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("Current RPM", "%.0f", currentRpm);
        telemetry.addData("Error", "%.0f", targetRpm - currentRpm);
        telemetry.addData("At speed", robot.flywheel.isAtSpeed());
        telemetry.addData("Step Size", "%.0f RPM (B Button)", stepSizes[stepIndex]);
        telemetry.addData("PIDF in use", "P=%.4f F=%.4f", pidf.p, pidf.f);
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
