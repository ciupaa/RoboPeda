package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@TeleOp(name = "Shooter Test & Tune", group = "Tuning")
public class ShooterTuningTeleOp extends OpMode {
    private Robot r;
    private double currentAngle = 0.5;
    private boolean lastUp = false, lastDown = false;

    @Override
    public void init() { r = new Robot(hardwareMap, Alliance.BLUE); }

    @Override
    public void loop() {
        r.periodic();

        // Driving
        r.drive.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
        if (gamepad1.options) r.drive.resetHeading();

        // 2 Speeds Testing
        if (gamepad1.right_bumper) {
            r.shooter.setTargetVelocity(1450); // High Speed Test
            r.shooter.spinUp();
        } else if (gamepad1.left_bumper) {
            r.shooter.setTargetVelocity(1100); // Low Speed Test
            r.shooter.spinUp();
        } else {
            r.shooter.stop();
        }

        // Angle Tuning
        if (gamepad1.dpad_up && !lastUp) currentAngle += 0.01;
        if (gamepad1.dpad_down && !lastDown) currentAngle -= 0.01;
        lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;

        currentAngle = Math.max(0, Math.min(1, currentAngle));
        r.shooter.setAngle(currentAngle);

        // Intake Test
        if (gamepad1.x) r.intake.intake();
        else if (gamepad1.y) r.intake.outtake();
        else r.intake.stop();

        telemetry.addData("CURRENT ANGLE", "%.2f", currentAngle);
        telemetry.addData("Shooter Velocity", r.shooter.getVelocity());
        telemetry.update();
    }
}