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

        r.drive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        if (gamepad1.options) r.drive.resetHeading();

        // Shared Logic with Main TeleOp
        if (gamepad1.right_bumper) {
            r.shooter.spinHigh();
            r.shooter.setAngle(1.0);
        }
        else if (gamepad1.left_bumper) {
            r.shooter.spinLow();
            r.shooter.setAngle(0.91);
            r.intake.outtakeSuperSlow();
        }
        else if (gamepad1.x) {
            r.intake.intake();
            r.shooter.reverse();
        }
        else if (gamepad1.y) {
            r.intake.outtakeSlow();
        }
        else if (gamepad1.dpad_left) {
            r.shooter.reverse();
        }
        else {
            r.shooter.stop();
            r.intake.stop();
        }

        // Small increments (0.01) for fine-tuning
        if (gamepad1.dpad_up && !lastUp) currentAngle += 0.01;
        if (gamepad1.dpad_down && !lastDown) currentAngle -= 0.01;
        lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;

        if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
            currentAngle = Math.max(0, Math.min(1, currentAngle));
            r.shooter.setAngle(currentAngle);
        }

        telemetry.addData("Live Tuning Angle", currentAngle);
        telemetry.update();
    }
}