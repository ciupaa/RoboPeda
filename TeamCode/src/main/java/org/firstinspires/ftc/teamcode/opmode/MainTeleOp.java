package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake; // Import for JAM_THRESHOLD

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {
    private Robot r;

    // --- LOGIC VARIABLES ---
    private ElapsedTime servoTimer = new ElapsedTime();
    private double calculatedWaitTime = 0;
    private boolean isShooting = false;

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;

    // MICRO-ADJUSTMENT VARIABLES
    private boolean lastUp = false, lastDown = false;
    private double highPreset = 0.9;
    private double lowPreset = 0.7;
    private double idlePreset = 1.0;

    // CONSTANTS
    private final double SERVO_SPEED_FACTOR = 600.0;
    private final double VELOCITY_TOLERANCE = 50;

    @Override
    public void init() { r = new Robot(hardwareMap, Alliance.BLUE); }

    @Override
    public void loop() {
        r.periodic();

        // 1. DRIVING
        r.drive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

        // IMU RESET + RUMBLE
        if (gamepad1.options) {
            r.drive.resetHeading();
            gamepad1.rumble(500);
        }

        // 2. ENDGAME RUMBLE (2 Minutes)
        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000);
            endgameRumbled = true;
        }

        // --- MICRO-ADJUSTMENTS ---
        if (gamepad1.dpad_up && !lastUp) {
            if (gamepad1.right_bumper) highPreset += 0.1;
            else if (gamepad1.left_bumper) lowPreset += 0.1;
            else idlePreset += 0.1;
        }
        if (gamepad1.dpad_down && !lastDown) {
            if (gamepad1.right_bumper) highPreset -= 0.1;
            else if (gamepad1.left_bumper) lowPreset -= 0.1;
            else idlePreset -= 0.1;
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        highPreset = Math.max(0, Math.min(1, highPreset));
        lowPreset = Math.max(0, Math.min(1, lowPreset));
        idlePreset = Math.max(0, Math.min(1, idlePreset));


        // 3. SHOOTING LOGIC (Bumpers)
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            jamRumbled = false;

            double targetAngle = gamepad1.right_bumper ? highPreset : lowPreset;
            double targetVel = gamepad1.right_bumper ? 1550 : 1200;

            if (!isShooting) {
                double currentPos = r.shooter.getAngle();
                double distance = Math.abs(targetAngle - currentPos);
                calculatedWaitTime = distance * SERVO_SPEED_FACTOR;
                r.shooter.setAngle(targetAngle);
                servoTimer.reset();
                isShooting = true;
            } else {
                r.shooter.setAngle(targetAngle);
            }

            if (servoTimer.milliseconds() >= calculatedWaitTime) {
                if (gamepad1.right_bumper) r.shooter.spinHigh();
                else r.shooter.spinLow();

                double currentVel = r.shooter.getVelocity();
                if (currentVel >= (targetVel - VELOCITY_TOLERANCE)) {
                    r.intake.intake();
                } else {
                    r.intake.stop();
                }
            } else {
                r.shooter.stop();
                r.intake.stop();
            }
        }
        // 4. INTAKE MODE (X) - WITH JAM PROTECTION (3.5A Limit)
        else if (gamepad1.x) {
            isShooting = false;
            r.shooter.setAngle(idlePreset);
            r.shooter.reverse();

            // --- CURRENT SENSING LOGIC ---
            double currentAmps = r.intake.getCurrentDraw();

            if (currentAmps > Intake.JAM_THRESHOLD) {
                // JAM DETECTED (Over 3.5A)!
                r.intake.stop();

                if (!jamRumbled) {
                    gamepad1.rumble(300);
                    jamRumbled = true;
                }
                telemetry.addData("WARNING", "INTAKE JAMMED! (%.1f A)", currentAmps);
            } else {
                // SAFE TO RUN
                r.intake.intake();
                jamRumbled = false;
            }
        }
        // 5. IDLE / STANDARD POSITION
        else {
            isShooting = false;
            jamRumbled = false;

            r.shooter.setAngle(idlePreset);
            r.shooter.stop();
            r.intake.stop();

            if (gamepad1.dpad_left) r.shooter.reverse();
        }

        telemetry.addData("State", isShooting ? "SHOOTING" : "IDLE/STANDARD");
        telemetry.addData("Intake Load", "%.1f A", r.intake.getCurrentDraw());

        telemetry.addData("High Preset", "%.2f", highPreset);
        telemetry.addData("Low Preset", "%.2f", lowPreset);
        telemetry.addData("Idle Preset", "%.2f", idlePreset);

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is niste slabi");
        telemetry.update();
    }
}