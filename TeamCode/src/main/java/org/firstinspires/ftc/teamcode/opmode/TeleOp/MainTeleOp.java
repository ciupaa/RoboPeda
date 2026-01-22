package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

/**
 * FILE: MainTeleOp.java
 * PURPOSE: The primary driver control program.
 * CLEANED: No Burp, No CRServos.
 * LOGIC: Blocker opens IMMEDIATELY on bumper press.
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    private Robot r;

    // --- TIMERS ---
    private ElapsedTime servoTimer = new ElapsedTime();
    private ElapsedTime blockerTimer = new ElapsedTime();

    private double calculatedWaitTime = 0;
    private boolean isShooting = false;

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;

    // BUTTON MEMORY
    private boolean lastUp = false, lastDown = false;
    private boolean lastA = false, lastB = false;

    // PRESETS
    private double highPreset = 0.65;
    private double lowPreset = 0.7;
    private double idlePreset = 1.0;

    // CONSTANTS
    private final double SERVO_SPEED_FACTOR = 650.0;
    private final double VELOCITY_TOLERANCE = 50;

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        r.shooter.block(); // Start Closed (0.85)
    }

    @Override
    public void loop() {
        r.periodic();

        // 1. DRIVING
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double finalRx = -gamepad1.right_stick_x + (gamepad1.left_trigger * 0.3) - (gamepad1.right_trigger * 0.3);

        r.drive.driveRobotCentric(x, y, finalRx);

        if (gamepad1.options) {
            r.drive.resetHeading();
            gamepad1.rumble(500);
        }

        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000);
            endgameRumbled = true;
        }

        // --- ADJUSTMENTS ---
        if (gamepad1.dpad_up && !lastUp) {
            if (gamepad1.right_bumper) highPreset += 0.01;
            else if (gamepad1.left_bumper) lowPreset += 0.01;
            else idlePreset += 0.01;
        }
        if (gamepad1.dpad_down && !lastDown) {
            if (gamepad1.right_bumper) highPreset -= 0.01;
            else if (gamepad1.left_bumper) lowPreset -= 0.01;
            else idlePreset -= 0.01;
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // Blocker Tuning (A/B)
        if (gamepad1.a && !lastA) {
            if (gamepad1.right_bumper || gamepad1.left_bumper) r.shooter.unblockPos += 0.01;
            else r.shooter.blockPos += 0.01;
        }
        if (gamepad1.b && !lastB) {
            if (gamepad1.right_bumper || gamepad1.left_bumper) r.shooter.unblockPos -= 0.01;
            else r.shooter.blockPos -= 0.01;
        }
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        // Safety Clamps
        highPreset = Math.max(0, Math.min(1, highPreset));
        lowPreset = Math.max(0, Math.min(1, lowPreset));
        r.shooter.unblockPos = Math.max(0, Math.min(1, r.shooter.unblockPos));
        r.shooter.blockPos = Math.max(0, Math.min(1, r.shooter.blockPos));


        // =========================================================
        //                 MAIN LOGIC
        // =========================================================

        // 1. SHOOTING (Bumpers)
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            jamRumbled = false;

            double targetAngle = gamepad1.right_bumper ? highPreset : lowPreset;
            double targetVel = gamepad1.right_bumper ? 1550 : 1200;

            if (!isShooting) {
                // --- START SEQUENCE ---
                double currentPos = r.shooter.getAngle();
                double distance = Math.abs(targetAngle - currentPos);
                calculatedWaitTime = distance * SERVO_SPEED_FACTOR;

                // A. Set Angle
                r.shooter.setAngle(targetAngle);

                // B. Open Blocker IMMEDIATELY
                r.shooter.unblock(); // Moves to 0.24

                // C. Reset Timers
                servoTimer.reset();
                blockerTimer.reset();

                isShooting = true;
            } else {
                // Hold states
                r.shooter.setAngle(targetAngle);
                r.shooter.unblock();
            }

            // D. Spin Flywheel
            if (gamepad1.right_bumper) r.shooter.spinHigh();
            else r.shooter.spinLow();

            // E. FEED LOGIC (Wait for everything to be ready)
            boolean angleReady = servoTimer.milliseconds() >= calculatedWaitTime;
            // Small delay (300ms) to ensure blocker is physically out of the way
            boolean blockerDelayDone = blockerTimer.milliseconds() >= 300;
            boolean velocityReady = r.shooter.getVelocity() >= (targetVel - VELOCITY_TOLERANCE);

            if (angleReady && blockerDelayDone && velocityReady) {
                r.intake.intake(); // SHOOT!
            } else {
                r.intake.stop();   // Wait
            }
        }

        // 2. INTAKE (X)
        else if (gamepad1.x) {
            isShooting = false;

            r.shooter.setAngle(idlePreset);
            r.shooter.stop();  // Coast
            r.shooter.block(); // Close (0.85)

            r.intake.intake();

            double currentAmps = r.intake.getCurrentDraw();
            if (currentAmps > Intake.JAM_THRESHOLD) {
                if (!jamRumbled) {
                    gamepad1.rumble(500);
                    jamRumbled = true;
                }
                telemetry.addData("WARNING", "INTAKE OVERLOAD! (%.1f A)", currentAmps);
            } else {
                jamRumbled = false;
            }
        }

        // 3. OUTTAKE (Y)
        else if (gamepad1.y) {
            isShooting = false;
            jamRumbled = false;
            r.shooter.setAngle(idlePreset);
            r.intake.outtakeSlow();
            r.shooter.stop();
            r.shooter.block();
        }

        // 4. IDLE / RELEASE
        else {
            isShooting = false;
            jamRumbled = false;

            r.shooter.setAngle(idlePreset);
            r.shooter.stop(); // Coast
            r.intake.stop();
            r.shooter.block(); // Close (0.85)

            if (gamepad1.dpad_left) r.shooter.reverse();
        }

        // Telemetry
        telemetry.addData("State", isShooting ? "SHOOTING" : "IDLE");
        telemetry.addData("Velocity", "%.0f", r.shooter.getVelocity());

        telemetry.addLine("--- Blocker Status ---");
        telemetry.addData("Target Pos", isShooting ? r.shooter.unblockPos : r.shooter.blockPos);

        telemetry.addData("High Preset", "%.2f", highPreset);
        telemetry.addData("Low Preset", "%.2f", lowPreset);

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is si ei smecheri");

        telemetry.update();
    }
}