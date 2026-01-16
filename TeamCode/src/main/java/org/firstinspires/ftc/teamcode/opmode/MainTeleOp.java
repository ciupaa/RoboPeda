package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {
    private Robot r;

    // --- TIMERS & STATE ---
    private ElapsedTime servoTimer = new ElapsedTime();
    private ElapsedTime releaseTimer = new ElapsedTime(); // Timer for Release Sequence

    private double calculatedWaitTime = 0;
    private boolean isShooting = false;
    private boolean isReleasing = false; // Flag for the Sequence

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;

    // BUTTON STATE
    private boolean lastX = false;
    private boolean lastUp = false, lastDown = false;

    // PRESETS
    private double highPreset = 0.65;
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

        // =========================================================
        //                 1. DRIVING (Hybrid Control)
        // =========================================================
        double y = gamepad1.left_stick_y; // Forward
        double x = -gamepad1.left_stick_x;  // Strafe

        // A. STICK TURN (Fast)
        double stickTurn = -gamepad1.right_stick_x;

        // B. TRIGGER TURN (Slow - 30%)
        // Left Trigger turns LEFT (Positive), Right Trigger turns RIGHT (Negative)
        double triggerTurn = (gamepad1.left_trigger * 0.3) - (gamepad1.right_trigger * 0.3);

        // Combine inputs
        double finalRx = stickTurn + triggerTurn;

        r.drive.driveRobotCentric(x, y, finalRx);

        if (gamepad1.options) {
            r.drive.resetHeading();
            gamepad1.rumble(500);
        }

        if (getRuntime() >= 135 && !endgameRumbled) {
            gamepad1.rumble(4000);
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

        // =========================================================
        //                 MAIN LOGIC (Priority Chain)
        // =========================================================

        // 1. SHOOTING LOGIC (Highest Priority)
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            jamRumbled = false;
            isReleasing = false; // Cancel release sequence

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

                if (r.shooter.getVelocity() >= (targetVel - VELOCITY_TOLERANCE)) {
                    r.intake.intake();
                } else {
                    r.intake.stop();
                }
            } else {
                r.shooter.stop();
                r.intake.stop();
            }
        }

        // 2. INTAKE MODE (X)
        else if (gamepad1.x) {
            isShooting = false;
            isReleasing = false; // RESET SEQUENCE (As requested)

            r.shooter.setAngle(idlePreset);
            r.shooter.stop();

            double currentAmps = r.intake.getCurrentDraw();

            if (currentAmps > Intake.JAM_THRESHOLD) {
                // JAM (Over 6.0A) -> STOP ONLY
                r.intake.stop();
                if (!jamRumbled) {
                    gamepad1.rumble(500);
                    jamRumbled = true;
                }
                telemetry.addData("WARNING", "INTAKE OVERLOAD! (%.1f A)", currentAmps);
            } else {
                r.intake.intake();
                jamRumbled = false;
            }
        }

        // 3. 'Y' OUTTAKE
        else if (gamepad1.y) {
            isShooting = false;
            isReleasing = false; // Cancel sequence
            jamRumbled = false;
            r.shooter.setAngle(idlePreset);
            r.intake.outtakeSlow();
            r.shooter.stop();
        }

        // 4. RELEASE SEQUENCE (Burp -> Wait 1s -> Reverse)
        else if (isReleasing) {
            double t = releaseTimer.milliseconds();

            // A. Burp (0ms - 150ms)
            if (t < 150) {
                r.intake.outtakeSuperSlow();
                r.shooter.stop();
                telemetry.addData("Seq", "Burping...");
            }
            // B. Wait (150ms - 1150ms) -> Total 1 sec wait after burp
            else if (t < 1150) {
                r.intake.stop();
                r.shooter.stop();
                telemetry.addData("Seq", "Waiting 1s...");
            }
            // C. Reverse Launcher (1150ms - 1650ms) -> 0.5s Reverse
            else if (t < 1650) {
                r.intake.stop();
                r.shooter.reverse();
                telemetry.addData("Seq", "Reversing Launcher...");
            }
            // D. Done
            else {
                isReleasing = false;
                r.shooter.stop();
                r.intake.stop();
            }
        }

        // 5. IDLE / CHECK FOR RELEASE TRIGGER
        else {
            isShooting = false;
            jamRumbled = false;

            // CHECK FOR RELEASE: If we just let go of X...
            if (lastX) {
                isReleasing = true; // Start Sequence
                releaseTimer.reset();
            }

            r.shooter.setAngle(idlePreset);
            r.shooter.stop();
            r.intake.stop();

            if (gamepad1.dpad_left) r.shooter.reverse();
        }

        // UPDATE BUTTON STATE
        lastX = gamepad1.x;

        // TELEMETRY
        telemetry.addData("State", isShooting ? "SHOOTING" : (isReleasing ? "RELEASE SEQ" : "IDLE"));
     //   telemetry.addData("Drive", "ROBOT CENTRIC + TRIGGERS");
        telemetry.addData("Amps", "%.2f A", r.intake.getCurrentDraw());
        telemetry.addData("High Preset", "%.2f", highPreset);
        telemetry.addData("Low Preset", "%.2f", lowPreset);
        telemetry.addData("Idle Preset", "%.2f", idlePreset);
        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is si ei SMECHERI");
        telemetry.update();
    }
}