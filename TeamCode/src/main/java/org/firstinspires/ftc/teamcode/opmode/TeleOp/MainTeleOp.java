package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

/**
 * FILE: MainTeleOp.java
 * PURPOSE: The primary driver control program (TeleOp).
 * * CONTROLS:
 * - Left Stick:  Robot movement (Robot Centric).
 * - Right Stick: Fast rotation.
 * - Triggers:    Slow rotation (Precision mode, 30% speed).
 * - Right/Left Bumper: Shoot High/Low Goal.
 * - X Button:    Intake (Holds to intake, Releases to Burp/Unjam).
 * - Y Button:    Manual Outtake.
 * - D-Pad Up/Down: Micro-adjust the shooter angle.
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    // The main Robot object that holds all subsystems
    private Robot r;

    // --- TIMERS & STATE VARIABLES ---
    // Timer to wait for the servo to reach its angle before shooting
    private ElapsedTime servoTimer = new ElapsedTime();

    // Timer to track the stages of the "Release Sequence" (Burp -> Wait -> Reverse)
    private ElapsedTime releaseTimer = new ElapsedTime();

    private double calculatedWaitTime = 0; // How long to wait based on distance moved
    private boolean isShooting = false;    // Are we currently in the middle of a shot?
    private boolean isReleasing = false;   // Are we actively running the Burp/Reverse sequence?

    // RUMBLE FLAGS (To prevent the controller from vibrating endlessly)
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;

    // BUTTON MEMORY (Used to detect when a button is *released*)
    private boolean lastX = false;
    private boolean lastUp = false, lastDown = false;

    // SHOOTER PRESETS (Modifiable via D-Pad)
    private double highPreset = 0.65;
    private double lowPreset = 0.7;
    private double idlePreset = 1.0;

    // CONSTANTS
    // Wait 650ms for every 1.0 unit of Servo movement. Higher = Safer/Slower.
    private final double SERVO_SPEED_FACTOR = 650.0;

    // Allow shooting if flywheel speed is within 50 ticks of target
    private final double VELOCITY_TOLERANCE = 50;

    @Override
    public void init() {
        // Initialize the robot hardware
        r = new Robot(hardwareMap, Alliance.BLUE);
    }

    @Override
    public void loop() {
        // Run periodic updates (Bulk Read sensors, PID loops)
        r.periodic();

        // =========================================================
        //                 1. DRIVING (Hybrid Control)
        // =========================================================
        double y = -gamepad1.left_stick_y; // Forward/Backward
        double x = gamepad1.left_stick_x;  // Strafe Left/Right

        // A. STICK TURN (Fast)
        double stickTurn = -gamepad1.right_stick_x;

        // B. TRIGGER TURN (Slow - 30% Power)
        // Left Trigger turns Left (+), Right Trigger turns Right (-)
        double triggerTurn = (gamepad1.left_trigger * 0.3) - (gamepad1.right_trigger * 0.3);

        // Combine inputs: Stick + Triggers
        double finalRx = stickTurn + triggerTurn;

        // Send commands to the Drive Subsystem
        r.drive.driveRobotCentric(x, y, finalRx);

        // Options Button: Reset Gyro (Re-centers "Forward")
        if (gamepad1.options) {
            r.drive.resetHeading();
            gamepad1.rumble(500);
        }

        // Endgame Alert: Rumble controller at 2 minutes
        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000);
            endgameRumbled = true;
        }

        // --- MICRO-ADJUSTMENTS (D-Pad) ---
        // Allows the driver to tweak the shooting angle mid-match
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

        // Safety Clamps: Ensure values never go below 0 or above 1
        highPreset = Math.max(0, Math.min(1, highPreset));
        lowPreset = Math.max(0, Math.min(1, lowPreset));
        idlePreset = Math.max(0, Math.min(1, idlePreset));

        // =========================================================
        //                 MAIN LOGIC (Priority Chain)
        // =========================================================

        // PRIORITY 1: SHOOTING (Bumpers)
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            jamRumbled = false;
            isReleasing = false; // Cancel any active burp sequence

            double targetAngle = gamepad1.right_bumper ? highPreset : lowPreset;
            double targetVel = gamepad1.right_bumper ? 1550 : 1200;

            // Step A: Setup (First Loop Only)
            if (!isShooting) {
                // Calculate wait time based on how far the servo needs to move
                double currentPos = r.shooter.getAngle();
                double distance = Math.abs(targetAngle - currentPos);
                calculatedWaitTime = distance * SERVO_SPEED_FACTOR;

                r.shooter.setAngle(targetAngle);
                servoTimer.reset();
                isShooting = true;
            } else {
                r.shooter.setAngle(targetAngle); // Hold angle
            }

            // Step B: Wait & Fire
            if (servoTimer.milliseconds() >= calculatedWaitTime) {
                // Spin Flywheel
                if (gamepad1.right_bumper) r.shooter.spinHigh();
                else r.shooter.spinLow();

                // Only feed (shoot) if the flywheel is up to speed
                if (r.shooter.getVelocity() >= (targetVel - VELOCITY_TOLERANCE)) {
                    r.intake.intake();
                } else {
                    r.intake.stop(); // Prevent jamming if motor slows down
                }
            } else {
                // Still waiting for servo...
                r.shooter.stop();
                r.intake.stop();
            }
        }

        // PRIORITY 2: INTAKE (X Button)
        else if (gamepad1.x) {
            isShooting = false;
            isReleasing = false; // Reset the release sequence if we press X again

            r.shooter.setAngle(idlePreset);
            r.shooter.stop();

            // JAM PROTECTION: Check motor Amps
            double currentAmps = r.intake.getCurrentDraw();

            if (currentAmps > Intake.JAM_THRESHOLD) {
                // JAM DETECTED (Over 6.0A) -> STOP!
                r.intake.stop();
                if (!jamRumbled) {
                    gamepad1.rumble(500);
                    jamRumbled = true;
                }
                telemetry.addData("WARNING", "INTAKE OVERLOAD! (%.1f A)", currentAmps);
            } else {
                // Safe to run
                r.intake.intake();
                jamRumbled = false;
            }
        }

        // PRIORITY 3: OUTTAKE (Y Button)
        else if (gamepad1.y) {
            isShooting = false;
            isReleasing = false;
            jamRumbled = false;
            r.shooter.setAngle(idlePreset);
            r.intake.outtakeSlow();
            r.shooter.stop();
        }

        // PRIORITY 4: RELEASE SEQUENCE (Smart Burp + Unjam)
        else if (isReleasing) {
            double t = releaseTimer.milliseconds();

            // Phase A: Burp (0ms - 150ms)
            // Quickly reverse intake to clear the "Flywheel Rub"
            if (t < 150) {
                r.intake.outtakeSuperSlow();
                r.shooter.stop();
                telemetry.addData("Seq", "Burping...");
            }
            // Phase B: Wait (150ms - 1150ms)
            // 1 Second of silence to let things settle
            else if (t < 1150) {
                r.intake.stop();
                r.shooter.stop();
                telemetry.addData("Seq", "Waiting 1s...");
            }
            // Phase C: Reverse Launcher (1150ms - 1650ms)
            // Run Launcher backwards for 0.5s to clear internal jams
            else if (t < 1650) {
                r.intake.stop();
                r.shooter.reverse();
                telemetry.addData("Seq", "Reversing Launcher...");
            }
            // Phase D: Done
            else {
                isReleasing = false;
                r.shooter.stop();
                r.intake.stop();
            }
        }

        // PRIORITY 5: IDLE / CHECK RELEASE TRIGGER
        else {
            isShooting = false;
            jamRumbled = false;

            // CHECK: Did we just release the X button?
            if (lastX) {
                isReleasing = true; // Start the Sequence
                releaseTimer.reset();
            }

            r.shooter.setAngle(idlePreset);
            r.shooter.stop();
            r.intake.stop();

            // Manual Override (D-Pad Left)
            if (gamepad1.dpad_left) r.shooter.reverse();
        }

        // Update button history for next loop
        lastX = gamepad1.x;

        // Telemetry (Debug Info)
        telemetry.addData("State", isShooting ? "SHOOTING" : (isReleasing ? "RELEASE SEQ" : "IDLE"));
        telemetry.addData("Servo Factor", "%.0f ms/unit", SERVO_SPEED_FACTOR);
        telemetry.addData("Amps", "%.2f A", r.intake.getCurrentDraw());

        telemetry.addData("High Preset", "%.2f", highPreset);
        telemetry.addData("Low Preset", "%.2f", lowPreset);
        telemetry.addData("Idle Preset", "%.2f", idlePreset);

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is niste slabi");
        telemetry.update();
    }
}