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
 * - Right/Left Bumper: Shoot High/Low Goal (Smart Shot Sequence).
 * - X Button:    Intake (Holds to intake).
 * - Y Button:    Manual Outtake.
 * - D-Pad Up/Down: Micro-adjust the shooter angle.
 * - A/B Buttons: Micro-adjust the Blocker servo (+/- 0.01).
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    // The main Robot object that holds all subsystems
    private Robot r;

    // --- TIMERS & STATE VARIABLES ---
    private ElapsedTime servoTimer = new ElapsedTime(); // For Angle Servo
    private ElapsedTime blockerTimer = new ElapsedTime(); // For Blocker Delay

    private double calculatedWaitTime = 0; // How long to wait based on distance moved
    private boolean isShooting = false;    // Are we currently in the middle of a shot?

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;

    // BUTTON MEMORY
    private boolean lastX = false;
    private boolean lastUp = false, lastDown = false;
    private boolean lastA = false, lastB = false;

    // SHOOTER PRESETS (Modifiable via D-Pad)
    private double highPreset = 0.65;
    private double lowPreset = 0.7;
    private double idlePreset = 1.0;

    // CONSTANTS
    private final double SERVO_SPEED_FACTOR = 650.0;
    private final double VELOCITY_TOLERANCE = 50;

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        r.shooter.block(); // Start Closed
    }

    @Override
    public void loop() {
        r.periodic();

        // =========================================================
        //                 1. DRIVING (Hybrid Control)
        // =========================================================
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double stickTurn = -gamepad1.right_stick_x;
        double triggerTurn = (gamepad1.left_trigger * 0.3) - (gamepad1.right_trigger * 0.3);
        double finalRx = stickTurn + triggerTurn;

        r.drive.driveRobotCentric(x, y, finalRx);

        if (gamepad1.options) {
            r.drive.resetHeading();
            gamepad1.rumble(500);
        }

        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000);
            endgameRumbled = true;
        }

        // --- SHOOTER ANGLE ADJUST (D-Pad) ---
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

        // --- BLOCKER ADJUST (A / B) ---
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
        idlePreset = Math.max(0, Math.min(1, idlePreset));
        r.shooter.unblockPos = Math.max(0, Math.min(1, r.shooter.unblockPos));
        r.shooter.blockPos = Math.max(0, Math.min(1, r.shooter.blockPos));

        // =========================================================
        //                 MAIN LOGIC (Priority Chain)
        // =========================================================

        // PRIORITY 1: SHOOTING (Bumpers)
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            jamRumbled = false;

            double targetAngle = gamepad1.right_bumper ? highPreset : lowPreset;
            double targetVel = gamepad1.right_bumper ? 1550 : 1200;

            if (!isShooting) {
                // --- INITIALIZE SHOT ---
                double currentPos = r.shooter.getAngle();
                double distance = Math.abs(targetAngle - currentPos);
                calculatedWaitTime = distance * SERVO_SPEED_FACTOR;

                // 1. Set Angle
                r.shooter.setAngle(targetAngle);

                // 2. Open Blocker IMMEDIATELY (Same time as angle)
                r.shooter.unblock();

                // 3. Reset Timers
                servoTimer.reset();
                blockerTimer.reset(); // Start 0.3s timer

                isShooting = true;
            }
            else {
                // --- HOLD STATE ---
                r.shooter.setAngle(targetAngle);
                r.shooter.unblock(); // Keep holding it open
            }

            // Spin Flywheel
            if (gamepad1.right_bumper) r.shooter.spinHigh();
            else r.shooter.spinLow();

            // --- FEED LOGIC ---
            // Wait for:
            // 1. Angle Servo to finish moving
            // 2. Blocker Timer to pass 0.3s (300ms)
            // 3. Flywheel Velocity to be ready

            boolean angleReady = servoTimer.milliseconds() >= calculatedWaitTime;
            boolean blockerReady = blockerTimer.milliseconds() >= 300;
            boolean velocityReady = r.shooter.getVelocity() >= (targetVel - VELOCITY_TOLERANCE);

            if (angleReady && blockerReady && velocityReady) {
                r.intake.intake(); // Feed rings
            } else {
                r.intake.stop();   // Wait
            }
        }

        // PRIORITY 2: INTAKE (X Button)
        else if (gamepad1.x) {
            isShooting = false;
            r.shooter.setAngle(idlePreset);
            r.shooter.stop();
            r.shooter.block(); // Close Gate

            r.intake.intake();

            // Jam Warning
            double currentAmps = r.intake.getCurrentDraw();
            if (currentAmps > Intake.JAM_THRESHOLD) {
                if (!jamRumbled) {
                    gamepad1.rumble(300);
                    jamRumbled = true;
                }
                telemetry.addData("WARNING", "INTAKE OVERLOAD! (%.1f A)", currentAmps);
            } else {
                jamRumbled = false;
            }
        }

        // PRIORITY 3: OUTTAKE (Y Button)
        else if (gamepad1.y) {
            isShooting = false;
            jamRumbled = false;
            r.shooter.setAngle(idlePreset);
            r.intake.outtakeSlow();
            r.shooter.stop();
            r.shooter.block();
        }

        // PRIORITY 4: IDLE
        else {
            isShooting = false;
            jamRumbled = false;

            r.shooter.setAngle(idlePreset);
            r.shooter.stop();
            r.intake.stop();

            // Close Gate when not shooting
            r.shooter.block();

            if (gamepad1.dpad_left) r.shooter.reverse();
        }

        // Update history
        lastX = gamepad1.x;

        // Telemetry
        telemetry.addData("State", isShooting ? "SHOOTING" : "IDLE");
        if(isShooting) {
            telemetry.addData("Sequence", blockerTimer.milliseconds() > 300 ? "FEEDING" : "WAITING 0.3s");
        }
        telemetry.addData("Blocker Pos", r.shooter.unblockPos);

        telemetry.addLine("--- Blocker Tuning ---");
        telemetry.addData("Block Open (A/B + Bumper)", "%.2f", r.shooter.unblockPos);
        telemetry.addData("Block Closed (A/B)", "%.2f", r.shooter.blockPos);

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is si ei smecheri");
        telemetry.update();
    }
}