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
 * LOGIC: Blocker opens IMMEDIATELY on bumper press and stays open until release.
 *
 * FIX: Feed is now a proper per-ball sequence:
 *  - Wait for angle
 *  - Wait for velocity to be stable at target
 *  - Pulse intake to launch ONE ball
 *  - Lockout + require velocity recovery before pulsing again
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    private Robot r;

    // --- TIMERS ---
    private final ElapsedTime servoTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime stableTimer = new ElapsedTime();

    private double calculatedWaitTimeMs = 0;

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;

    // BUTTON MEMORY
    private boolean lastUp = false, lastDown = false;
    private boolean lastA = false, lastB = false;

    // PRESETS
    private double highPreset = 0.65;
    private double lowPreset  = 0.70;
    private double idlePreset = 1.0;

    // CONSTANTS (your existing model)
    private final double SERVO_SPEED_FACTOR_MS_PER_UNIT = 650.0;
    private final double VELOCITY_TOLERANCE = 50;

    // --- NEW: Per-ball sequencing tuning ---
    private static final double VELOCITY_STABLE_MS = 120;  // must be at-speed continuously for this long
    private static final double FEED_PULSE_MS = 140;       // how long intake runs to push ONE ball
    private static final double POST_FEED_LOCKOUT_MS = 160;// dead time after a shot (prevents "double feed")

    private enum ShootState { IDLE, AIMING, SPINUP_STABLE, FEED_PULSE, RECOVER }
    private ShootState shootState = ShootState.IDLE;

    private boolean isShooting = false; // kept for your telemetry/state display
    private double currentTargetAngle = 1.0;
    private double currentTargetVel = 0.0;

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);

        // Start closed
        r.shooter.block();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        r.periodic();

        // =========================================================
        // 1) DRIVING
        // =========================================================
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double finalRx = -gamepad1.right_stick_x
                + (gamepad1.left_trigger * 0.3)
                - (gamepad1.right_trigger * 0.3);

        r.drive.driveRobotCentric(x, y, finalRx);

        // Endgame rumble
        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000);
            endgameRumbled = true;
        }

        // =========================================================
        // 2) LIVE TUNING
        // =========================================================
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

        // Blocker tuning (A/B) - unchanged
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

        // Safety clamps
        highPreset = Math.max(0, Math.min(1, highPreset));
        lowPreset = Math.max(0, Math.min(1, lowPreset));
        idlePreset = Math.max(0, Math.min(1, idlePreset));
        r.shooter.unblockPos = Math.max(0, Math.min(1, r.shooter.unblockPos));
        r.shooter.blockPos   = Math.max(0, Math.min(1, r.shooter.blockPos));

        // =========================================================
        //                 MAIN LOGIC
        // =========================================================

        boolean highHeld = gamepad1.right_bumper;
        boolean lowHeld = gamepad1.left_bumper;
        boolean shootHeld = highHeld || lowHeld;

        // 1) SHOOTING (Bumpers) - FIXED sequence, blocker stays open while held
        if (shootHeld) {
            jamRumbled = false;
            isShooting = true;

            currentTargetAngle = highHeld ? highPreset : lowPreset;
            currentTargetVel   = highHeld ? 1550 : 1200;

            boolean angleReady = servoTimer.milliseconds() >= calculatedWaitTimeMs;
            boolean velocityReady = r.shooter.getVelocity() >= (currentTargetVel - VELOCITY_TOLERANCE);

            // Your requirement: open forever while bumper is held
            r.shooter.unblock();

            switch (shootState) {
                case IDLE: {
                    // Start aiming calculation once
                    double currentPos = r.shooter.getAngle();
                    double distance = Math.abs(currentTargetAngle - currentPos);
                    calculatedWaitTimeMs = distance * SERVO_SPEED_FACTOR_MS_PER_UNIT;

                    r.shooter.setAngle(currentTargetAngle);
                    servoTimer.reset();

                    stableTimer.reset();
                    shotTimer.reset();

                    shootState = ShootState.AIMING;
                    break;
                }

                case AIMING: {
                    r.shooter.setAngle(currentTargetAngle);

                    // Spin while aiming (faster overall)
                    if (highHeld) r.shooter.spinHigh();
                    else r.shooter.spinLow();

                    // Do not feed while angle moving
                    r.intake.stop();

                    if (angleReady) {
                        stableTimer.reset();
                        shootState = ShootState.SPINUP_STABLE;
                    }
                    break;
                }

                case SPINUP_STABLE: {
                    r.shooter.setAngle(currentTargetAngle);

                    if (highHeld) r.shooter.spinHigh();
                    else r.shooter.spinLow();

                    // Only allow a shot when velocity is stable continuously
                    r.intake.stop();

                    if (velocityReady) {
                        if (stableTimer.milliseconds() >= VELOCITY_STABLE_MS) {
                            shotTimer.reset();
                            shootState = ShootState.FEED_PULSE;
                        }
                    } else {
                        stableTimer.reset();
                    }
                    break;
                }

                case FEED_PULSE: {
                    r.shooter.setAngle(currentTargetAngle);

                    if (highHeld) r.shooter.spinHigh();
                    else r.shooter.spinLow();

                    // Pulse intake for ONE ball attempt
                    if (shotTimer.milliseconds() <= FEED_PULSE_MS) {
                        r.intake.intake();
                    } else {
                        r.intake.stop();
                    }

                    // After pulse + lockout, go recover
                    if (shotTimer.milliseconds() > (FEED_PULSE_MS + POST_FEED_LOCKOUT_MS)) {
                        stableTimer.reset();
                        shootState = ShootState.RECOVER;
                    }
                    break;
                }

                case RECOVER: {
                    r.shooter.setAngle(currentTargetAngle);

                    if (highHeld) r.shooter.spinHigh();
                    else r.shooter.spinLow();

                    // Never feed in recovery; wait for velocity stable again
                    r.intake.stop();

                    if (velocityReady) {
                        if (stableTimer.milliseconds() >= VELOCITY_STABLE_MS) {
                            shotTimer.reset();
                            shootState = ShootState.FEED_PULSE;
                        }
                    } else {
                        stableTimer.reset();
                    }
                    break;
                }
            }
        }

        // 2) INTAKE (X)
        else if (gamepad1.x) {
            // Reset shooting state hard
            shootState = ShootState.IDLE;
            isShooting = false;

            r.shooter.setAngle(idlePreset);
            r.shooter.stop();
            r.shooter.block();

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

        // 3) OUTTAKE (Y)
        else if (gamepad1.y) {
            shootState = ShootState.IDLE;
            isShooting = false;
            jamRumbled = false;

            r.shooter.setAngle(idlePreset);
            r.intake.outtakeSlow();
            r.shooter.stop();
            r.shooter.block();
        }

        // 4) IDLE
        else {
            shootState = ShootState.IDLE;
            isShooting = false;
            jamRumbled = false;

            r.shooter.setAngle(idlePreset);
            r.shooter.stop();
            r.intake.stop();
            r.shooter.block();

            if (gamepad1.dpad_left) r.shooter.reverse();
        }

        // =========================================================
        // TELEMETRY
        // =========================================================
        telemetry.addData("State", isShooting ? ("SHOOTING: " + shootState) : "IDLE");
        telemetry.addData("Velocity", "%.0f", r.shooter.getVelocity());
        telemetry.addData("TargetVel", "%.0f", currentTargetVel);
        telemetry.addData("AngleReadyMs", "%.0f/%.0f", servoTimer.milliseconds(), calculatedWaitTimeMs);

        telemetry.addLine("--- Blocker Status ---");
        telemetry.addData("Target Pos", shootHeld ? r.shooter.unblockPos : r.shooter.blockPos);

        telemetry.addData("High Preset", "%.2f", highPreset);
        telemetry.addData("Low Preset", "%.2f", lowPreset);

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is si ei smecheri");

        telemetry.update();
    }
}
