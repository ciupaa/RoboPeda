package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**
 * CAMERA TeleOp - Auto-Align + Distance-Based Shooting
 *
 * CONTROLS:
 * - Both Triggers: Auto-align to AprilTag
 * - Right Bumper: AUTO SHOT (uses Limelight distance)
 * - Left Bumper: MANUAL SHOT (fixed preset)
 * - X: Intake
 * - Y: Outtake
 * - D-Pad Left: Reverse launcher
 */
@TeleOp(name = "Main TeleOp Camera", group = "Competition")
public class MainTeleOp_camera extends OpMode {

    private Robot_camera r;
    private final ElapsedTime recoveryTimer = new ElapsedTime();

    // AUTO-ALIGN PID
    private static final double ALIGN_kP = 0.015;
    private static final double ALIGN_kD = 0.001;
    private static final double ALIGN_MIN_POWER = 0.08;
    private static final double ALIGN_MAX_POWER = 0.4;
    private static final double ALIGN_TOLERANCE = 2.0;

    // PID state
    private double lastAlignError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;
    private boolean alignedRumbled = false;

    // MANUAL SHOT PRESET (left bumper)
    private static final double MANUAL_ANGLE = 0.70;
    private static final double MANUAL_VELOCITY = 1350;
    private static final double IDLE_PRESET = 1.0;

    // VELOCITY DETECTION
    private static final double VELOCITY_DROP_THRESHOLD = 100;
    private static final double MIN_RECOVERY_TIME_MS = 80;
    private static final double VELOCITY_READY_THRESHOLD = 50;

    private enum ShootState { WAIT_SPINUP, FEEDING, WAIT_RECOVERY }
    private ShootState shootState = ShootState.WAIT_SPINUP;

    private double currentTargetAngle = 1.0;
    private double currentTargetVel = 0.0;
    private double lastStableVelocity = 0.0;

    @Override
    public void init() {
        r = new Robot_camera(hardwareMap, Alliance.BLUE);
        r.shooter.block();
        pidTimer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "CAMERA TeleOp");
        telemetry.addLine("Right Bumper = AUTO (distance)");
        telemetry.addLine("Left Bumper = MANUAL");
    }

    @Override
    public void loop() {
        r.periodic();

        // =========================================================
        // DRIVING WITH AUTO-ALIGN
        // =========================================================
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;

        boolean autoAlignActive = gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5;

        double finalRx;

        if (autoAlignActive && r.limelight.hasTarget()) {
            // PID AUTO-ALIGN
            double tx = r.limelight.getTx();
            double dt = pidTimer.seconds();
            pidTimer.reset();

            double error = tx;
            double derivative = (error - lastAlignError) / dt;

            double alignPower = (ALIGN_kP * error) + (ALIGN_kD * derivative);

            // Apply minimum
            if (Math.abs(alignPower) > 0.01) {
                if (alignPower > 0) {
                    alignPower = Math.max(alignPower, ALIGN_MIN_POWER);
                } else {
                    alignPower = Math.min(alignPower, -ALIGN_MIN_POWER);
                }
            }

            // Clamp max
            alignPower = Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, alignPower));
            finalRx = -alignPower;

            lastAlignError = error;

            // Rumble when aligned
            if (Math.abs(tx) < ALIGN_TOLERANCE) {
                if (!alignedRumbled) {
                    gamepad1.rumble(100);
                    alignedRumbled = true;
                }
            } else {
                alignedRumbled = false;
            }

        } else {
            // MANUAL ROTATION
            finalRx = -gamepad1.right_stick_x;
            lastAlignError = 0;
            alignedRumbled = false;
        }

        r.drive.driveRobotCentric(x, y, finalRx);

        // Endgame rumble
        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000);
            endgameRumbled = true;
        }

        // =========================================================
        // SHOOTING - AUTO vs MANUAL
        // =========================================================
        boolean autoShot = gamepad1.right_bumper;
        boolean manualShot = gamepad1.left_bumper;
        boolean shootHeld = autoShot || manualShot;

        if (shootHeld) {
            jamRumbled = false;

            // AUTO: Use distance
            if (autoShot && r.limelight.hasTarget()) {
                double distance = r.limelight.getDistanceToTarget();
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distance);

                currentTargetAngle = config.angle;
                currentTargetVel = config.velocity;
            }
            // MANUAL: Fixed preset
            else {
                currentTargetAngle = MANUAL_ANGLE;
                currentTargetVel = MANUAL_VELOCITY;
            }

            // Set servos
            r.shooter.setAngle(currentTargetAngle);
            r.shooter.unblock();

            // Spin to calculated velocity
            r.shooter.launcher.setVelocity(currentTargetVel);

            double currentVel = r.shooter.getVelocity();
            boolean atSpeed = currentVel >= (currentTargetVel - VELOCITY_READY_THRESHOLD);

            switch (shootState) {
                case WAIT_SPINUP:
                    r.intake.stop();

                    if (atSpeed) {
                        lastStableVelocity = currentVel;
                        shootState = ShootState.FEEDING;
                    }
                    break;

                case FEEDING:
                    r.intake.intake();

                    boolean velocityDropped = (lastStableVelocity - currentVel) > VELOCITY_DROP_THRESHOLD;

                    if (velocityDropped) {
                        r.intake.stop();
                        recoveryTimer.reset();
                        shootState = ShootState.WAIT_RECOVERY;
                    } else if (atSpeed) {
                        lastStableVelocity = currentVel;
                    }
                    break;

                case WAIT_RECOVERY:
                    r.intake.stop();

                    boolean minTimeElapsed = recoveryTimer.milliseconds() >= MIN_RECOVERY_TIME_MS;

                    if (minTimeElapsed && atSpeed) {
                        lastStableVelocity = currentVel;
                        shootState = ShootState.FEEDING;
                    }
                    break;
            }
        }

        // INTAKE (X)
        else if (gamepad1.x) {
            shootState = ShootState.WAIT_SPINUP;

            r.shooter.setAngle(IDLE_PRESET);
            r.shooter.stop();
            r.shooter.block();
            r.intake.intake();

            double currentAmps = r.intake.getCurrentDraw();
            if (currentAmps > Intake.JAM_THRESHOLD) {
                if (!jamRumbled) {
                    gamepad1.rumble(500);
                    jamRumbled = true;
                }
                telemetry.addData("⚠ WARNING", "JAM! (%.1f A)", currentAmps);
            } else {
                jamRumbled = false;
            }
        }

        // OUTTAKE (Y)
        else if (gamepad1.y) {
            shootState = ShootState.WAIT_SPINUP;
            jamRumbled = false;

            r.shooter.setAngle(IDLE_PRESET);
            r.intake.outtakeSlow();
            r.shooter.stop();
            r.shooter.block();
        }

        // IDLE
        else {
            shootState = ShootState.WAIT_SPINUP;
            jamRumbled = false;

            r.shooter.setAngle(IDLE_PRESET);
            r.shooter.stop();
            r.intake.stop();
            r.shooter.block();

            if (gamepad1.dpad_left) {
                r.shooter.reverse();
            }
        }

        // =========================================================
        // TELEMETRY
        // =========================================================

        telemetry.addData("🎯 Auto-Align", autoAlignActive ? "ACTIVE" : "Manual");

        if (r.limelight.hasTarget()) {
            double distance = r.limelight.getDistanceToTarget();

            telemetry.addData("📏 Distance", "%.1f\" (%.1f ft)",
                    distance, distance / 12.0);
            telemetry.addData("TX", "%.1f°", r.limelight.getTx());
            telemetry.addData("Aligned", r.limelight.isAligned(ALIGN_TOLERANCE) ? "✓" : "✗");

            if (autoShot) {
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distance);
                telemetry.addData("Calc Angle", "%.2f", config.angle);
                telemetry.addData("Calc Vel", "%.0f", config.velocity);
            }
        } else {
            telemetry.addData("Limelight", "No Target");
        }

        telemetry.addLine("---");
        telemetry.addData("Mode", autoShot ? "AUTO" : (manualShot ? "MANUAL" : "IDLE"));
        telemetry.addData("State", shootState);
        telemetry.addData("Vel", "%.0f / %.0f", r.shooter.getVelocity(), currentTargetVel);

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is si ei smecheri");

        telemetry.update();
    }
}