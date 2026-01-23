package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**
 * CAMERA TeleOp - CM UNITS + MORE POWER FOR LEFT BUMPER
 *
 * Manual shot (left bumper): 1400 velocity (increased from 1200)
 * All distances displayed in CENTIMETERS
 */
@TeleOp(name = "Main TeleOp Camera", group = "Competition")
public class MainTeleOp_camera extends OpMode {

    private Robot_camera r;
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    // AUTO-ALIGN PID
    private static final double ALIGN_kP = 0.018;
    private static final double ALIGN_kD = 0.004;
    private static final double ALIGN_MIN_POWER = 0.10;
    private static final double ALIGN_MAX_POWER = 0.45;
    private static final double ALIGN_TOLERANCE = 1.0;

    // PID state
    private double lastAlignError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;
    private boolean alignedRumbled = false;

    // MANUAL SHOT PRESET (left bumper - CLOSE RANGE)
    private static final double MANUAL_ANGLE = 0.70;
    private static final double MANUAL_VELOCITY = 1400;  // INCREASED from 1200!
    private static final double IDLE_PRESET = 1.0;

    // PULSE FEEDING STRATEGY
    private static final double FEED_PULSE_MS = 120;
    private static final double FEED_PAUSE_MS = 100;
    private static final double VELOCITY_DROP_THRESHOLD = 150;
    private static final double MIN_RECOVERY_TIME_MS = 400;

    // VELOCITY READY THRESHOLDS
    private static final double AUTO_VELOCITY_THRESHOLD = 80;    // Auto mode (varies by distance)
    private static final double MANUAL_VELOCITY_THRESHOLD = 120; // Manual close range (FASTER!)

    private enum ShootState {
        WAIT_SPINUP,
        PULSE_FEED,
        PULSE_PAUSE,
        WAIT_RECOVERY
    }
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
        telemetry.addData("Units", "CENTIMETERS");
        telemetry.addLine("Right = AUTO, Left = MANUAL (1400 vel)");
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
            // SMOOTH PID
            double tx = r.limelight.getTx();
            double dt = pidTimer.seconds();
            pidTimer.reset();

            double error = tx;
            double derivative = (error - lastAlignError) / dt;

            double alignPower = (ALIGN_kP * error) + (ALIGN_kD * derivative);

            if (Math.abs(alignPower) > 0.01) {
                if (alignPower > 0) {
                    alignPower = Math.max(alignPower, ALIGN_MIN_POWER);
                } else {
                    alignPower = Math.min(alignPower, -ALIGN_MIN_POWER);
                }
            }

            alignPower = Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, alignPower));
            finalRx = -alignPower;

            lastAlignError = error;

            if (Math.abs(tx) < ALIGN_TOLERANCE) {
                if (!alignedRumbled) {
                    gamepad1.rumble(100);
                    alignedRumbled = true;
                }
            } else {
                alignedRumbled = false;
            }

        } else {
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
        // SHOOTING - MORE POWER FOR MANUAL MODE
        // =========================================================
        boolean autoShot = gamepad1.right_bumper;
        boolean manualShot = gamepad1.left_bumper;
        boolean shootHeld = autoShot || manualShot;

        if (shootHeld) {
            jamRumbled = false;

            // AUTO: Use distance (normal velocity)
            if (autoShot && r.limelight.hasTarget()) {
                double distanceCm = r.limelight.getDistanceToTarget();
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distanceCm);

                currentTargetAngle = config.angle;
                currentTargetVel = config.velocity;
            }
            // MANUAL: Fixed preset (MORE POWER - 1400!)
            else {
                currentTargetAngle = MANUAL_ANGLE;
                currentTargetVel = MANUAL_VELOCITY;
            }

            // Set servos
            r.shooter.setAngle(currentTargetAngle);
            r.shooter.unblock();

            // Spin launcher
            r.shooter.launcher.setVelocity(currentTargetVel);

            double currentVel = r.shooter.getVelocity();

            // DIFFERENT THRESHOLD: Manual is faster!
            double velocityThreshold = manualShot ? MANUAL_VELOCITY_THRESHOLD : AUTO_VELOCITY_THRESHOLD;
            boolean atSpeed = currentVel >= (currentTargetVel - velocityThreshold);

            switch (shootState) {
                case WAIT_SPINUP:
                    r.intake.stop();

                    if (atSpeed) {
                        lastStableVelocity = currentVel;
                        feedTimer.reset();
                        shootState = ShootState.PULSE_FEED;
                    }
                    break;

                case PULSE_FEED:
                    // SHORT PULSE
                    if (feedTimer.milliseconds() < FEED_PULSE_MS) {
                        r.intake.intake();
                    } else {
                        r.intake.stop();
                        feedTimer.reset();
                        shootState = ShootState.PULSE_PAUSE;
                    }

                    // Check for ball launch
                    boolean velocityDropped = (lastStableVelocity - currentVel) > VELOCITY_DROP_THRESHOLD;
                    if (velocityDropped) {
                        r.intake.stop();
                        recoveryTimer.reset();
                        shootState = ShootState.WAIT_RECOVERY;
                    } else if (atSpeed) {
                        lastStableVelocity = currentVel;
                    }
                    break;

                case PULSE_PAUSE:
                    r.intake.stop();

                    // Check for delayed detection
                    boolean delayedDrop = (lastStableVelocity - currentVel) > VELOCITY_DROP_THRESHOLD;
                    if (delayedDrop) {
                        recoveryTimer.reset();
                        shootState = ShootState.WAIT_RECOVERY;
                    }
                    // Pulse again if no ball
                    else if (feedTimer.milliseconds() >= FEED_PAUSE_MS) {
                        if (atSpeed) {
                            feedTimer.reset();
                            shootState = ShootState.PULSE_FEED;
                        }
                    }
                    break;

                case WAIT_RECOVERY:
                    r.intake.stop();

                    boolean minTimeElapsed = recoveryTimer.milliseconds() >= MIN_RECOVERY_TIME_MS;

                    if (minTimeElapsed && atSpeed) {
                        lastStableVelocity = currentVel;
                        feedTimer.reset();
                        shootState = ShootState.PULSE_FEED;
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
                telemetry.addData("WARNING", "JAM! (%.1f A)", currentAmps);
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
        // TELEMETRY - IN CENTIMETERS
        // =========================================================

        telemetry.addData("Auto-Align", autoAlignActive ? "ACTIVE" : "Manual");

        if (r.limelight.hasTarget()) {
            double distanceCm = r.limelight.getDistanceToTarget();

            telemetry.addData("Distance", "%.1f cm (%.2f m)",
                    distanceCm, distanceCm / 100.0);
            telemetry.addData("TX", "%.1f deg", r.limelight.getTx());
            telemetry.addData("Aligned", r.limelight.isAligned(ALIGN_TOLERANCE) ? "YES" : "NO");

            if (autoShot) {
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distanceCm);
                telemetry.addData("Auto Angle", "%.2f", config.angle);
                telemetry.addData("Auto Vel", "%.0f", config.velocity);
            }
        } else {
            telemetry.addData("Limelight", "No Target");
        }

        telemetry.addLine("---");
        if (autoShot) {
            telemetry.addData("Mode", "AUTO (normal)");
        } else if (manualShot) {
            telemetry.addData("Mode", "MANUAL (1400 vel)");
        } else {
            telemetry.addData("Mode", "IDLE");
        }
        telemetry.addData("State", shootState);
        telemetry.addData("Vel", "%.0f / %.0f", r.shooter.getVelocity(), currentTargetVel);
        telemetry.addData("Feed Timer", "%.0f ms", feedTimer.milliseconds());
        telemetry.addData("Recovery", "%.0f ms", recoveryTimer.milliseconds());

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is si ei smecheri");

        telemetry.update();
    }
}