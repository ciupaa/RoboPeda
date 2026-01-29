package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**
 * RED ALLIANCE - TAG 24 ONLY - PIPELINE 2
 *
 * RIGHT BUMPER: Auto-align heading (Pinpoint) → THEN auto-shoot when aligned
 * LEFT BUMPER: Manual close shot (no alignment)
 */
@Configurable
@TeleOp(name = "MainTeleOp_Red_camera", group = "Competition")
public class MainTeleOp_Red_camera extends OpMode {

    private Robot_camera r;
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    // AUTO-ALIGN HEADING PID
    public static double HEADING_kP = 0.020;
    public static double HEADING_kD = 0.005;
    public static double HEADING_MIN_POWER = 0.04;
    public static double HEADING_MAX_POWER = 0.35;
    public static double HEADING_TOLERANCE = 0.5;
    public static double HEADING_DEADBAND = 0.3;

    public static double RED_TARGET_HEADING = 130.0;

    private double lastHeadingError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;
    private boolean alignedRumbled = false;

    private static final double MANUAL_ANGLE = 0.70;
    private static final double MANUAL_VELOCITY = 1200;
    private static final double IDLE_PRESET = 1.0;

    private static final double FEED_PULSE_MS = 120;
    private static final double FEED_PAUSE_MS = 100;
    private static final double VELOCITY_DROP_THRESHOLD = 150;
    private static final double MIN_RECOVERY_TIME_MS = 400;

    private static final double AUTO_VELOCITY_THRESHOLD = 80;
    private static final double MANUAL_VELOCITY_THRESHOLD = 120;

    private enum AutoShootState {
        IDLE,
        ALIGNING,
        ALIGNED_SHOOTING
    }
    private AutoShootState autoShootState = AutoShootState.IDLE;

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
        r = new Robot_camera(hardwareMap, Alliance.RED);
        r.shooter.block();
        pidTimer.reset();

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Target Tag", "24");
        telemetry.addData("Pipeline", "2");
        telemetry.addLine("Right = ALIGN + SHOOT");
        telemetry.addLine("Left = MANUAL");
    }

    @Override
    public void loop() {
        r.periodic();

        if (gamepad1.options) {
            Pose current = r.f.getPose();
            r.f.setStartingPose(new Pose(current.getX(), current.getY(), 0));
            gamepad1.rumble(200);
        }

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        boolean rightBumper = gamepad1.right_bumper;
        boolean leftBumper = gamepad1.left_bumper;

        // ========== RIGHT BUMPER: ALIGN THEN SHOOT ==========
        if (rightBumper && r.limelight.hasTarget()) {

            double currentHeadingDeg = Math.toDegrees(r.f.getPose().getHeading());
            double headingError = normalizeAngle(RED_TARGET_HEADING - currentHeadingDeg);

            if (Math.abs(headingError) < HEADING_DEADBAND) {
                headingError = 0;
            }

            double dt = pidTimer.seconds();
            pidTimer.reset();

            double derivative = (headingError - lastHeadingError) / dt;
            double alignPower = (HEADING_kP * headingError) + (HEADING_kD * derivative);

            if (Math.abs(alignPower) > 0.01) {
                alignPower = alignPower > 0
                        ? Math.max(alignPower, HEADING_MIN_POWER)
                        : Math.min(alignPower, -HEADING_MIN_POWER);
            }

            alignPower = Math.max(-HEADING_MAX_POWER, Math.min(HEADING_MAX_POWER, alignPower));
            lastHeadingError = headingError;

            boolean isAligned = Math.abs(headingError) < HEADING_TOLERANCE;

            if (isAligned) {
                // ALIGNED! NOW SHOOT
                autoShootState = AutoShootState.ALIGNED_SHOOTING;
                r.drive.driveRobotCentric(x, y, 0);

                double distanceCm = r.limelight.getDistanceToTarget();
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distanceCm);

                currentTargetAngle = config.angle;
                currentTargetVel = config.velocity;

                executeShootingSequence(AUTO_VELOCITY_THRESHOLD);

                if (!alignedRumbled) {
                    gamepad1.rumble(100);
                    alignedRumbled = true;
                }
            } else {
                // STILL ALIGNING - DON'T SHOOT YET
                autoShootState = AutoShootState.ALIGNING;
                alignedRumbled = false;

                r.drive.driveRobotCentric(x, y, alignPower);  // RED: INVERTED

                r.shooter.stop();
                r.intake.stop();
                r.shooter.block();
                shootState = ShootState.WAIT_SPINUP;
            }
        }

        // ========== LEFT BUMPER: MANUAL ==========
        else if (leftBumper) {
            autoShootState = AutoShootState.IDLE;
            alignedRumbled = false;
            lastHeadingError = 0;

            r.drive.driveRobotCentric(x, y, -gamepad1.right_stick_x);

            currentTargetAngle = MANUAL_ANGLE;
            currentTargetVel = MANUAL_VELOCITY;

            executeShootingSequence(MANUAL_VELOCITY_THRESHOLD);
        }

        // ========== INTAKE (X) ==========
        else if (gamepad1.x) {
            autoShootState = AutoShootState.IDLE;
            shootState = ShootState.WAIT_SPINUP;
            alignedRumbled = false;
            lastHeadingError = 0;

            r.drive.driveRobotCentric(x, y, -gamepad1.right_stick_x);

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
            } else {
                jamRumbled = false;
            }
        }

        // ========== OUTTAKE (Y) ==========
        else if (gamepad1.y) {
            autoShootState = AutoShootState.IDLE;
            shootState = ShootState.WAIT_SPINUP;
            jamRumbled = false;
            alignedRumbled = false;
            lastHeadingError = 0;

            r.drive.driveRobotCentric(x, y, -gamepad1.right_stick_x);

            r.shooter.setAngle(IDLE_PRESET);
            r.intake.outtakeSlow();
            r.shooter.stop();
            r.shooter.block();
        }

        // ========== IDLE ==========
        else {
            autoShootState = AutoShootState.IDLE;
            shootState = ShootState.WAIT_SPINUP;
            jamRumbled = false;
            alignedRumbled = false;
            lastHeadingError = 0;

            r.drive.driveRobotCentric(x, y, -gamepad1.right_stick_x);

            r.shooter.setAngle(IDLE_PRESET);
            r.shooter.stop();
            r.intake.stop();
            r.shooter.block();

            if (gamepad1.dpad_left) {
                r.shooter.reverse();
            }
        }

        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000);
            endgameRumbled = true;
        }

        updateTelemetry();
    }

    private void executeShootingSequence(double velocityThreshold) {
        r.shooter.setAngle(currentTargetAngle);
        r.shooter.unblock();
        r.shooter.launcher.setVelocity(currentTargetVel);

        double currentVel = r.shooter.getVelocity();
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
                if (feedTimer.milliseconds() < FEED_PULSE_MS) {
                    r.intake.intake();
                } else {
                    r.intake.stop();
                    feedTimer.reset();
                    shootState = ShootState.PULSE_PAUSE;
                }

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
                boolean delayedDrop = (lastStableVelocity - currentVel) > VELOCITY_DROP_THRESHOLD;
                if (delayedDrop) {
                    recoveryTimer.reset();
                    shootState = ShootState.WAIT_RECOVERY;
                } else if (feedTimer.milliseconds() >= FEED_PAUSE_MS) {
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

    private void updateTelemetry() {
        double currentHeadingDeg = Math.toDegrees(r.f.getPose().getHeading());
        double headingError = normalizeAngle(RED_TARGET_HEADING - currentHeadingDeg);

        telemetry.addLine("=== RED (TAG 24, PIPELINE 2) ===");
        telemetry.addData("Heading", "%.2f° → %.2f°", currentHeadingDeg, RED_TARGET_HEADING);

        if (autoShootState == AutoShootState.IDLE) {
            telemetry.addData("Preview", "Error: %.2f°", headingError);
        } else if (autoShootState == AutoShootState.ALIGNING) {
            telemetry.addData("Status", "ALIGNING (%.2f°)", headingError);
        } else if (autoShootState == AutoShootState.ALIGNED_SHOOTING) {
            telemetry.addData("Status", "✓ ALIGNED - SHOOTING");
        }

        telemetry.addLine("");
        if (r.limelight.hasTarget()) {
            double distance = r.limelight.getDistanceToTarget();
            telemetry.addData("Distance", "%.1f cm", distance);
            telemetry.addData("TX", "%.1f°", r.limelight.getTx());

            if (gamepad1.right_bumper) {
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distance);
                telemetry.addData("Auto Angle", "%.2f", config.angle);
                telemetry.addData("Auto Vel", "%.0f", config.velocity);
            }
        } else {
            telemetry.addData("Camera", "No Tag 24");
        }

        telemetry.addLine("");
        telemetry.addData("Mode", gamepad1.right_bumper ? "AUTO" :
                gamepad1.left_bumper ? "MANUAL" : "IDLE");
        telemetry.addData("State", shootState);
        telemetry.addData("Vel", "%.0f / %.0f", r.shooter.getVelocity(), currentTargetVel);

        if (Shooter.USE_ANGLE_OVERRIDE) {
            telemetry.addData("⚠ OVERRIDE", "%.3f", Shooter.MANUAL_ANGLE_OVERRIDE);
        } else {
            telemetry.addData("Angle", "%.3f", currentTargetAngle);
        }

        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}