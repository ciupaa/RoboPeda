package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**
 * BLUE ALLIANCE CAMERA TeleOp - PIPELINE 1
 *
 * STARTING HEADING: Always 0° (wherever robot faces at start)
 * TARGET HEADING: -130° (from starting position)
 * TRIGGERS: Rotate to -130° from start (DEAD CENTER)
 */
@TeleOp(name = "MainTeleOp_Blue_camera", group = "Competition")
public class MainTeleOp_Blue_camera extends OpMode {

    private Robot_camera r;
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    // HEADING PID - TIGHTENED FOR DEAD CENTER ACCURACY
    private static final double HEADING_kP = 0.020;
    private static final double HEADING_kD = 0.005;
    private static final double HEADING_MIN_POWER = 0.04;
    private static final double HEADING_MAX_POWER = 0.35;
    private static final double HEADING_TOLERANCE = 0.5;
    private static final double HEADING_DEADBAND = 0.3;

    // BLUE: Target -130° from starting position (0°)
    private static final double BLUE_TARGET_HEADING = -130.0;

    // Heading alignment state
    private double lastHeadingError = 0.0;
    private final ElapsedTime headingPidTimer = new ElapsedTime();

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;
    private boolean alignedRumbled = false;

    // MANUAL SHOT PRESET
    private static final double MANUAL_ANGLE = 0.70;
    private static final double MANUAL_VELOCITY = 1400;
    private static final double IDLE_PRESET = 1.0;

    // PULSE FEEDING
    private static final double FEED_PULSE_MS = 120;
    private static final double FEED_PAUSE_MS = 100;
    private static final double VELOCITY_DROP_THRESHOLD = 150;
    private static final double MIN_RECOVERY_TIME_MS = 400;

    // VELOCITY THRESHOLDS
    private static final double AUTO_VELOCITY_THRESHOLD = 80;
    private static final double MANUAL_VELOCITY_THRESHOLD = 120;

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
        r = new Robot_camera(hardwareMap, Alliance.BLUE); // Pipeline 1
        r.shooter.block();
        headingPidTimer.reset();

        // ALWAYS START AT 0° - wherever the robot is facing
        r.f.setStartingPose(new Pose(0, 0, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("Pipeline", "1");
        telemetry.addData("Starting Heading", "0°");
        telemetry.addData("Target Heading", "%.0f°", BLUE_TARGET_HEADING);
        telemetry.addData("Tolerance", "±%.1f°", HEADING_TOLERANCE);
        telemetry.addLine("Triggers = Rotate to -130° (DEAD CENTER)");
    }

    @Override
    public void loop() {
        r.periodic();

        // =========================================================
        // DRIVING
        // =========================================================
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;

        boolean autoAlignActive = gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5;

        double finalRx;

        if (autoAlignActive) {
            double currentHeadingRad = r.f.getPose().getHeading();
            double currentHeadingDeg = Math.toDegrees(currentHeadingRad);

            double error = normalizeAngle(BLUE_TARGET_HEADING - currentHeadingDeg);

            double dt = headingPidTimer.seconds();
            headingPidTimer.reset();
            if (dt < 0.001) dt = 0.001;

            // Apply deadband
            if (Math.abs(error) < HEADING_DEADBAND) {
                error = 0;
            }

            double derivative = (error - lastHeadingError) / dt;

            double alignPower = (HEADING_kP * error) + (HEADING_kD * derivative);

            // Apply minimum power only outside deadband
            if (Math.abs(error) > HEADING_DEADBAND) {
                if (alignPower > 0) {
                    alignPower = Math.max(alignPower, HEADING_MIN_POWER);
                } else if (alignPower < 0) {
                    alignPower = Math.min(alignPower, -HEADING_MIN_POWER);
                }
            }

            alignPower = Math.max(-HEADING_MAX_POWER, Math.min(HEADING_MAX_POWER, alignPower));

            finalRx = -alignPower;

            lastHeadingError = error;

            if (Math.abs(error) < HEADING_TOLERANCE) {
                if (!alignedRumbled) {
                    gamepad1.rumble(100);
                    alignedRumbled = true;
                }
            } else {
                alignedRumbled = false;
            }

        } else {
            finalRx = gamepad1.right_stick_x;
            lastHeadingError = 0;
            alignedRumbled = false;
            headingPidTimer.reset();
        }

        r.drive.driveRobotCentric(x, y, finalRx);

        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000);
            endgameRumbled = true;
        }

        // =========================================================
        // SHOOTING
        // =========================================================
        boolean autoShot = gamepad1.right_bumper;
        boolean manualShot = gamepad1.left_bumper;
        boolean shootHeld = autoShot || manualShot;

        if (shootHeld) {
            jamRumbled = false;

            if (autoShot && r.limelight.hasTarget()) {
                double distanceCm = r.limelight.getDistanceToTarget();
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distanceCm);

                currentTargetAngle = config.angle;
                currentTargetVel = config.velocity;
            } else {
                currentTargetAngle = MANUAL_ANGLE;
                currentTargetVel = MANUAL_VELOCITY;
            }

            r.shooter.setAngle(currentTargetAngle);
            r.shooter.unblock();
            r.shooter.launcher.setVelocity(currentTargetVel);

            double currentVel = r.shooter.getVelocity();
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

        else if (gamepad1.y) {
            shootState = ShootState.WAIT_SPINUP;
            jamRumbled = false;
            r.shooter.setAngle(IDLE_PRESET);
            r.intake.outtakeSlow();
            r.shooter.stop();
            r.shooter.block();
        }

        else {
            shootState = ShootState.WAIT_SPINUP;
            jamRumbled = false;
            r.shooter.setAngle(IDLE_PRESET);
            r.shooter.stop();
            r.intake.stop();
            r.shooter.block();
        }

        // =========================================================
        // TELEMETRY
        // =========================================================

        double currentHeadingDeg = Math.toDegrees(r.f.getPose().getHeading());
        double headingError = normalizeAngle(BLUE_TARGET_HEADING - currentHeadingDeg);

        telemetry.addLine("=== BLUE (PIPELINE 1) ===");
        telemetry.addData("Current", "%.2f°", currentHeadingDeg);
        telemetry.addData("Target", "%.2f°", BLUE_TARGET_HEADING);
        telemetry.addData("Error", "%.2f°", headingError);
        telemetry.addData("Align", autoAlignActive ? "ACTIVE" : "Manual");

        if (Math.abs(headingError) < HEADING_TOLERANCE) {
            telemetry.addData("✓", "ALIGNED (±%.1f°)", HEADING_TOLERANCE);
        }

        telemetry.addLine("");
        if (r.limelight.hasTarget()) {
            double distance = r.limelight.getDistanceToTarget();

            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Distance", "%.1f cm", distance);
            telemetry.addData("TX", "%.1f°", r.limelight.getTx());
            telemetry.addData("TY", "%.1f°", r.limelight.getTy());

            if (autoShot) {
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distance);
                telemetry.addData("Auto Angle", "%.2f", config.angle);
                telemetry.addData("Auto Vel", "%.0f", config.velocity);
            }
        } else {
            telemetry.addData("Camera", "No Target");
        }

        telemetry.addLine("");
        if (autoShot) {
            telemetry.addData("Mode", "AUTO");
        } else if (manualShot) {
            telemetry.addData("Mode", "MANUAL");
        } else {
            telemetry.addData("Mode", "IDLE");
        }
        telemetry.addData("State", shootState);
        telemetry.addData("Vel", "%.0f / %.0f", r.shooter.getVelocity(), currentTargetVel);

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is si ei smecheri");

        telemetry.update();
    }

    private double normalizeAngle(double angleDegrees) {
        while (angleDegrees > 180) angleDegrees -= 360;
        while (angleDegrees < -180) angleDegrees += 360;
        return angleDegrees;
    }
}