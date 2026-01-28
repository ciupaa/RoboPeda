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
 * BLUE ALLIANCE CAMERA TeleOp
 *
 * Controls:
 * - RIGHT BUMPER: Auto-align heading + auto-shoot (distance-based)
 * - LEFT BUMPER: Manual close shot (0.70 angle, 1200 velocity)
 * - OPTIONS: Reset heading to 0°
 * - X: Intake
 * - Y: Outtake
 *
 * All distances in CENTIMETERS
 */
@Configurable
@TeleOp(name = "MainTeleOp_Blue_camera", group = "Competition")
public class MainTeleOp_Blue_camera extends OpMode {

    private Robot_camera r;
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    // AUTO-ALIGN HEADING PID (Pinpoint-based)
    public static double HEADING_kP = 0.020;
    public static double HEADING_kD = 0.005;
    public static double HEADING_MIN_POWER = 0.04;
    public static double HEADING_MAX_POWER = 0.35;
    public static double HEADING_TOLERANCE = 0.5;
    public static double HEADING_DEADBAND = 0.3;

    // TARGET HEADINGS (degrees)
    public static double BLUE_TARGET_HEADING = -130.0;

    // PID state
    private double lastHeadingError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;
    private boolean alignedRumbled = false;

    // MANUAL SHOT PRESET (left bumper - CLOSE RANGE)
    private static final double MANUAL_ANGLE = 0.70;
    private static final double MANUAL_VELOCITY = 1200;
    private static final double IDLE_PRESET = 1.0;

    // PULSE FEEDING STRATEGY
    private static final double FEED_PULSE_MS = 120;
    private static final double FEED_PAUSE_MS = 100;
    private static final double VELOCITY_DROP_THRESHOLD = 150;
    private static final double MIN_RECOVERY_TIME_MS = 400;

    // VELOCITY READY THRESHOLDS
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
        r = new Robot_camera(hardwareMap, Alliance.BLUE);
        r.shooter.block();
        pidTimer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("Pipeline", "1 (Blue Tags)");
        telemetry.addLine("Right = AUTO, Left = MANUAL");
        telemetry.addLine("OPTIONS = Reset Heading");
    }

    @Override
    public void loop() {
        r.periodic();

        // =========================================================
        // HEADING RESET (OPTIONS BUTTON)
        // =========================================================
        if (gamepad1.options) {
            Pose current = r.f.getPose();
            r.f.setStartingPose(new Pose(current.getX(), current.getY(), 0));
            gamepad1.rumble(200);
        }

        // =========================================================
        // DRIVING WITH AUTO-ALIGN (BLUE - STANDARD)
        // =========================================================
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;

        boolean rightBumper = gamepad1.right_bumper;
        boolean leftBumper = gamepad1.left_bumper;

        // Auto-align active when RIGHT BUMPER + has target
        boolean autoAlignActive = rightBumper && r.limelight.hasTarget();

        double finalRx;

        if (autoAlignActive) {
            // BLUE ALLIANCE: Standard heading alignment
            double currentHeadingDeg = Math.toDegrees(r.f.getPose().getHeading());
            double targetHeadingDeg = BLUE_TARGET_HEADING;

            double headingError = normalizeAngle(targetHeadingDeg - currentHeadingDeg);

            // Apply deadband
            if (Math.abs(headingError) < HEADING_DEADBAND) {
                headingError = 0;
            }

            double dt = pidTimer.seconds();
            pidTimer.reset();

            double derivative = (headingError - lastHeadingError) / dt;

            double alignPower = (HEADING_kP * headingError) + (HEADING_kD * derivative);

            // Minimum power enforcement
            if (Math.abs(alignPower) > 0.01) {
                if (alignPower > 0) {
                    alignPower = Math.max(alignPower, HEADING_MIN_POWER);
                } else {
                    alignPower = Math.min(alignPower, -HEADING_MIN_POWER);
                }
            }

            alignPower = Math.max(-HEADING_MAX_POWER, Math.min(HEADING_MAX_POWER, alignPower));

            // BLUE: Standard direction
            finalRx = -alignPower;

            lastHeadingError = headingError;

            // Rumble when aligned
            if (Math.abs(headingError) < HEADING_TOLERANCE) {
                if (!alignedRumbled) {
                    gamepad1.rumble(100);
                    alignedRumbled = true;
                }
            } else {
                alignedRumbled = false;
            }

        } else {
            finalRx = -gamepad1.right_stick_x;
            lastHeadingError = 0;
            alignedRumbled = false;
        }

        r.drive.driveRobotCentric(x, y, finalRx);

        // Endgame rumble
        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000);
            endgameRumbled = true;
        }

        // =========================================================
        // SHOOTING
        // =========================================================
        boolean shootHeld = rightBumper || leftBumper;

        if (shootHeld) {
            jamRumbled = false;

            // RIGHT BUMPER: Auto-shoot (distance-based)
            if (rightBumper && r.limelight.hasTarget()) {
                double distanceCm = r.limelight.getDistanceToTarget();
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distanceCm);

                currentTargetAngle = config.angle;
                currentTargetVel = config.velocity;
            }
            // LEFT BUMPER: Manual close shot
            else {
                currentTargetAngle = MANUAL_ANGLE;
                currentTargetVel = MANUAL_VELOCITY;
            }

            r.shooter.setAngle(currentTargetAngle);
            r.shooter.unblock();
            r.shooter.launcher.setVelocity(currentTargetVel);

            double currentVel = r.shooter.getVelocity();
            double velocityThreshold = leftBumper ? MANUAL_VELOCITY_THRESHOLD : AUTO_VELOCITY_THRESHOLD;
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

        double currentHeadingDeg = Math.toDegrees(r.f.getPose().getHeading());
        double headingError = normalizeAngle(BLUE_TARGET_HEADING - currentHeadingDeg);

        telemetry.addLine("=== BLUE (PIPELINE 1) ===");
        telemetry.addData("Current Heading", "%.2f°", currentHeadingDeg);
        telemetry.addData("Target Heading", "%.2f°", BLUE_TARGET_HEADING);

        if (!autoAlignActive && rightBumper) {
            telemetry.addData("Status", "No Limelight Target!");
        } else if (!autoAlignActive) {
            telemetry.addData("Preview Error", "%.2f°", headingError);
        } else {
            telemetry.addData("Align Error", "%.2f°", headingError);
            telemetry.addData("Status", "AUTO-ALIGNING");
        }

        if (Math.abs(headingError) < HEADING_TOLERANCE && autoAlignActive) {
            telemetry.addData("✓", "ALIGNED");
        }

        telemetry.addLine("");
        if (r.limelight.hasTarget()) {
            double distance = r.limelight.getDistanceToTarget();

            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Distance", "%.1f cm", distance);
            telemetry.addData("TX", "%.1f°", r.limelight.getTx());

            if (rightBumper) {
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distance);
                telemetry.addData("Auto Angle", "%.2f", config.angle);
                telemetry.addData("Auto Vel", "%.0f", config.velocity);
            }
        } else {
            telemetry.addData("Camera", "No Target");
        }

        telemetry.addLine("");
        telemetry.addLine("=== SHOOTER ===");
        if (rightBumper) {
            telemetry.addData("Mode", "AUTO (Right Bumper)");
        } else if (leftBumper) {
            telemetry.addData("Mode", "MANUAL (Left Bumper)");
        } else {
            telemetry.addData("Mode", "IDLE");
        }
        telemetry.addData("State", shootState);
        telemetry.addData("Vel", "%.0f / %.0f", r.shooter.getVelocity(), currentTargetVel);

        // ANGLE OVERRIDE STATUS
        if (Shooter.USE_ANGLE_OVERRIDE) {
            telemetry.addData("⚠ ANGLE OVERRIDE", "ACTIVE");
            telemetry.addData("Override Angle", "%.3f", Shooter.MANUAL_ANGLE_OVERRIDE);
        } else {
            telemetry.addData("Angle", "%.3f (Auto)", currentTargetAngle);
        }

        telemetry.addLine("");
        telemetry.addData("OPTIONS", "Reset Heading");

        telemetry.addLine("Ciupa BOSS");
        telemetry.update();
    }

    /**
     * Normalize angle to [-180, 180]
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}