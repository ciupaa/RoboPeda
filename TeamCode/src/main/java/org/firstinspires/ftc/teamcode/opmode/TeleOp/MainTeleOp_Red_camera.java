package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

@Config
@TeleOp(name = "MainTeleOp_Red_camera", group = "Competition")
public class MainTeleOp_Red_camera extends OpMode {

    private Robot_camera r;
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    public static double TX_kP = 0.018;
    public static double TX_kD = 0.004;
    public static double TX_MIN_POWER = 0.10;
    public static double TX_MAX_POWER = 0.45;
    public static double TX_TOLERANCE = 1.0;

    private double lastTxError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;
    private boolean alignedRumbled = false;

    // --- TOGGLE STATE VARIABLES ---
    private boolean intakeActive = false;
    private boolean lastX = false;

    // BLOCKER STATE TRACKING
    private boolean lastShootHeld = false;

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
    }

    @Override
    public void loop() {
        r.periodic();
        telemetry.addLine("Acest robot a fost programar de Cristi, Alex si Ciupa, 3 fraieri");
        telemetry.addLine(" ");
        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Mario e cel mai slab(bun) driver");
        telemetry.addLine("Cristi e cel mai autist(extraordinar) coach");
        telemetry.addLine("Acest robot a fost programar de Cristi, Alex si Ciupa, 3 fraieri");

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        boolean rightBumper = gamepad1.right_bumper;
        boolean leftBumper = gamepad1.left_bumper;

        // --- INTAKE TOGGLE LOGIC ---
        if (gamepad1.x && !lastX) {
            intakeActive = !intakeActive;
        }
        lastX = gamepad1.x;

        // Safety: If we start shooting or resetting, force intake OFF
        if (rightBumper || leftBumper || gamepad1.b || gamepad1.y) {
            intakeActive = false;
        }

        // BLOCKER CONTROL - Only toggle on button press/release
        boolean shootHeld = rightBumper || leftBumper;
        if (shootHeld && !lastShootHeld) {
            // Button just pressed - OPEN blocker
            r.shooter.unblock();
        } else if (!shootHeld && lastShootHeld) {
            // Button just released - CLOSE blocker
            r.shooter.block();
        }
        lastShootHeld = shootHeld;

        // --- STATE MACHINE ---

        if (rightBumper && r.limelight.hasTarget()) {
            double tx = r.limelight.getTx();
            double dt = pidTimer.seconds();
            pidTimer.reset();

            double derivative = (tx - lastTxError) / dt;
            double alignPower = (TX_kP * tx) + (TX_kD * derivative);

            if (Math.abs(alignPower) > 0.01) {
                alignPower = alignPower > 0
                        ? Math.max(alignPower, TX_MIN_POWER)
                        : Math.min(alignPower, -TX_MIN_POWER);
            }

            alignPower = Math.max(-TX_MAX_POWER, Math.min(TX_MAX_POWER, alignPower));
            lastTxError = tx;

            boolean isAligned = Math.abs(tx) < TX_TOLERANCE;

            if (isAligned) {
                autoShootState = AutoShootState.ALIGNED_SHOOTING;
                r.drive.driveRobotCentric(x, y, 0);

                if (shootState == ShootState.WAIT_SPINUP) {
                    double distanceCm = r.limelight.getDistanceToTarget();

                    if (distanceCm > 0) {
                        ShooterCalculator_camera.ShooterConfig config =
                                ShooterCalculator_camera.getConfig(distanceCm);
                        currentTargetAngle = config.angle;
                        currentTargetVel = config.velocity;
                    } else {
                        currentTargetAngle = MANUAL_ANGLE;
                        currentTargetVel = MANUAL_VELOCITY;
                    }
                }

                executeShootingSequence(AUTO_VELOCITY_THRESHOLD);

                if (!alignedRumbled) {
                    gamepad1.rumble(100);
                    alignedRumbled = true;
                }
            } else {
                autoShootState = AutoShootState.ALIGNING;
                alignedRumbled = false;

                r.drive.driveRobotCentric(x, y, -alignPower);

                r.shooter.stop();
                r.intake.stop();
                shootState = ShootState.WAIT_SPINUP;
            }
        }

        else if (leftBumper) {
            autoShootState = AutoShootState.IDLE;
            alignedRumbled = false;
            lastTxError = 0;

            r.drive.driveRobotCentric(x, y, rx);

            currentTargetAngle = MANUAL_ANGLE;
            currentTargetVel = MANUAL_VELOCITY;

            executeShootingSequence(MANUAL_VELOCITY_THRESHOLD);
        }

        else if (intakeActive) {
            autoShootState = AutoShootState.IDLE;
            shootState = ShootState.WAIT_SPINUP;
            alignedRumbled = false;
            lastTxError = 0;

            r.drive.driveRobotCentric(x, y, rx);

            r.shooter.setAngle(IDLE_PRESET);
            r.shooter.stop();
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

        else if (gamepad1.y) {
            autoShootState = AutoShootState.IDLE;
            shootState = ShootState.WAIT_SPINUP;
            jamRumbled = false;
            alignedRumbled = false;
            lastTxError = 0;

            r.drive.driveRobotCentric(x, y, rx);

            r.shooter.setAngle(IDLE_PRESET);
            r.intake.outtakeSlow();
            r.shooter.stop();
        }

        else {
            autoShootState = AutoShootState.IDLE;
            shootState = ShootState.WAIT_SPINUP;
            jamRumbled = false;
            alignedRumbled = false;
            lastTxError = 0;

            r.drive.driveRobotCentric(x, y, rx);

            r.shooter.setAngle(IDLE_PRESET);
            r.shooter.stop();
            r.intake.stop();

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
        // BLOCKER IS ALREADY CONTROLLED BY BUTTON PRESS/RELEASE - DON'T COMMAND IT HERE
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

        telemetry.addLine("=== RED (TAG 24) ===");
        if (r.limelight.hasTarget()) {
            double distance = r.limelight.getDistanceToTarget();
            telemetry.addLine("=== MEGATAG POSITION ===");
            telemetry.addData("Robot X", "%.3f m", r.limelight.getLastBotposeX());
            telemetry.addData("Robot Y", "%.3f m", r.limelight.getLastBotposeY());
            telemetry.addData("Robot Z", "%.3f m", r.limelight.getLastBotposeZ());
            telemetry.addLine("");
            telemetry.addData("TX", "%.2f deg", r.limelight.getTx());
            telemetry.addData("TY", "%.2f deg", r.limelight.getTy());
            telemetry.addLine("");
            if (distance > 0) {
                telemetry.addData("Distance", "%.1f cm (%.2f m)", distance, distance/100.0);
            } else {
                telemetry.addData("Distance", "INVALID - Tune target coords!");
            }
            if (autoShootState == AutoShootState.ALIGNING) {
                telemetry.addData("Status", "ALIGNING");
            } else if (autoShootState == AutoShootState.ALIGNED_SHOOTING) {
                telemetry.addData("Status", "ALIGNED");
            }
        } else {
            telemetry.addData("Camera", "No Tag 24");
        }
        telemetry.addLine("");
        telemetry.addData("Mode", gamepad1.right_bumper ? "AUTO" :
                gamepad1.left_bumper ? "MANUAL" : "IDLE");
        telemetry.addData("Intake", intakeActive ? "ON (Toggle)" : "OFF");
        telemetry.addData("Vel", "%.0f / %.0f", r.shooter.getVelocity(), currentTargetVel);
        telemetry.addData("Angle", "%.3f", currentTargetAngle);

        telemetry.update();
    }
}