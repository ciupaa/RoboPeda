package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

/**
 * Standard TeleOp - FASTER CLOSE RANGE + AUTO-ALIGN READY
 *
 * Left bumper (1200 velocity) spins up faster!
 * Auto-align code present but inactive (no camera in Robot class)
 * Hold BOTH triggers to activate auto-align (when camera is available)
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    private Robot r;
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    // AUTO-ALIGN PID (ready for when camera is added)
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

    // YOUR WORKING PRESETS
    private static final double HIGH_PRESET = 0.65;  // Far shot - 1550 velocity
    private static final double LOW_PRESET = 0.70;   // Close shot - 1200 velocity
    private static final double IDLE_PRESET = 1.0;   // Safe position

    // PULSE FEEDING STRATEGY
    private static final double FEED_PULSE_MS = 120;
    private static final double FEED_PAUSE_MS = 100;
    private static final double VELOCITY_DROP_THRESHOLD = 150;
    private static final double MIN_RECOVERY_TIME_MS = 400;

    // VELOCITY READY THRESHOLD - DIFFERENT FOR HIGH vs LOW
    private static final double HIGH_VELOCITY_THRESHOLD = 80;  // Far shot needs to be closer
    private static final double LOW_VELOCITY_THRESHOLD = 120;  // Close shot can start sooner!

    private enum ShootState {
        WAIT_SPINUP,
        PULSE_FEED,
        PULSE_PAUSE,
        WAIT_RECOVERY
    }
    private ShootState shootState = ShootState.WAIT_SPINUP;

    private double currentTargetVel = 0.0;
    private double lastStableVelocity = 0.0;

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        r.shooter.block();
        pidTimer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "FASTER Close Range");
        telemetry.addLine("Hold BOTH triggers for auto-align");
    }

    @Override
    public void loop() {
        r.periodic();

        // =========================================================
        // DRIVING WITH AUTO-ALIGN TRIGGER
        // =========================================================
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;

        // Auto-align trigger (both triggers pressed)
        boolean autoAlignActive = gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5;

        double finalRx;

        // NOTE: This code is ready for camera integration
        // Currently falls through to manual control since Robot class has no camera
        if (autoAlignActive) {
            // If camera were available:
            // - Would use PID to align to target
            // - Would rumble when aligned
            // For now, just use manual control
            finalRx = -gamepad1.right_stick_x;
            lastAlignError = 0;
            alignedRumbled = false;
        } else {
            // Manual rotation control
            finalRx = -gamepad1.right_stick_x
                    + (gamepad1.left_trigger * 0.3)   // Fine tune left
                    - (gamepad1.right_trigger * 0.3);  // Fine tune right
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
        // SHOOTING LOGIC - FASTER FOR CLOSE RANGE
        // =========================================================
        boolean highHeld = gamepad1.right_bumper;
        boolean lowHeld = gamepad1.left_bumper;
        boolean shootHeld = highHeld || lowHeld;

        if (shootHeld) {
            jamRumbled = false;

            double targetAngle = highHeld ? HIGH_PRESET : LOW_PRESET;
            currentTargetVel = highHeld ? 1550 : 1200;

            // Set servos immediately
            r.shooter.setAngle(targetAngle);
            r.shooter.unblock();

            // Spin launcher
            if (highHeld) r.shooter.spinHigh();
            else r.shooter.spinLow();

            double currentVel = r.shooter.getVelocity();

            // DIFFERENT THRESHOLD FOR HIGH vs LOW
            double velocityThreshold = highHeld ? HIGH_VELOCITY_THRESHOLD : LOW_VELOCITY_THRESHOLD;
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

        // INTAKE MODE (X button)
        else if (gamepad1.x) {
            shootState = ShootState.WAIT_SPINUP;

            r.shooter.setAngle(IDLE_PRESET);
            r.shooter.stop();
            r.shooter.block();
            r.intake.intake();

            // Jam detection
            double currentAmps = r.intake.getCurrentDraw();
            if (currentAmps > Intake.JAM_THRESHOLD) {
                if (!jamRumbled) {
                    gamepad1.rumble(500);
                    jamRumbled = true;
                }
                telemetry.addData("WARNING", "INTAKE JAM! (%.1f A)", currentAmps);
            } else {
                jamRumbled = false;
            }
        }

        // OUTTAKE MODE (Y button)
        else if (gamepad1.y) {
            shootState = ShootState.WAIT_SPINUP;
            jamRumbled = false;

            r.shooter.setAngle(IDLE_PRESET);
            r.intake.outtakeSlow();
            r.shooter.stop();
            r.shooter.block();
        }

        // IDLE MODE
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

        // Show auto-align status
        if (autoAlignActive) {
            telemetry.addData("Auto-Align", "TRIGGERED (no camera)");
        }

        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Velocity", "%.0f / %.0f", r.shooter.getVelocity(), currentTargetVel);
        telemetry.addData("Vel Drop", "%.0f", lastStableVelocity - r.shooter.getVelocity());
        telemetry.addData("Feed Timer", "%.0f ms", feedTimer.milliseconds());
        telemetry.addData("Recovery", "%.0f ms", recoveryTimer.milliseconds());

        telemetry.addLine("---");
        telemetry.addData("Blocker", shootHeld ? "OPEN" : "CLOSED");
        if (shootHeld) {
            String mode = highHeld ? "HIGH (1550)" : "LOW (1200 FAST)";
            telemetry.addData("Mode", mode);
            telemetry.addData("Angle", highHeld ? "0.65" : "0.70");
        } else {
            telemetry.addData("Mode", "IDLE");
        }

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is si ei smecheri");

        telemetry.update();
    }
}