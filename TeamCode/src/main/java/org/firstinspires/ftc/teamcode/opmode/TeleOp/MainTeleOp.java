package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

/**
 * Standard TeleOp - NO AUTO-ALIGN
 * Just manual driving and shooting
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    private Robot r;
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;

    // SHOOTING PRESETS
    private static final double HIGH_PRESET = 0.65;
    private static final double LOW_PRESET = 0.70;
    private static final double IDLE_PRESET = 1.0;

    // PULSE FEEDING STRATEGY
    private static final double FEED_PULSE_MS = 120;
    private static final double FEED_PAUSE_MS = 100;
    private static final double VELOCITY_DROP_THRESHOLD = 150;
    private static final double MIN_RECOVERY_TIME_MS = 400;

    // VELOCITY READY THRESHOLD
    private static final double HIGH_VELOCITY_THRESHOLD = 80;
    private static final double LOW_VELOCITY_THRESHOLD = 120;

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

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Manual Control");
        telemetry.addLine("No auto-align");
    }

    @Override
    public void loop() {
        r.periodic();

        // =========================================================
        // MANUAL DRIVING ONLY
        // =========================================================
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;

        // Just manual rotation - triggers included in rotation control
        double finalRx = -gamepad1.right_stick_x
                + (gamepad1.left_trigger * 0.3)   // Fine tune left
                - (gamepad1.right_trigger * 0.3);  // Fine tune right

        r.drive.driveRobotCentric(x, y, finalRx);

        // Endgame rumble
        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000);
            endgameRumbled = true;
        }

        // =========================================================
        // SHOOTING LOGIC
        // =========================================================
        boolean highHeld = gamepad1.right_bumper;
        boolean lowHeld = gamepad1.left_bumper;
        boolean shootHeld = highHeld || lowHeld;

        if (shootHeld) {
            jamRumbled = false;

            double targetAngle = highHeld ? HIGH_PRESET : LOW_PRESET;
            currentTargetVel = highHeld ? 1550 : 1200;

            r.shooter.setAngle(targetAngle);
            r.shooter.unblock();

            if (highHeld) r.shooter.spinHigh();
            else r.shooter.spinLow();

            double currentVel = r.shooter.getVelocity();

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
                    }
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
        }

        // =========================================================
        // TELEMETRY
        // =========================================================

        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Velocity", "%.0f / %.0f", r.shooter.getVelocity(), currentTargetVel);

        telemetry.addLine("---");
        if (shootHeld) {
            String mode = highHeld ? "HIGH (1550)" : "LOW (1200)";
            telemetry.addData("Mode", mode);
        } else {
            telemetry.addData("Mode", "IDLE");
        }

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is si ei smecheri");

        telemetry.update();
    }
}