package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

/**
 * Standard TeleOp - NO CAMERA
 *
 * Controls:
 * - RIGHT BUMPER: High shot (0.65 angle, 1550 velocity)
 * - LEFT BUMPER: Low shot (0.84 angle, 1140 velocity) ← UPDATED!
 * - X: Intake
 * - Y: Outtake
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    private Robot r;
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;

    // YOUR WORKING PRESETS
    private static final double HIGH_PRESET = 0.8;   // Far shot - 1550 velocity
    private static final double LOW_PRESET = 0.84;    // Close shot - 1140 velocity ← UPDATED!
    private static final double IDLE_PRESET = 0.9;    // Safe position

    // PULSE FEEDING STRATEGY
    private static final double FEED_PULSE_MS = 120;
    private static final double FEED_PAUSE_MS = 100;
    private static final double VELOCITY_DROP_THRESHOLD = 150;
    private static final double MIN_RECOVERY_TIME_MS = 400;

    // VELOCITY READY THRESHOLD - DIFFERENT FOR HIGH vs LOW
    private static final double HIGH_VELOCITY_THRESHOLD = 80;
    private static final double LOW_VELOCITY_THRESHOLD = 120;

    // BLOCKER STATE TRACKING
    private boolean lastShootHeld = false;

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
        telemetry.addData("Mode", "UPDATED Close Range");
        telemetry.addLine("Left: 0.84 / 1140");
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

        // =========================================================
        // DRIVING
        // =========================================================
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        r.drive.driveRobotCentric(x, y, rx);

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

        // BLOCKER CONTROL - Only toggle on button press/release
        if (shootHeld && !lastShootHeld) {
            r.shooter.unblock();
        } else if (!shootHeld && lastShootHeld) {
            r.shooter.block();
        }
        lastShootHeld = shootHeld;

        if (shootHeld) {
            jamRumbled = false;

            double targetAngle = highHeld ? HIGH_PRESET : LOW_PRESET;
            currentTargetVel = highHeld ? 1630 : 1140;  // ← UPDATED!

            r.shooter.setAngle(targetAngle);

            if (highHeld) r.shooter.spinHigh();
            else r.shooter.launcher.setVelocity(1140);  // ← UPDATED!

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

        // INTAKE MODE (X button)
        else if (gamepad1.x) {
            shootState = ShootState.WAIT_SPINUP;
            r.shooter.setAngle(IDLE_PRESET);
            r.shooter.stop();
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
        }

        // IDLE MODE
        else {
            shootState = ShootState.WAIT_SPINUP;
            jamRumbled = false;
            r.shooter.setAngle(IDLE_PRESET);
            r.shooter.stop();
            r.intake.stop();

            if (gamepad1.dpad_left) {
                r.shooter.reverse();
            }
        }

        // =========================================================
        // TELEMETRY
        // =========================================================

        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Velocity", "%.0f / %.0f", r.shooter.getVelocity(), currentTargetVel);
        telemetry.addData("Vel Drop", "%.0f", lastStableVelocity - r.shooter.getVelocity());

        telemetry.addLine("---");
        telemetry.addData("Blocker", shootHeld ? "OPEN" : "CLOSED");
        if (shootHeld) {
            String mode = highHeld ? "HIGH (1550)" : "LOW (1140)";  // ← UPDATED!
            telemetry.addData("Mode", mode);
            telemetry.addData("Angle", highHeld ? "0.65" : "0.84");  // ← UPDATED!
        } else {
            telemetry.addData("Mode", "IDLE");
        }

        telemetry.update();
    }
}