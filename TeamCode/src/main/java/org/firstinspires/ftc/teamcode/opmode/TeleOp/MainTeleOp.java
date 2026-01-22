package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

/**
 * Standard TeleOp - No Limelight
 * Velocity-based ball feeding with manual shooter control
 *
 * CONTROLS:
 * - Left Stick: Drive (strafe/forward)
 * - Right Stick X: Rotate
 * - Left/Right Trigger: Fine rotation adjustment
 * - Right Bumper: High shot (far)
 * - Left Bumper: Low shot (close)
 * - X: Intake
 * - Y: Outtake
 * - D-Pad Left: Reverse launcher
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    private Robot r;
    private final ElapsedTime recoveryTimer = new ElapsedTime();

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;
    private boolean jamRumbled = false;

    // SHOOTER PRESETS (tune before match)
    private static final double HIGH_PRESET = 0.65;  // Far shot
    private static final double LOW_PRESET = 0.70;   // Close shot
    private static final double IDLE_PRESET = 1.0;   // Safe position

    // VELOCITY DETECTION CONSTANTS
    private static final double VELOCITY_DROP_THRESHOLD = 100;
    private static final double MIN_RECOVERY_TIME_MS = 80;
    private static final double VELOCITY_READY_THRESHOLD = 50;

    private enum ShootState { WAIT_SPINUP, FEEDING, WAIT_RECOVERY }
    private ShootState shootState = ShootState.WAIT_SPINUP;

    private double currentTargetVel = 0.0;
    private double lastStableVelocity = 0.0;

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        r.shooter.block();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Standard TeleOp");
    }

    @Override
    public void loop() {
        r.periodic();

        // =========================================================
        // DRIVING
        // =========================================================
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double finalRx = -gamepad1.right_stick_x
                + (gamepad1.left_trigger * 0.3)
                - (gamepad1.right_trigger * 0.3);

        r.drive.driveRobotCentric(x, y, finalRx);

        // Endgame rumble at 2 minutes
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

            // Set servos immediately
            r.shooter.setAngle(targetAngle);
            r.shooter.unblock();

            // Spin launcher
            if (highHeld) r.shooter.spinHigh();
            else r.shooter.spinLow();

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

                    // Detect velocity drop (ball launched)
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

            // Reverse launcher (D-pad left)
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
        telemetry.addData("Angle", shootHeld ? (highHeld ? "HIGH" : "LOW") : "IDLE");

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is si ei smecheri");

        telemetry.update();
    }
}