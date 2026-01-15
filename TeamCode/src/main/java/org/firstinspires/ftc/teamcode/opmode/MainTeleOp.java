package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {
    private Robot r;

    // --- LOGIC VARIABLES ---
    private ElapsedTime servoTimer = new ElapsedTime();
    private double calculatedWaitTime = 0;
    private boolean isShooting = false;

    // RUMBLE FLAGS
    private boolean endgameRumbled = false;

    // MICRO-ADJUSTMENT VARIABLES
    private boolean lastUp = false, lastDown = false;
    // These replace the hardcoded numbers so you can change them live
    private double highPreset = 0.9;
    private double lowPreset = 0.7;
    private double idlePreset = 1.0;

    // CONSTANTS
    private final double SERVO_SPEED_FACTOR = 600.0; // Higher = Safer Wait
    private final double VELOCITY_TOLERANCE = 50;

    @Override
    public void init() { r = new Robot(hardwareMap, Alliance.BLUE); }

    @Override
    public void loop() {
        r.periodic();

        // 1. DRIVING
        r.drive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

        // IMU RESET + RUMBLE
        if (gamepad1.options) {
            r.drive.resetHeading();
            gamepad1.rumble(500); // Rumble for 0.5s to confirm reset
        }

        // 2. ENDGAME RUMBLE (2 Minutes)
        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000); // Rumble for 2 seconds
            endgameRumbled = true;
        }

        // --- MICRO-ADJUSTMENTS (Context Aware) ---
        // 0.1 Increment per click
        if (gamepad1.dpad_up && !lastUp) {
            if (gamepad1.right_bumper) highPreset += 0.1;
            else if (gamepad1.left_bumper) lowPreset += 0.1;
            else idlePreset += 0.1;
        }
        if (gamepad1.dpad_down && !lastDown) {
            if (gamepad1.right_bumper) highPreset -= 0.1;
            else if (gamepad1.left_bumper) lowPreset -= 0.1;
            else idlePreset -= 0.1;
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // Safety Clamps (0.0 to 1.0)
        highPreset = Math.max(0, Math.min(1, highPreset));
        lowPreset = Math.max(0, Math.min(1, lowPreset));
        idlePreset = Math.max(0, Math.min(1, idlePreset));


        // 3. SHOOTING LOGIC (Bumpers)
        if (gamepad1.right_bumper || gamepad1.left_bumper) {

            // Use the adjustable presets instead of hardcoded numbers
            double targetAngle = gamepad1.right_bumper ? highPreset : lowPreset;
            double targetVel = gamepad1.right_bumper ? 1550 : 1200;

            // -- INITIALIZE SEQUENCE --
            if (!isShooting) {
                // Calculate how long to wait based on distance to move
                double currentPos = r.shooter.getAngle();
                double distance = Math.abs(targetAngle - currentPos);

                calculatedWaitTime = distance * SERVO_SPEED_FACTOR;

                r.shooter.setAngle(targetAngle);
                servoTimer.reset();
                isShooting = true;
            } else {
                // Keep updating angle in case you adjust Dpad WHILE shooting
                r.shooter.setAngle(targetAngle);
            }

            // -- EXECUTE SEQUENCE --
            if (servoTimer.milliseconds() >= calculatedWaitTime) {
                // Servo arrived, Spin Motors
                if (gamepad1.right_bumper) r.shooter.spinHigh();
                else r.shooter.spinLow();

                // Check Velocity
                double currentVel = r.shooter.getVelocity();
                if (currentVel >= (targetVel - VELOCITY_TOLERANCE)) {
                    // Feed (Forward)
                    r.intake.intake();
                } else {
                    // Speed drop -> Stop feeding
                    r.intake.stop();
                }

            } else {
                // Waiting for servo...
                r.shooter.stop();
                r.intake.stop();
            }

        }
        // 4. INTAKE MODE (X)
        else if (gamepad1.x) {
            isShooting = false;
            r.shooter.setAngle(idlePreset); // Use adjusted idle
            r.intake.intake();       // Feed Forward
            r.shooter.reverse();     // Launcher Backward
        }
        // 5. IDLE / STANDARD POSITION
        else {
            isShooting = false;

            // Standard position when nothing is requested
            r.shooter.setAngle(idlePreset); // Use adjusted idle

            r.shooter.stop();
            r.intake.stop();

            // Manual Unjam
            if (gamepad1.dpad_left) r.shooter.reverse();
        }

        telemetry.addData("State", isShooting ? "SHOOTING" : "IDLE/STANDARD");
        telemetry.addData("Match Time", "%.1f", getRuntime());

        // Show the adjustable values so you know what they are
        telemetry.addData("High Preset (RB+Dpad)", "%.2f", highPreset);
        telemetry.addData("Low Preset (LB+Dpad)", "%.2f", lowPreset);
        telemetry.addData("Idle Preset (Dpad)", "%.2f", idlePreset);

        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is niste slabi");
        telemetry.update();
    }
}