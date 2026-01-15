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

    // CONSTANTS
    private final double INTAKE_ANGLE = 1.0;  // Standard Position (Down)
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
        // getRuntime() returns seconds since START was pressed
        if (getRuntime() >= 120 && !endgameRumbled) {
            gamepad1.rumble(2000); // Rumble for 2 seconds
            endgameRumbled = true; // Ensure it only happens once
        }

        // 3. SHOOTING LOGIC (Bumpers)
        if (gamepad1.right_bumper || gamepad1.left_bumper) {

            double targetAngle = gamepad1.right_bumper ? 0.9 : 0.7;
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
            r.shooter.setAngle(INTAKE_ANGLE); // Ensure Down
            r.intake.intake();       // Feed Forward
            r.shooter.reverse();     // Launcher Backward
        }
        // 5. IDLE / STANDARD POSITION
        else {
            isShooting = false;

            // Standard position when nothing is requested
            r.shooter.setAngle(INTAKE_ANGLE);

            r.shooter.stop();
            r.intake.stop();

            // Manual Unjam
            if (gamepad1.dpad_left) r.shooter.reverse();
        }

        telemetry.addData("State", isShooting ? "SHOOTING" : "IDLE/STANDARD");
        telemetry.addData("Match Time", "%.1f", getRuntime());
        telemetry.addLine("Ciupa BOSS");
        telemetry.addLine("Cristi si Mario is niste slabi");
        telemetry.update();
    }
}