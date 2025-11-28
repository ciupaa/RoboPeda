package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystem.MecanumDrive;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    private Robot r;

    @Override
    public void init() {
        // Initialize the Robot
        // NOTE: This initializes PedroPathing in the background.
        // Ensure your Constants.java motor names are fixed to prevent "leftFront" errors!
        r = new Robot(hardwareMap, Alliance.BLUE);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.addData("Drive", "Field Centric");
    }

    @Override
    public void loop() {
        // 1. UPDATE ROBOT LOGIC
        // This clears the Bulk Cache (optimizes speed) and updates subsystem PIDs
        r.periodic();

        // 2. DRIVER CONTROLS (Standard Mecanum Field-Centric)
        // We use r.drive directly to avoid PedroPathing drive logic in TeleOp
        double y = -gamepad1.left_stick_y; // Forward/Back
        double x = gamepad1.left_stick_x;  // Strafe
        double rx = -gamepad1.right_stick_x; // Turn

        // Drive the robot
        r.drive.driveFieldCentric(x, y, rx);

        // Reset Heading (Field Centric Fix)
        if (gamepad1.options) {
            r.drive.resetHeading();
            gamepad1.rumble(200); // Small feedback
        }

        // 3. SHOOTER CONTROLS
        if (gamepad1.right_bumper) {
            // FIRE SEQUENCE
            // A. Spin up the flywheel
            r.shooter.spinUp();

            // B. Only feed if the flywheel is at target velocity
            if (r.shooter.isReady()) {
                r.shooter.feed();
            } else {
                // If speed drops (due to a shot), stop feeding to let it recover
                r.shooter.stopFeeders();
            }
        }
        else if (gamepad1.left_bumper) {
            // UNJAM (Reverse Everything)
            r.shooter.reverse();
        }
        else {
            // IDLE (Stop Everything)
            r.shooter.stop();
        }

        // 4. TELEMETRY
        telemetry.addData("Shooter Velocity", "%.0f", r.shooter.getVelocity());
        telemetry.addData("Ready to Fire", r.shooter.isReady());
        telemetry.addData("Heading", "Reset with Options");
        telemetry.update();
    }
}