package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@TeleOp(name = "Professional TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        // Initialize the Robot object
        robot = new Robot(hardwareMap, Alliance.BLUE);
        telemetry.addData("Status", "Robot Initialized");
    }

    @Override
    public void loop() {
        // 1. UPDATE ROBOT LOGIC (Must be called first)
        robot.periodic();

        // 2. DRIVER CONTROLS (GamePad 1)
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        // Uses the Field Centric Drive from MecanumDrive.java
        robot.drive.driveFieldCentric(x, y, rx);

        if (gamepad1.options) {
            robot.drive.resetHeading();
        }

        // 3. SHOOTER CONTROLS
        if (gamepad1.right_bumper) {
            // Spin up
            robot.shooter.spinUp();

            // Only feed if at speed
            if (robot.shooter.isReady()) {
                robot.shooter.feed();
            } else {
                robot.shooter.stopFeeders();
            }
        } else if (gamepad1.left_bumper) {
            // Reverse Everything
            robot.shooter.reverse();
        } else {
            // Stop Everything
            robot.shooter.stop();
        }

        // 4. TELEMETRY
        telemetry.addData("Velocity", robot.shooter.getVelocity());
        telemetry.addData("Ready?", robot.shooter.isReady());
        telemetry.update();
    }
}