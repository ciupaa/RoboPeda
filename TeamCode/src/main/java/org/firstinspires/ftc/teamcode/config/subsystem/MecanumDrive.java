package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * FILE: MecanumDrive.java
 * PURPOSE: Handles manual driving logic (TeleOp).
 * MODE: Robot Centric (Forward is always Robot's Front).
 */
public class MecanumDrive {
    private final DcMotor fl, fr, bl, br;
    private final IMU imu;

    public MecanumDrive(HardwareMap hardwareMap) {
        // Initialize Motors
        fl = hardwareMap.get(DcMotor.class, "fata_stanga");
        fr = hardwareMap.get(DcMotor.class, "fata_dreapta");
        bl = hardwareMap.get(DcMotor.class, "spate_stanga");
        br = hardwareMap.get(DcMotor.class, "spate_dreapta");

        // Set Directions (Reverse Left Side)
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // Set Brake Mode (Stops quickly)
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU (Gyroscope)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    /**
     * driveRobotCentric: Calculates mecanum wheel powers.
     * @param x Strafe (Left/Right)
     * @param y Forward/Backward
     * @param rx Rotation
     */
    public void driveRobotCentric(double x, double y, double rx) {
        // Denominator ensures no motor is commanded > 1.0 (100%)
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        fl.setPower((y + x + rx) / denominator);
        bl.setPower((y - x + rx) / denominator);
        fr.setPower((y - x - rx) / denominator);
        br.setPower((y + x - rx) / denominator);
    }

    public void resetHeading() { imu.resetYaw(); }
}