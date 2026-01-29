package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * MecanumDrive - FIXED INVERSIONS
 *
 * All controls are now inverted to match expected behavior:
 * - Forward stick → Robot moves forward
 * - Right stick → Robot strafes right
 * - Right rotation → Robot rotates right
 */
public class MecanumDrive {
    private final DcMotor fl, fr, bl, br;
    private final IMU imu;

    public MecanumDrive(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, "fata_stanga");
        fr = hardwareMap.get(DcMotor.class, "fata_dreapta");
        bl = hardwareMap.get(DcMotor.class, "spate_stanga");
        br = hardwareMap.get(DcMotor.class, "spate_dreapta");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    /**
     * FIXED: All inputs are now inverted to match expected behavior
     */
    public void driveRobotCentric(double x, double y, double rx) {
        // INVERT ALL INPUTS
        x = -x;   // Fix: Right strafe was going left
        y = -y;   // Fix: Forward was going backward
        rx = -rx; // Fix: Rotation was reversed

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        fl.setPower((y - x + rx) / denominator);
        bl.setPower((y + x + rx) / denominator);
        fr.setPower((y + x - rx) / denominator);
        br.setPower((y - x - rx) / denominator);
    }
}