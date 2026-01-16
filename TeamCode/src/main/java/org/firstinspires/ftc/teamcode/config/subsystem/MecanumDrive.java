package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    // --- FIELD CENTRIC (Old) ---
    public void driveFieldCentric(double x, double y, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        fl.setPower((rotY + rotX + rx) / denominator);
        bl.setPower((rotY - rotX + rx) / denominator);
        fr.setPower((rotY - rotX - rx) / denominator);
        br.setPower((rotY + rotX - rx) / denominator);
    }

    // --- ROBOT CENTRIC (New) ---
    public void driveRobotCentric(double x, double y, double rx) {
        // No Gyro Math - Just pure inputs
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        fl.setPower((y + x + rx) / denominator);
        bl.setPower((y - x + rx) / denominator);
        fr.setPower((y - x - rx) / denominator);
        br.setPower((y + x - rx) / denominator);
    }

    public void resetHeading() { imu.resetYaw(); }
}