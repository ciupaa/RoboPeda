package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Shooter extends SubsystemBase {
    private final DcMotorEx launcher;
    private final CRServo leftFeeder, rightFeeder;

    // Tuning (Editable in Dashboard)
    public static double TARGET_VELOCITY = 1125;
    public static double REVERSE_VELOCITY = -1125;
    public static double TOLERANCE = 50;

    public Shooter(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void spinUp() {
        launcher.setVelocity(TARGET_VELOCITY);
    }

    public void stop() {
        launcher.setVelocity(0);
        stopFeeders();
    }

    public void feed() {
        leftFeeder.setPower(1.0);
        rightFeeder.setPower(1.0);
    }

    public void stopFeeders() {
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    public void reverse() {
        launcher.setVelocity(REVERSE_VELOCITY);
        leftFeeder.setPower(-1.0);
        rightFeeder.setPower(-1.0);
    }

    public boolean isReady() {
        return Math.abs(launcher.getVelocity() - TARGET_VELOCITY) < TOLERANCE;
    }

    public double getVelocity() {
        return launcher.getVelocity();
    }
}