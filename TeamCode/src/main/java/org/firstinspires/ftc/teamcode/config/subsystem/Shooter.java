package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * FILE: Shooter.java
 * PURPOSE: Controls the Flywheel (Launcher) and Angle Adjustment.
 * FEATURES: PIDF Velocity Control, Servo Positioning, Feeder Control.
 */
@Config // Allows editing variables live in Dashboard
public class Shooter extends SubsystemBase {
    private final DcMotorEx launcher;
    private final Servo angle;
    private CRServo leftFeeder, rightFeeder;

    // --- TUNING VALUES ---
    // P = Proportional (Power boost based on error)
    // F = Feedforward (Base power to overcome friction)
    public static double P = 800;
    public static double I = 0;
    public static double D = 0;
    public static double F = 13;

    public static double TARGET_VELOCITY = 1550;

    public Shooter(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        angle = hardwareMap.get(Servo.class, "angle");

        // Try-Catch protects against crashing if feeders aren't in Config
        try {
            leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
            rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
            rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {}

        // Brake Mode: Motor stops instantly when power is 0
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run Using Encoder: Necessary for Velocity (RPM) control
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        updatePIDF();
    }

    // Applies new PIDF values (Call if changed in Dashboard)
    public void updatePIDF() {
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F));
    }

    @Override
    public void periodic() {
        updatePIDF();
    }

    // --- ACTIONS ---
    public void setAngle(double pos) { angle.setPosition(pos); }
    public double getAngle() { return angle.getPosition(); }

    public void spinHigh() { launcher.setVelocity(TARGET_VELOCITY); }
    public void spinLow() { launcher.setVelocity(1200); }

    public void reverse() {
        launcher.setVelocity(-1125);
        if(leftFeeder != null) { leftFeeder.setPower(-1.0); rightFeeder.setPower(-1.0); }
    }

    public void stop() {
        launcher.setVelocity(0);
        stopFeeders();
    }

    public void feed() {
        if(leftFeeder != null) { leftFeeder.setPower(1.0); rightFeeder.setPower(1.0); }
    }

    public void stopFeeders() {
        if(leftFeeder != null) { leftFeeder.setPower(0); rightFeeder.setPower(0); }
    }

    public double getVelocity() { return launcher.getVelocity(); }
}