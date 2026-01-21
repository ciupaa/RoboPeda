package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * FILE: Shooter.java
 * PURPOSE: Controls the Flywheel (Launcher) and the Blocker Arm.
 * FEATURES:
 * - PIDF Velocity Control for consistent shots.
 * - Servo Positioning for aiming.
 * - Blocker Arm ("Gate") to control when rings enter the flywheel.
 */
@Config // Allows editing variables live in Dashboard
public class Shooter extends SubsystemBase {
    private final DcMotorEx launcher;
    private final Servo angle;
    // NEW: Blocker Servo to stop rings from entering prematurely
    private final Servo blocker;

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

        // NEW: Initialize Blocker Servo and set to Closed (0.0)
        // Position 0 = Down (Blocking)
        // Position 1 = Up (Shooting)
        blocker = hardwareMap.get(Servo.class, "test_block_servo");
        blocker.setPosition(0.0);

        // Brake Mode: CHANGED TO FLOAT so it coasts on inertia (Saves Battery)
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Encoder Mode: Essential for maintaining specific Velocity (RPM)
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
        // Live Tuning: Updates PIDF if you change it in Dashboard while running
        updatePIDF();
    }

    // --- ACTIONS ---
    public void setAngle(double pos) { angle.setPosition(pos); }
    public double getAngle() { return angle.getPosition(); }

    // NEW: Blocker Methods
    // "Block" puts the arm DOWN to stop intake items from hitting the wheel
    public void block() { blocker.setPosition(0.0); }

    // "Unblock" lifts the arm UP to let items shoot
    public void unblock() { blocker.setPosition(1.0); }

    public void spinHigh() { launcher.setVelocity(TARGET_VELOCITY); }
    public void spinLow() { launcher.setVelocity(1200); }

    // Manual reverse if something gets stuck
    public void reverse() {
        launcher.setVelocity(-1125);
    }

    public void stop() {
        // CHANGED: Set Power to 0 to let it coast (Float).
        // Velocity 0 would actively brake, wasting energy.
        launcher.setPower(0);
    }

    // Kept empty methods to preserve compatibility with other files if needed
    public void feed() {}
    public void stopFeeders() {}

    public double getVelocity() { return launcher.getVelocity(); }
}