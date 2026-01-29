package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

/**
 * SHOOTER TUNER - Live Tuning from FTC Dashboard
 *
 * Controls everything from FTC Dashboard (Panels):
 * - Shooter angle
 * - Shooter velocity
 * - Blocker position
 * - Intake on/off
 *
 * NO GAMEPAD NEEDED - Pure Dashboard Control!
 *
 * Dashboard URL: http://192.168.43.1:8080
 */
@Configurable
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends OpMode {

    private Robot_camera r;

    // === DASHBOARD CONTROLS ===

    // Shooter Controls
    public static boolean SHOOTER_ENABLED = false;
    public static double SHOOTER_VELOCITY = 1200;
    public static double SHOOTER_ANGLE = 0.70;

    // Blocker Controls
    public static boolean BLOCKER_OPEN = false;

    // Intake Controls
    public static boolean INTAKE_ENABLED = false;
    public static boolean INTAKE_REVERSE = false;

    // Distance Display (Read-only)
    private double currentDistance = 0;
    private double calculatedAngle = 0;
    private double calculatedVelocity = 0;

    @Override
    public void init() {
        r = new Robot_camera(hardwareMap, Alliance.BLUE);

        telemetry.addLine("=== SHOOTER TUNER INITIALIZED ===");
        telemetry.addLine("Open FTC Dashboard to control:");
        telemetry.addLine("http://192.168.43.1:8080");
        telemetry.addLine("");
        telemetry.addLine("Variables > ShooterTuner");
        telemetry.update();
    }

    @Override
    public void loop() {
        r.periodic();

        // ========== SHOOTER CONTROL ==========
        if (SHOOTER_ENABLED) {
            r.shooter.launcher.setVelocity(SHOOTER_VELOCITY);
        } else {
            r.shooter.stop();
        }

        // Always set angle (even when shooter is off)
        r.shooter.setAngle(SHOOTER_ANGLE);

        // ========== BLOCKER CONTROL ==========
        if (BLOCKER_OPEN) {
            r.shooter.unblock();
        } else {
            r.shooter.block();
        }

        // ========== INTAKE CONTROL ==========
        if (INTAKE_ENABLED) {
            if (INTAKE_REVERSE) {
                r.intake.outtakeSlow();
            } else {
                r.intake.intake();
            }
        } else {
            r.intake.stop();
        }

        // ========== LIMELIGHT DATA ==========
        if (r.limelight.hasTarget()) {
            currentDistance = r.limelight.getDistanceToTarget();

            // Show what the equations would calculate
            calculatedAngle = org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera.calculateAngle(currentDistance);
            calculatedVelocity = org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera.calculateVelocity(currentDistance);
        } else {
            currentDistance = -1;
            calculatedAngle = 0;
            calculatedVelocity = 0;
        }

        // ========== TELEMETRY ==========
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addLine("=== SHOOTER TUNER ===");
        telemetry.addLine("Control from Dashboard!");
        telemetry.addLine("");

        telemetry.addLine("=== CURRENT SETTINGS ===");
        telemetry.addData("Shooter", SHOOTER_ENABLED ? "RUNNING" : "OFF");
        telemetry.addData("Velocity", "%.0f (actual: %.0f)", SHOOTER_VELOCITY, r.shooter.getVelocity());
        telemetry.addData("Angle", "%.3f", SHOOTER_ANGLE);
        telemetry.addData("Blocker", BLOCKER_OPEN ? "OPEN" : "CLOSED");
        telemetry.addData("Intake", INTAKE_ENABLED ? (INTAKE_REVERSE ? "REVERSE" : "FORWARD") : "OFF");

        telemetry.addLine("");
        telemetry.addLine("=== LIMELIGHT DATA ===");
        if (r.limelight.hasTarget()) {
            telemetry.addData("Distance", "%.1f cm", currentDistance);
            telemetry.addData("TX", "%.2f deg", r.limelight.getTx());
            telemetry.addData("TY", "%.2f deg", r.limelight.getTy());

            telemetry.addLine("");
            telemetry.addLine("=== EQUATION PREDICTIONS ===");
            telemetry.addData("Predicted Angle", "%.3f", calculatedAngle);
            telemetry.addData("Predicted Velocity", "%.0f", calculatedVelocity);

            telemetry.addLine("");
            telemetry.addLine("=== COMPARISON ===");
            telemetry.addData("Angle Diff", "%.3f", SHOOTER_ANGLE - calculatedAngle);
            telemetry.addData("Velocity Diff", "%.0f", SHOOTER_VELOCITY - calculatedVelocity);
        } else {
            telemetry.addData("Camera", "No Target");
        }

        telemetry.addLine("");
        telemetry.addLine("Open Dashboard to tune!");
        telemetry.update();
    }
}