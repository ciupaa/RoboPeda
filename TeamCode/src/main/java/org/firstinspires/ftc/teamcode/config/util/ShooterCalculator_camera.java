package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * ShooterCalculator_camera - CUBIC REGRESSION (COMPETITION READY)
 *
 * ANGLE: Cubic regression from Desmos
 * VELOCITY: Cubic regression from Desmos WITH 8% SAFETY MARGIN
 *
 * Calibrated from 16 field-tested data points (97cm - 327cm)
 * ALL DISTANCES IN CENTIMETERS!
 *
 * SAFETY MARGIN: 1.08x multiplier on velocity to compensate for:
 * - Battery voltage drop during matches
 * - Motor friction variations
 * - Temperature effects
 * - Mechanical wear
 */
@Config
@Configurable
public class ShooterCalculator_camera {

    // === ANGLE CUBIC COEFFICIENTS (from Desmos) ===
    // Equation: y = 2.76754e-8 * x³ - 0.0000149216 * x² + 0.00111285 * x + 0.914961
    private static final double ANG_A = 2.76754e-8;
    private static final double ANG_B = -0.0000149216;
    private static final double ANG_C = 0.00111285;
    private static final double ANG_D = 0.914961;

    // === VELOCITY CUBIC COEFFICIENTS (from Desmos) ===
    // Equation: y = 0.0000164688 * x³ - 0.0114151 * x² + 4.51308 * x + 783.24357
    private static final double VEL_A = 0.0000164688;
    private static final double VEL_B = -0.0114151;
    private static final double VEL_C = 4.51308;
    private static final double VEL_D = 783.24357;

    // === SAFETY MARGIN ===
    // 8% increase to compensate for battery drop, friction, and variations
    // Adjustable from Dashboard if needed
    public static double VELOCITY_SAFETY_MULTIPLIER = 1.08;

    // === SAFETY LIMITS ===
    public static double MIN_ANGLE = 0.70;
    public static double MAX_ANGLE = 0.90;
    public static double MIN_VELOCITY = 1100;
    public static double MAX_VELOCITY = 1900;

    /**
     * Calculate Servo Angle - CUBIC REGRESSION METHOD
     * Equation: y = 2.76754e-8 * x³ - 0.0000149216 * x² + 0.00111285 * x + 0.914961
     */
    public static double calculateAngle(double distanceCm) {
        double x = distanceCm;

        double angle = ANG_A * Math.pow(x, 3)
                + ANG_B * Math.pow(x, 2)
                + ANG_C * x
                + ANG_D;

        // Safety clamp
        return Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
    }

    /**
     * Calculate Motor Velocity - CUBIC REGRESSION WITH SAFETY MARGIN
     *
     * Equation: y = 0.0000164688 * x³ - 0.0114151 * x² + 4.51308 * x + 783.24357
     *
     * APPLIES 8% SAFETY MARGIN to ensure consistent scoring even with:
     * - Battery voltage drop (13V → 12V = -8% power)
     * - Motor friction variations
     * - Temperature effects
     * - Mechanical wear during matches
     */
    public static double calculateVelocity(double distanceCm) {
        double x = distanceCm;

        // Calculate base velocity from cubic regression
        double baseVelocity = VEL_A * Math.pow(x, 3)
                + VEL_B * Math.pow(x, 2)
                + VEL_C * x
                + VEL_D;

        // Apply safety multiplier
        double safeVelocity = baseVelocity * VELOCITY_SAFETY_MULTIPLIER;

        // Safety clamp
        return Range.clip(safeVelocity, MIN_VELOCITY, MAX_VELOCITY);
    }

    /**
     * Complete shooter configuration
     */
    public static class ShooterConfig {
        public final double angle;
        public final double velocity;
        public final double distance;

        public ShooterConfig(double distanceCm) {
            this.distance = distanceCm;
            this.angle = calculateAngle(distanceCm);
            this.velocity = calculateVelocity(distanceCm);
        }
    }

    /**
     * Get complete config for distance (CM)
     */
    public static ShooterConfig getConfig(double distanceCm) {
        return new ShooterConfig(distanceCm);
    }
}