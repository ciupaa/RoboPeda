package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * ShooterCalculator_camera - HYBRID APPROACH (COMPETITION READY)
 *
 * ANGLE: Lookup table with linear interpolation (from field testing)
 * VELOCITY: Quadratic regression from Desmos WITH 8% SAFETY MARGIN
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

    // === ANGLE LOOKUP TABLE - YOUR EXACT MEASURED VALUES ===
    public static double[] ANGLE_DISTANCES = {
            97, 110, 130, 140, 160, 180, 200, 210, 223, 235, 250, 270, 290, 305, 318, 327
    };

    public static double[] ANGLE_VALUES = {
            0.84, 0.83, 0.80, 0.80, 0.80, 0.80, 0.78, 0.78, 0.76, 0.77, 0.77, 0.77, 0.77, 0.77, 0.79, 0.76
    };

    // === VELOCITY QUADRATIC COEFFICIENTS (from Desmos) ===
    // Equation: y = 0.0101799x² - 2.26609x + 1263.85611
    private static final double VEL_A = 0.0101799;
    private static final double VEL_B = -2.26609;
    private static final double VEL_C = 1263.85611;

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
     * Calculate Servo Angle - LOOKUP TABLE METHOD
     * Uses your exact tested values with linear interpolation
     * 100% accurate to your field calibration
     */
    public static double calculateAngle(double distanceCm) {
        // Below minimum distance - use first value
        if (distanceCm <= ANGLE_DISTANCES[0]) {
            return ANGLE_VALUES[0];
        }

        // Above maximum distance - use last value
        if (distanceCm >= ANGLE_DISTANCES[ANGLE_DISTANCES.length - 1]) {
            return ANGLE_VALUES[ANGLE_VALUES.length - 1];
        }

        // Linear interpolation between closest two points
        for (int i = 0; i < ANGLE_DISTANCES.length - 1; i++) {
            if (distanceCm >= ANGLE_DISTANCES[i] && distanceCm <= ANGLE_DISTANCES[i + 1]) {
                double x1 = ANGLE_DISTANCES[i];
                double y1 = ANGLE_VALUES[i];
                double x2 = ANGLE_DISTANCES[i + 1];
                double y2 = ANGLE_VALUES[i + 1];

                // Linear interpolation formula
                double slope = (y2 - y1) / (x2 - x1);
                double interpolatedAngle = y1 + (distanceCm - x1) * slope;

                // Safety clamp
                return Range.clip(interpolatedAngle, MIN_ANGLE, MAX_ANGLE);
            }
        }

        // Fallback (should never reach here)
        return 0.78;
    }

    /**
     * Calculate Motor Velocity - QUADRATIC REGRESSION WITH SAFETY MARGIN
     *
     * Equation: y = 0.0101799x² - 2.26609x + 1263.85611
     *
     * APPLIES 8% SAFETY MARGIN to ensure consistent scoring even with:
     * - Battery voltage drop (13V → 12V = -8% power)
     * - Motor friction variations
     * - Temperature effects
     * - Mechanical wear during matches
     */
    public static double calculateVelocity(double distanceCm) {
        double x = distanceCm;

        // Calculate base velocity from quadratic regression
        double baseVelocity =
                VEL_A * Math.pow(x, 2)
                        + VEL_B * x
                        + VEL_C;

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