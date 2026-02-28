package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * ShooterCalculator_camera - HYBRID SYSTEM (COMPETITION READY)
 *
 * ANGLE:    Lookup table + linear interpolation from 12 field-tested points
 *           This is perfectly accurate between measured points - no regression error.
 *           R^2 = 1.0 at every calibration point.
 *
 * VELOCITY: Quartic regression from Desmos (R^2 = 0.9937) WITH 8% SAFETY MARGIN
 *           Equation: y = 1.9497e-8*x^4 - 0.0000119132*x^3 + 0.00228041*x^2 + 1.88038*x + 949.69847
 *
 * Calibration data (12 field-tested points, 115cm-330cm):
 *   115->0.88 | 135->0.86 | 148->0.86 | 167->0.83 | 190->0.74 | 200->0.77
 *   220->0.77 | 240->0.71 | 250->0.69 | 280->0.67 | 308->0.65 | 330->0.65
 *
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

    // =========================================================
    // ANGLE LOOKUP TABLE - 12 field-tested calibration points
    // DISTANCES must be strictly ascending!
    // =========================================================
    private static final double[] ANGLE_DISTANCES = {
            115.0,  // Point 1
            135.0,  // Point 2
            148.0,  // Point 3
            167.0,  // Point 4
            190.0,  // Point 5
            200.0,  // Point 6
            220.0,  // Point 7
            240.0,  // Point 8
            250.0,  // Point 9
            280.0,  // Point 10
            308.0,  // Point 11
            330.0   // Point 12
    };

    private static final double[] ANGLE_VALUES = {
            0.88,   // 115 cm
            0.86,   // 135 cm
            0.86,   // 148 cm
            0.83,   // 167 cm
            0.74,   // 190 cm  <- dip in data, lookup table preserves this exactly
            0.77,   // 200 cm
            0.77,   // 220 cm
            0.71,   // 240 cm
            0.69,   // 250 cm
            0.67,   // 280 cm
            0.65,   // 308 cm
            0.65    // 330 cm
    };

    // =========================================================
    // VELOCITY QUARTIC COEFFICIENTS (from Desmos, R^2 = 0.9937)
    // Equation: y = 1.9497e-8*x^4 - 0.0000119132*x^3 + 0.00228041*x^2 + 1.88038*x + 949.69847
    // =========================================================
    private static final double VEL_A = 1.9497e-8;
    private static final double VEL_B = -0.0000119132;
    private static final double VEL_C = 0.00228041;
    private static final double VEL_D = 1.88038;
    private static final double VEL_E = 949.69847;

    // =========================================================
    // SAFETY MARGIN
    // 8% increase to compensate for battery drop, friction, variations
    // Adjustable from Dashboard if needed during competition
    // =========================================================
    public static double VELOCITY_SAFETY_MULTIPLIER = 1.08;

    // =========================================================
    // SAFETY CLAMPS
    // =========================================================
    public static double MIN_ANGLE    = 0.65;
    public static double MAX_ANGLE    = 0.90;
    public static double MIN_VELOCITY = 1100.0;
    public static double MAX_VELOCITY = 1900.0;

    /**
     * Calculate Servo Angle - LOOKUP TABLE + LINEAR INTERPOLATION
     *
     * Exactly matches every calibration point.
     * Linearly interpolates between measured points.
     * Clamps to edge values outside the calibrated range.
     */
    public static double calculateAngle(double distanceCm) {
        int n = ANGLE_DISTANCES.length;

        // Below minimum range -> use closest known value
        if (distanceCm <= ANGLE_DISTANCES[0]) {
            return ANGLE_VALUES[0];
        }

        // Above maximum range -> use closest known value
        if (distanceCm >= ANGLE_DISTANCES[n - 1]) {
            return ANGLE_VALUES[n - 1];
        }

        // Find the two surrounding points and linearly interpolate
        for (int i = 0; i < n - 1; i++) {
            if (distanceCm >= ANGLE_DISTANCES[i] && distanceCm <= ANGLE_DISTANCES[i + 1]) {
                double x0 = ANGLE_DISTANCES[i];
                double x1 = ANGLE_DISTANCES[i + 1];
                double y0 = ANGLE_VALUES[i];
                double y1 = ANGLE_VALUES[i + 1];

                // Linear interpolation: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
                double angle = y0 + (distanceCm - x0) * (y1 - y0) / (x1 - x0);
                return Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
            }
        }

        // Fallback (should never reach here)
        return Range.clip(ANGLE_VALUES[n - 1], MIN_ANGLE, MAX_ANGLE);
    }

    /**
     * Calculate Motor Velocity - QUARTIC REGRESSION WITH SAFETY MARGIN
     *
     * Equation: y = 1.9497e-8*x^4 - 0.0000119132*x^3 + 0.00228041*x^2 + 1.88038*x + 949.69847
     * R^2 = 0.9937 - excellent fit across 115cm-330cm range.
     *
     * APPLIES 8% SAFETY MARGIN to ensure consistent scoring under:
     * - Battery voltage drop (13V -> 12V ~= -8% power)
     * - Motor friction variations
     * - Temperature effects
     * - Mechanical wear during matches
     */
    public static double calculateVelocity(double distanceCm) {
        double x = distanceCm;

        // Quartic regression
        double baseVelocity = VEL_A * Math.pow(x, 4)
                + VEL_B * Math.pow(x, 3)
                + VEL_C * Math.pow(x, 2)
                + VEL_D * x
                + VEL_E;

        // Apply safety multiplier
        double safeVelocity = baseVelocity * VELOCITY_SAFETY_MULTIPLIER;

        return Range.clip(safeVelocity, MIN_VELOCITY, MAX_VELOCITY);
    }

    /**
     * Complete shooter configuration for a given distance
     */
    public static class ShooterConfig {
        public final double angle;
        public final double velocity;
        public final double distance;

        public ShooterConfig(double distanceCm) {
            this.distance = distanceCm;
            this.angle    = calculateAngle(distanceCm);
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