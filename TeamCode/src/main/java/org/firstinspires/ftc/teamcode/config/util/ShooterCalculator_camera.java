package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;

/**
 * ShooterCalculator_camera - REAL CALIBRATED DISTANCES
 *
 * Based on your actual shooting positions:
 * - CLOSE: 109 cm → 0.70 angle, 1400 velocity
 * - MID: 121 cm → 0.68 angle, 1475 velocity
 * - FAR: 669 cm → 0.65 angle, 1550 velocity
 *
 * ALL DISTANCES IN CENTIMETERS!
 */
@Config
public class ShooterCalculator_camera {

    // === LOOKUP TABLE - REAL DISTANCES ===

    /**
     * Your actual shooting distances (CM) - MUST be ascending!
     *
     * Measured from testing:
     * - 109 cm: Close shot (left bumper preset)
     * - 121 cm: Mid-close shot (needs slight right rotation)
     * - 669 cm: Far shot (right bumper preset)
     */
    public static double[] DISTANCES = {
            109.0,   // CLOSE: Left bumper position
            121.0,   // MID: Mid-close (rotate right)
            300.0,   // MEDIUM: Interpolated
            669.0    // FAR: Right bumper position
    };

    /**
     * Servo angles - Based on YOUR working values!
     * 0.70 = close (left bumper)
     * 0.65 = far (right bumper)
     */
    public static double[] ANGLES = {
            0.70,   // Close shot (109 cm)
            0.67,   // Mid-close (121 cm)
            0.67,   // Medium (300 cm) - interpolated
            0.64    // Far shot (669 cm)
    };

    /**
     * Motor velocities - INCREASED FOR LEFT BUMPER!
     * 1400 = close (left bumper - INCREASED from 1200 for more power!)
     * 1550 = far (right bumper)
     */
    public static double[] VELOCITIES = {
            1200,   // Close shot (109 cm) - INCREASED!
            1200,   // Mid-close (121 cm) - interpolated
            1550,   // Medium (300 cm) - interpolated
            1580    // Far shot (669 cm)
    };

    // === FALLBACK VALUES ===
    public static double DEFAULT_ANGLE = 0.7;
    public static double DEFAULT_VELOCITY = 1200;

    /**
     * Calculate shooter angle from distance (CM)
     */
    public static double calculateAngle(double distanceCm) {
        if (distanceCm < 0) return DEFAULT_ANGLE;

        if (distanceCm <= DISTANCES[0]) {
            return ANGLES[0];
        }

        if (distanceCm >= DISTANCES[DISTANCES.length - 1]) {
            return ANGLES[ANGLES.length - 1];
        }

        for (int i = 0; i < DISTANCES.length - 1; i++) {
            if (distanceCm >= DISTANCES[i] && distanceCm <= DISTANCES[i + 1]) {
                return interpolate(
                        DISTANCES[i], ANGLES[i],
                        DISTANCES[i + 1], ANGLES[i + 1],
                        distanceCm
                );
            }
        }

        return DEFAULT_ANGLE;
    }

    /**
     * Calculate shooter velocity from distance (CM)
     */
    public static double calculateVelocity(double distanceCm) {
        if (distanceCm < 0) return DEFAULT_VELOCITY;

        if (distanceCm <= DISTANCES[0]) {
            return VELOCITIES[0];
        }

        if (distanceCm >= DISTANCES[DISTANCES.length - 1]) {
            return VELOCITIES[VELOCITIES.length - 1];
        }

        for (int i = 0; i < DISTANCES.length - 1; i++) {
            if (distanceCm >= DISTANCES[i] && distanceCm <= DISTANCES[i + 1]) {
                return interpolate(
                        DISTANCES[i], VELOCITIES[i],
                        DISTANCES[i + 1], VELOCITIES[i + 1],
                        distanceCm
                );
            }
        }

        return DEFAULT_VELOCITY;
    }

    /**
     * Linear interpolation
     */
    private static double interpolate(double x1, double y1, double x2, double y2, double x) {
        double slope = (y2 - y1) / (x2 - x1);
        return y1 + (x - x1) * slope;
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