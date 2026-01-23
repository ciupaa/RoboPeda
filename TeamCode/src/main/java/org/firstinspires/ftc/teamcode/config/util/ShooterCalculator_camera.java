package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;

/**
 * ShooterCalculator_camera - CENTIMETERS + INCREASED LEFT BUMPER VELOCITY
 *
 * Based on your working presets:
 * - Right Bumper: 0.65 angle, 1550 velocity = FAR shot
 * - Left Bumper: 0.70 angle, 1400 velocity = CLOSE shot (INCREASED from 1200)
 *
 * ALL DISTANCES NOW IN CENTIMETERS!
 */
@Config
public class ShooterCalculator_camera {

    // === LOOKUP TABLE - IN CENTIMETERS ===

    /**
     * Test distances (CM) - MUST be ascending!
     *
     * Converted from inches to CM (1 inch = 2.54 cm):
     * - 40" = 101.6 cm
     * - 60" = 152.4 cm
     * - 80" = 203.2 cm
     * - 100" = 254.0 cm
     *
     * TODO: Measure actual distances in CM and update!
     */
    public static double[] DISTANCES = {
            100.0,   // CLOSE: Where left bumper (0.70, 1400) works
            150.0,   // MEDIUM CLOSE: Interpolate
            200.0,   // MEDIUM FAR: Interpolate
            250.0    // FAR: Where right bumper (0.65, 1550) works
    };

    /**
     * Servo angles - Based on YOUR working values!
     * 0.70 = close (left bumper)
     * 0.65 = far (right bumper)
     */
    public static double[] ANGLES = {
            0.70,   // Close shot (left bumper preset)
            0.68,   // Interpolated
            0.66,   // Interpolated
            0.65    // Far shot (right bumper preset)
    };

    /**
     * Motor velocities - INCREASED FOR LEFT BUMPER!
     * 1400 = close (left bumper - INCREASED from 1200 for more power!)
     * 1550 = far (right bumper)
     */
    public static double[] VELOCITIES = {
            1400,   // Close shot (left bumper preset) - INCREASED!
            1450,   // Interpolated: 1400 + (1550-1400)/3
            1500,   // Interpolated: 1400 + 2*(1550-1400)/3
            1550    // Far shot (right bumper preset)
    };

    // === FALLBACK VALUES ===
    public static double DEFAULT_ANGLE = 0.65;
    public static double DEFAULT_VELOCITY = 1550;

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