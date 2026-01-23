package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;

/**
 * ShooterCalculator_camera - TUNED with your working presets
 *
 * Based on your MainTeleOp.java:
 * - Right Bumper (HIGH_PRESET): 0.65 angle, 1550 velocity = FAR shot
 * - Left Bumper (LOW_PRESET): 0.70 angle, 1200 velocity = CLOSE shot
 *
 * Now you need to measure the ACTUAL DISTANCES these work at!
 */
@Config
public class ShooterCalculator_camera {

    // === LOOKUP TABLE - TUNE THESE BASED ON TESTING ===

    /**
     * Test distances (inches) - MUST be ascending!
     *
     * TODO: Measure these with tape measure:
     * 1. Stand at distance where LEFT BUMPER works perfectly
     * 2. Measure that distance → Update DISTANCES[0]
     * 3. Stand at distance where RIGHT BUMPER works perfectly
     * 4. Measure that distance → Update DISTANCES[3]
     * 5. Add 2 intermediate points
     */
    public static double[] DISTANCES = {
            40.0,   // CLOSE: Where left bumper (0.70, 1200) works
            60.0,   // MEDIUM CLOSE: Interpolate
            80.0,   // MEDIUM FAR: Interpolate
            100.0   // FAR: Where right bumper (0.65, 1550) works
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
     * Motor velocities - Based on YOUR working values!
     * 1200 = close (left bumper)
     * 1550 = far (right bumper)
     */
    public static double[] VELOCITIES = {
            1200,   // Close shot (left bumper preset)
            1283,   // Interpolated: 1200 + (1550-1200)/3
            1433,   // Interpolated: 1200 + 2*(1550-1200)/3
            1550    // Far shot (right bumper preset)
    };

    // === FALLBACK VALUES ===
    public static double DEFAULT_ANGLE = 0.65;
    public static double DEFAULT_VELOCITY = 1550;

    /**
     * Calculate shooter angle from distance
     */
    public static double calculateAngle(double distance) {
        if (distance < 0) return DEFAULT_ANGLE;

        if (distance <= DISTANCES[0]) {
            return ANGLES[0];
        }

        if (distance >= DISTANCES[DISTANCES.length - 1]) {
            return ANGLES[ANGLES.length - 1];
        }

        for (int i = 0; i < DISTANCES.length - 1; i++) {
            if (distance >= DISTANCES[i] && distance <= DISTANCES[i + 1]) {
                return interpolate(
                        DISTANCES[i], ANGLES[i],
                        DISTANCES[i + 1], ANGLES[i + 1],
                        distance
                );
            }
        }

        return DEFAULT_ANGLE;
    }

    /**
     * Calculate shooter velocity from distance
     */
    public static double calculateVelocity(double distance) {
        if (distance < 0) return DEFAULT_VELOCITY;

        if (distance <= DISTANCES[0]) {
            return VELOCITIES[0];
        }

        if (distance >= DISTANCES[DISTANCES.length - 1]) {
            return VELOCITIES[VELOCITIES.length - 1];
        }

        for (int i = 0; i < DISTANCES.length - 1; i++) {
            if (distance >= DISTANCES[i] && distance <= DISTANCES[i + 1]) {
                return interpolate(
                        DISTANCES[i], VELOCITIES[i],
                        DISTANCES[i + 1], VELOCITIES[i + 1],
                        distance
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

        public ShooterConfig(double distance) {
            this.distance = distance;
            this.angle = calculateAngle(distance);
            this.velocity = calculateVelocity(distance);
        }
    }

    /**
     * Get complete config for distance
     */
    public static ShooterConfig getConfig(double distance) {
        return new ShooterConfig(distance);
    }
}