package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;

/**
 * ShooterCalculator_camera - Calculates shooter settings from distance
 *
 * HOW TO TUNE:
 * 1. Test at 3-4 distances (e.g., 36", 60", 84", 108")
 * 2. Find angle and velocity that score consistently
 * 3. Update arrays below
 * 4. System auto-interpolates between points
 */
@Config
public class ShooterCalculator_camera {

    // === LOOKUP TABLE - TUNE THESE ===

    /**
     * Test distances (inches) - MUST be ascending!
     */
    public static double[] DISTANCES = {
            30.0,   // Very close
            60.0,   // Medium close
            90.0,   // Medium far
            120.0   // Far
    };

    /**
     * Servo angles at each distance (0.0 to 1.0)
     * Lower = shoots farther
     */
    public static double[] ANGLES = {
            0.80,   // Close = low angle
            0.70,   // Medium close
            0.60,   // Medium far
            0.50    // Far = high angle
    };

    /**
     * Motor velocities at each distance (RPM)
     */
    public static double[] VELOCITIES = {
            1200,   // Close = slower
            1350,   // Medium close
            1500,   // Medium far
            1650    // Far = faster
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