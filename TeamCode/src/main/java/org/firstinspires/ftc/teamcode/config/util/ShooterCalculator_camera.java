package org.firstinspires.ftc.teamcode.config.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * ShooterCalculator_camera - REGRESSION EQUATION
 * * Uses polynomial regression calculated from Desmos to determine
 * exact angle and velocity for any given distance.
 * * Equations provided (UPDATED 4 - Anti-Jitter):
 * Velocity: y = 0.0000794842x^3 - 0.0414009x^2 + 8.04858x + 715.42863
 * Angle:    y = (2.76692e-8)x^3 - 0.0000164662x^2 + 0.0027993x + 0.555101
 */
@Configurable
public class ShooterCalculator_camera {

    /**
     * Get the full configuration for a specific distance
     * @param distanceCm Distance from the Limelight in Centimeters
     * @return ShooterConfig containing target angle and velocity
     */
    public static ShooterConfig getConfig(double distanceCm) {
        return new ShooterConfig(distanceCm);
    }

    /**
     * Calculates Servo Angle using Cubic Regression
     * Equation: y = (2.76692e-8)x^3 - 0.0000164662x^2 + 0.0027993x + 0.555101
     */
    public static double calculateAngle(double distanceCm) {
        double x = distanceCm;

        double calculatedAngle =
                (2.76692e-8) * Math.pow(x, 3)
                        - 0.0000164662 * Math.pow(x, 2)
                        + 0.0027993 * x
                        + 0.555101;

        // Safety clamp: 0.0 to 1.0 (Servo Position)
        return Range.clip(calculatedAngle, 0.0, 1.0);
    }

    /**
     * Calculates Motor Velocity (RPM/Ticks) using Cubic Regression
     * Equation: y = 0.0000794842x^3 - 0.0414009x^2 + 8.04858x + 715.42863
     */
    public static double calculateVelocity(double distanceCm) {
        double x = distanceCm;

        double calculatedVel =
                0.0000794842 * Math.pow(x, 3)
                        - 0.0414009 * Math.pow(x, 2)
                        + 8.04858 * x
                        + 715.42863;

        // Safety clamp: 0 to 2500
        return Range.clip(calculatedVel, 0.0, 2500.0);
    }

    /**
     * Complete shooter configuration object
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
}