package org.firstinspires.ftc.teamcode.config.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * ShooterCalculator_camera - REGRESSION EQUATION
 * * Uses polynomial regression calculated from Desmos to determine
 * exact angle and velocity for any given distance.
 * * Equations provided:
 * Velocity: y = -0.00299656x^2 + 3.01616x + 893.4823
 * Angle:    y = -(2.27572 * 10^-8)x^3 + 0.0000248578x^2 - 0.00731359x + 1.23132
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
     * Equation: y = -(2.27572e-8)x^3 + 0.0000248578x^2 - 0.00731359x + 1.23132
     */
    public static double calculateAngle(double distanceCm) {
        double x = distanceCm;

        double calculatedAngle =
                -(2.27572e-8) * Math.pow(x, 3)
                        + 0.0000248578 * Math.pow(x, 2)
                        - 0.00731359 * x
                        + 1.23132;

        // Safety clamp to ensure we never send an invalid signal to the servo
        // Assuming your servo valid range is roughly 0.0 to 1.0
        return Range.clip(calculatedAngle, 0.0, 1.0);
    }

    /**
     * Calculates Motor Velocity (RPM/Ticks) using Quadratic Regression
     * Equation: y = -0.00299656x^2 + 3.01616x + 893.4823
     */
    public static double calculateVelocity(double distanceCm) {
        double x = distanceCm;

        double calculatedVel =
                -0.00299656 * Math.pow(x, 2)
                        + 3.01616 * x
                        + 893.4823;

        // Safety clamp: Velocity cannot be negative, and shouldn't exceed motor max (e.g., 2000)
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