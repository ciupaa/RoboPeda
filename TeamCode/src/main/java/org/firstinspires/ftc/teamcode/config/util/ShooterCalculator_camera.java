package org.firstinspires.ftc.teamcode.config.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * ShooterCalculator_camera - REGRESSION EQUATION
 * * Uses polynomial regression calculated from Desmos to determine
 * exact angle and velocity for any given distance.
 * * Equations provided (UPDATED 2):
 * Velocity: y = -0.00116902x^3 + 0.953686x^2 - 190.72172x
 * Angle:    y = (1.5289e-7)x^3 - 0.000128837x^2 + 0.0287064x
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
     * Equation: y = (1.5289e-7)x^3 - 0.000128837x^2 + 0.0287064x
     */
    public static double calculateAngle(double distanceCm) {
        double x = distanceCm;

        double calculatedAngle =
                (1.5289e-7) * Math.pow(x, 3)
                        - 0.000128837 * Math.pow(x, 2)
                        + 0.0287064 * x;
        // + 0 (Intercept is 0)

        // Safety clamp: 0 to 180 degrees.
        // NOTE: If your servo needs 0.0-1.0, you may need to divide this result by the max angle.
        return Range.clip(calculatedAngle, 0.0, 180.0);
    }

    /**
     * Calculates Motor Velocity (RPM/Ticks) using Cubic Regression
     * Equation: y = -0.00116902x^3 + 0.953686x^2 - 190.72172x
     */
    public static double calculateVelocity(double distanceCm) {
        double x = distanceCm;

        double calculatedVel =
                -0.00116902 * Math.pow(x, 3)
                        + 0.953686 * Math.pow(x, 2)
                        - 190.72172 * x;
        // + 0 (Intercept is 0)

        // Safety clamp:
        // This equation generates negative values for low distances (x < ~200).
        // Clipping at 0.0 prevents the motor from reversing.
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