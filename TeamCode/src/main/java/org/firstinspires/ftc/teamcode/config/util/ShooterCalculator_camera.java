package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * ShooterCalculator_camera - REGRESSION EQUATION
 *
 * Uses polynomial regression calculated from Desmos to determine
 * exact angle and velocity for any given distance.
 *
 * NEW EQUATIONS (UPDATED):
 *
 * Velocity: y = -0.0000285939x³ + 0.0243239x² - 4.86525x + 1483.56669
 * Angle:    y = -(5.72338×10⁻⁹)x³ + 0.00000542892x² - 0.00175533x + 0.846639
 */
@Config
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
     *
     * NEW EQUATION:
     * y = -(5.72338×10⁻⁹)x³ + 0.00000542892x² - 0.00175533x + 0.846639
     */
    public static double calculateAngle(double distanceCm) {
        double x = distanceCm;

        double calculatedAngle =
                -(5.72338e-9) * Math.pow(x, 3)
                        + 0.00000542892 * Math.pow(x, 2)
                        - 0.00175533 * x
                        + 0.846639;

        // Safety clamp: 0.0 to 1.0 (Servo Position)
        return Range.clip(calculatedAngle, 0.0, 1.0);
    }

    /**
     * Calculates Motor Velocity (RPM/Ticks) using Cubic Regression
     *
     * NEW EQUATION:
     * y = -0.0000285939x³ + 0.0243239x² - 4.86525x + 1483.56669
     */
    public static double calculateVelocity(double distanceCm) {
        double x = distanceCm;

        double calculatedVel =
                -0.0000285939 * Math.pow(x, 3)
                        + 0.0243239 * Math.pow(x, 2)
                        - 4.86525 * x
                        + 1483.56669;

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