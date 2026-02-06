package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * ShooterCalculator_camera - QUARTIC REGRESSION EQUATIONS
 *
 * Uses quartic (4th degree) polynomial regression calculated from Desmos to determine
 * exact angle and velocity for any given distance.
 *
 * NEW EQUATIONS (QUARTIC - UPDATED):
 *
 * Velocity: y = 0.0000029312x⁴ - 0.00271878x³ + 0.911705x² - 127.19172x + 7347.21042
 * Angle:    y = -(7.91756×10⁻¹⁰)x⁴ + (7.91916×10⁻⁷)x³ - 0.00027955x² + 0.0405522x - 1.19504
 *
 * These equations should provide better accuracy across the full range of distances
 * compared to the previous cubic equations.
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
     * Calculates Servo Angle using Quartic (4th degree) Regression
     *
     * NEW EQUATION:
     * y = -(7.91756×10⁻¹⁰)x⁴ + (7.91916×10⁻⁷)x³ - 0.00027955x² + 0.0405522x - 1.19504
     */
    public static double calculateAngle(double distanceCm) {
        double x = distanceCm;

        double calculatedAngle =
                -(7.91756e-10) * Math.pow(x, 4)
                        + (7.91916e-7) * Math.pow(x, 3)
                        - 0.00027955 * Math.pow(x, 2)
                        + 0.0405522 * x
                        - 1.19504;

        // Safety clamp: 0.0 to 1.0 (Servo Position)
        return Range.clip(calculatedAngle, 0.0, 1.0);
    }

    /**
     * Calculates Motor Velocity (RPM/Ticks) using Quartic (4th degree) Regression
     *
     * NEW EQUATION:
     * y = 0.0000029312x⁴ - 0.00271878x³ + 0.911705x² - 127.19172x + 7347.21042
     */
    public static double calculateVelocity(double distanceCm) {
        double x = distanceCm;

        double calculatedVel =
                0.0000029312 * Math.pow(x, 4)
                        - 0.00271878 * Math.pow(x, 3)
                        + 0.911705 * Math.pow(x, 2)
                        - 127.19172 * x
                        + 7347.21042;

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