package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

/**
 * Velocity-compensated shooter calculator per Team 23435 Gyrobotic Droids:
 * "Hood Angle and Velocity Calcs" + velocity compensation for shooting while moving.
 *
 * @see <a href="https://www.youtube.com/watch?v=oSVNTER_37A">YouTube explanation</a>
 *
 * Uses:
 * - Projectile motion so the ball passes through the goal at (x, y) at angle θ.
 * - Robot velocity (from odometry) to compensate turret offset, launch angle, and launch speed.
 * - Linear mapping from launch angle (deg) → hood servo position and launch speed (in/s) → flywheel velocity.
 *
 * All distances/angles in inches and degrees unless noted.
 */
@Config
@Configurable
public class ShooterCalculatorVelocityComp {

    /** Gravity in in/s² (9.8 m/s² ≈ 386.1 in/s²) */
    public static double G_IN_PER_S2 = 386.1;

    /** Required trajectory angle at the goal (degrees, relative to horizontal). 0 = level entry. */
    public static double REQUIRED_ANGLE_AT_GOAL_DEG = 0.0;

    /** Hood angle limits (degrees) – clamp computed angle to this range. */
    public static double HOOD_MIN_DEG = 35.0;
    public static double HOOD_MAX_DEG = 55.0;

    /** Hood servo: two-point linear map. Angle (deg) → servo position. */
    public static double HOOD_ANGLE_1_DEG = 35.0;
    public static double HOOD_SERVO_1 = 0.70;
    public static double HOOD_ANGLE_2_DEG = 55.0;
    public static double HOOD_SERVO_2 = 0.65;

    /** Flywheel: linear map launch speed (in/s) → motor velocity (REV ticks/s or same units as setVelocity). */
    public static double LAUNCH_SPEED_TO_VELOCITY_SLOPE = 80.0;
    public static double LAUNCH_SPEED_TO_VELOCITY_OFFSET = 400.0;

    /** Minimum horizontal distance (in) to avoid div-by-zero / bad math. */
    public static double MIN_DISTANCE_IN = 6.0;

    /**
     * Result of the velocity-compensated shot calculation.
     */
    public static class ShotResult {
        /** Launch angle (degrees, relative to horizontal). */
        public final double launchAngleDeg;
        /** Launch speed (in/s). */
        public final double launchSpeedInPerSec;
        /** Hood servo position (0–1). */
        public final double hoodServoPosition;
        /** Flywheel velocity (same units as TurretedShooter.setVelocity). */
        public final double flywheelVelocity;
        /** Turret offset (degrees) to add to base aim for velocity compensation. */
        public final double turretOffsetDeg;
        /** Time of flight to goal (seconds). */
        public final double timeOfFlightSec;
        /** Whether velocity compensation was applied (robot was moving). */
        public final boolean usedVelocityCompensation;

        public ShotResult(double launchAngleDeg, double launchSpeedInPerSec,
                          double hoodServoPosition, double flywheelVelocity,
                          double turretOffsetDeg, double timeOfFlightSec,
                          boolean usedVelocityCompensation) {
            this.launchAngleDeg = launchAngleDeg;
            this.launchSpeedInPerSec = launchSpeedInPerSec;
            this.hoodServoPosition = hoodServoPosition;
            this.flywheelVelocity = flywheelVelocity;
            this.turretOffsetDeg = turretOffsetDeg;
            this.timeOfFlightSec = timeOfFlightSec;
            this.usedVelocityCompensation = usedVelocityCompensation;
        }
    }

    /**
     * Compute stationary launch angle and speed so the ball passes through (x, y) at angle θ.
     * x = horizontal distance (in), y = height of goal above launch (in), θ = required angle at goal (deg).
     */
    public static double launchAngleRadStationary(double xIn, double yIn, double thetaGoalDeg) {
        if (xIn < MIN_DISTANCE_IN) xIn = MIN_DISTANCE_IN;
        double thetaRad = Math.toRadians(thetaGoalDeg);
        double tanAlpha = 2.0 * yIn / xIn - Math.tan(thetaRad);
        return Math.atan(tanAlpha);
    }

    /**
     * Launch speed (in/s) for stationary shot given launch angle α (rad) and goal (x, y), θ (rad).
     */
    public static double launchSpeedInPerSecStationary(double xIn, double yIn, double alphaRad, double thetaRad) {
        if (xIn < MIN_DISTANCE_IN) xIn = MIN_DISTANCE_IN;
        double cosAlpha = Math.cos(alphaRad);
        double tanAlpha = Math.tan(alphaRad);
        double denom = cosAlpha * cosAlpha * (tanAlpha - Math.tan(thetaRad));
        if (denom <= 0) return 400.0; // fallback
        double v0Sq = G_IN_PER_S2 * xIn / denom;
        return Math.sqrt(Math.max(v0Sq, 0));
    }

    /**
     * Full velocity-compensated shot from robot pose and velocity (field frame).
     *
     * @param robotXIn       robot X (in)
     * @param robotYIn       robot Y (in)
     * @param goalXIn        goal X (in)
     * @param goalYIn        goal Y (in)
     * @param goalHeightIn   height of goal center above robot launch point (in)
     * @param robotVelXInPerS robot velocity X (in/s), field frame
     * @param robotVelYInPerS robot velocity Y (in/s), field frame
     */
    public static ShotResult computeShot(double robotXIn, double robotYIn,
                                         double goalXIn, double goalYIn,
                                         double goalHeightIn,
                                         double robotVelXInPerS, double robotVelYInPerS) {
        double dx = goalXIn - robotXIn;
        double dy = goalYIn - robotYIn;
        double distanceHorizontalIn = Math.hypot(dx, dy);
        if (distanceHorizontalIn < MIN_DISTANCE_IN) distanceHorizontalIn = MIN_DISTANCE_IN;

        double thetaGoalDeg = REQUIRED_ANGLE_AT_GOAL_DEG;
        double thetaGoalRad = Math.toRadians(thetaGoalDeg);

        // --- Part A: stationary launch angle and speed ---
        double alphaRad = launchAngleRadStationary(distanceHorizontalIn, goalHeightIn, thetaGoalDeg);
        double v0 = launchSpeedInPerSecStationary(distanceHorizontalIn, goalHeightIn, alphaRad, thetaGoalRad);

        double vxRobot = robotVelXInPerS;
        double vyRobot = robotVelYInPerS;
        double vMag = Math.hypot(vxRobot, vyRobot);
        double turretOffsetDeg = 0.0;
        boolean usedComp = false;

        if (vMag > 2.0) {
            usedComp = true;
            // --- Part B: velocity compensation ---
            double thetaLineRad = Math.atan2(dy, dx);
            double thetaVelocityRad = Math.atan2(vyRobot, vxRobot);
            double theta = thetaVelocityRad - thetaLineRad;

            double vRadial = -Math.cos(theta) * vMag;
            double vTangential = Math.sin(theta) * vMag;

            double t = distanceHorizontalIn / (v0 * Math.cos(alphaRad));
            if (t <= 0) t = 0.1;

            double vxComp = (distanceHorizontalIn / t) + vRadial;
            double vxNew = Math.sqrt(vxComp * vxComp + vTangential * vTangential);
            if (vxNew < 10) vxNew = 10;

            double vy = v0 * Math.sin(alphaRad);
            double alphaNewRad = Math.atan2(vy, vxNew);
            alphaNewRad = Range.clip(alphaNewRad, Math.toRadians(HOOD_MIN_DEG), Math.toRadians(HOOD_MAX_DEG));
            alphaRad = alphaNewRad;

            double xNew = vxNew * t;
            double denomNew = 2.0 * Math.cos(alphaNewRad) * Math.cos(alphaNewRad) * (xNew * Math.tan(alphaNewRad) - goalHeightIn);
            if (denomNew > 0) {
                v0 = Math.sqrt(G_IN_PER_S2 * xNew * xNew / denomNew);
            }

            turretOffsetDeg = Math.toDegrees(Math.atan2(vTangential, vxComp));
        }

        double alphaDeg = Math.toDegrees(alphaRad);
        alphaDeg = Range.clip(alphaDeg, HOOD_MIN_DEG, HOOD_MAX_DEG);

        double hoodServo = (HOOD_SERVO_2 - HOOD_SERVO_1) / (HOOD_ANGLE_2_DEG - HOOD_ANGLE_1_DEG) * (alphaDeg - HOOD_ANGLE_1_DEG) + HOOD_SERVO_1;
        hoodServo = Range.clip(hoodServo, 0.0, 1.0);

        double flywheelVel = LAUNCH_SPEED_TO_VELOCITY_SLOPE * v0 + LAUNCH_SPEED_TO_VELOCITY_OFFSET;
        flywheelVel = Range.clip(flywheelVel, 0.0, 6000.0);

        double t = distanceHorizontalIn / (v0 * Math.cos(alphaRad));
        if (t <= 0) t = 0.1;

        return new ShotResult(alphaDeg, v0, hoodServo, flywheelVel, turretOffsetDeg, t, usedComp);
    }

    /**
     * Stationary shot only (no velocity compensation). Convenience when robot velocity is zero.
     */
    public static ShotResult computeShotStationary(double distanceHorizontalIn, double goalHeightIn) {
        return computeShot(0, 0, distanceHorizontalIn, 0, goalHeightIn, 0, 0);
    }
}
