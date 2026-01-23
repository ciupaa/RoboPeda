package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Limelight Vision Subsystem - CORRECTED Distance
 *
 * NEW Distance correction:
 * Shows 36.5" but actual is 59.84" → Correction: 59.84/36.5 = 1.639
 */
@Config
public class Limelight_camera extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult = null;

    // === PHYSICAL CONSTANTS ===

    public static double LIMELIGHT_HEIGHT_INCHES = 12.6;
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 15.0;
    public static double TARGET_HEIGHT_INCHES = 29.53;

    /**
     * DISTANCE CORRECTION FACTOR - UPDATED
     *
     * Your NEW measurements: Shows 36.5", actual is 59.84"
     * Correction: 59.84 / 36.5 = 1.639
     *
     * TUNING: Adjust in FTC Dashboard if needed
     */
    public static double DISTANCE_CORRECTION_FACTOR = 1.639;

    public Limelight_camera(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
    }

    // ========== TARGET DETECTION ==========

    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    // ========== RAW TARGETING DATA ==========

    public double getTx() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTx();
    }

    public double getTy() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTy();
    }

    public double getTa() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTa();
    }

    // ========== DISTANCE CALCULATION - WITH CORRECTION ==========

    /**
     * Calculate distance to AprilTag with empirical correction
     *
     * @return Corrected distance in INCHES, or -1 if no target
     */
    public double getDistanceToTarget() {
        if (!hasTarget()) return -1.0;

        double ty = getTy();
        double angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);

        double heightDifference = TARGET_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES;
        double rawDistance = heightDifference / Math.tan(angleToTargetRad);

        // Apply empirical correction factor
        double correctedDistance = rawDistance * DISTANCE_CORRECTION_FACTOR;

        return correctedDistance;
    }

    /**
     * Get RAW distance (before correction) for debugging
     */
    public double getRawDistance() {
        if (!hasTarget()) return -1.0;

        double ty = getTy();
        double angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);

        double heightDifference = TARGET_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES;
        return heightDifference / Math.tan(angleToTargetRad);
    }

    /**
     * Get distance in feet
     */
    public double getDistanceToTargetFeet() {
        double inches = getDistanceToTarget();
        if (inches < 0) return -1.0;
        return inches / 12.0;
    }

    /**
     * Check if in optimal shooting range
     */
    public boolean isInRange(double minDistance, double maxDistance) {
        double distance = getDistanceToTarget();
        return distance > 0 && distance >= minDistance && distance <= maxDistance;
    }

    // ========== ALIGNMENT HELPERS ==========

    public boolean isAligned(double tolerance) {
        return hasTarget() && Math.abs(getTx()) <= tolerance;
    }

    public double getAlignmentError() {
        return getTx();
    }

    // ========== PIPELINE CONTROL ==========

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public void stop() {
        limelight.stop();
    }
}