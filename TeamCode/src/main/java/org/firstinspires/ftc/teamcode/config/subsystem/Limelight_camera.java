package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Limelight Vision Subsystem - CENTIMETERS
 *
 * CORRECTED Distance calculation:
 * Shows 135" in driver station but actual distance is 135cm
 * Conversion: 135 inches * 2.54 = 342.9 cm (what it calculates)
 * Actual: 135 cm
 * Correction factor: 135 / 342.9 = 0.3937 (converts to CM)
 */
@Config
public class Limelight_camera extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult = null;

    // === PHYSICAL CONSTANTS - IN CENTIMETERS ===

    public static double LIMELIGHT_HEIGHT_CM = 32.0;      // 12.6" = 32.0 cm
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 15.0;
    public static double TARGET_HEIGHT_CM = 75.0;         // 29.53" = 75.0 cm

    /**
     * DISTANCE CORRECTION FACTOR - CONVERTS TO CM
     *
     * Driver station shows: 135 inches
     * Actual distance: 135 cm
     *
     * Raw calculation gives inches, so we need to:
     * 1. NOT multiply by 2.54 (that would give wrong CM)
     * 2. Instead use correction: 135cm / (135in * 2.54) = 0.3937
     *
     * This directly converts the inch-based calculation to CM
     */
    public static double DISTANCE_CORRECTION_FACTOR = 0.3937;

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

    // ========== DISTANCE CALCULATION - IN CENTIMETERS ==========

    /**
     * Calculate distance to AprilTag in CENTIMETERS
     *
     * @return Corrected distance in CM, or -1 if no target
     */
    public double getDistanceToTarget() {
        if (!hasTarget()) return -1.0;

        double ty = getTy();
        double angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);

        double heightDifference = TARGET_HEIGHT_CM - LIMELIGHT_HEIGHT_CM;
        double rawDistance = heightDifference / Math.tan(angleToTargetRad);

        // Apply correction to convert to CM
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

        double heightDifference = TARGET_HEIGHT_CM - LIMELIGHT_HEIGHT_CM;
        return heightDifference / Math.tan(angleToTargetRad);
    }

    /**
     * Get distance in meters
     */
    public double getDistanceToTargetMeters() {
        double cm = getDistanceToTarget();
        if (cm < 0) return -1.0;
        return cm / 100.0;
    }

    /**
     * Check if in optimal shooting range (CM)
     */
    public boolean isInRange(double minDistanceCm, double maxDistanceCm) {
        double distance = getDistanceToTarget();
        return distance > 0 && distance >= minDistanceCm && distance <= maxDistanceCm;
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