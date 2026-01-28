package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Limelight Vision Subsystem - CENTIMETERS (CALIBRATED)
 *
 * CALIBRATION DATA (from real testing):
 * Real distance: 121 cm
 * Raw distance: 169 cm
 * Correction factor: 121 / 169 = 0.716
 *
 * IMPORTANT: Tune DISTANCE_CORRECTION_FACTOR in FTC Dashboard if needed!
 */
@Configurable
public class Limelight_camera extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult = null;

    // === PHYSICAL CONSTANTS - IN CENTIMETERS ===

    public static double LIMELIGHT_HEIGHT_CM = 32.0;      // Height of camera from ground
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 15.0; // Angle camera is tilted up
    public static double TARGET_HEIGHT_CM = 75.0;          // Height of AprilTag from ground

    /**
     * DISTANCE CORRECTION FACTOR - CALIBRATED!
     *
     * Based on real testing:
     * Real: 121 cm, Raw: 169 cm
     * Correction: 121 / 169 = 0.716
     *
     * HOW TO CALIBRATE:
     * 1. Stand at exact distance with tape (e.g., 100 cm, 150 cm, 200 cm)
     * 2. Check "RAW" distance in telemetry
     * 3. Correction = Real_Distance / RAW_Distance
     * 4. Update this value in FTC Dashboard
     * 5. Test at multiple distances and average
     */
    public static double DISTANCE_CORRECTION_FACTOR = 0.716;

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

        // Apply correction factor to get actual CM
        double correctedDistance = rawDistance * DISTANCE_CORRECTION_FACTOR;

        return correctedDistance;
    }

    /**
     * Get RAW distance (before correction) for debugging
     * This shows what the trigonometry calculates without correction
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