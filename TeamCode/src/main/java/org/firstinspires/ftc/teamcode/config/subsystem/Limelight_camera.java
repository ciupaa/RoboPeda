package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Limelight Vision Subsystem - RAW DISTANCE (CENTIMETERS)
 *
 * Uses pure trigonometric distance calculation - NO CORRECTION FACTOR.
 * Make sure these values are PRECISELY measured:
 * - Camera height from ground to lens center
 * - Camera tilt angle (use protractor or level app)
 * - AprilTag height from ground to tag center
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
     * Constructor with pipeline selection
     * @param hardwareMap Hardware map
     * @param pipeline Pipeline number (1 for Blue, 2 for Red, etc.)
     */
    public Limelight_camera(HardwareMap hardwareMap, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
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

    // ========== DISTANCE CALCULATION - RAW (NO CORRECTION) ==========

    /**
     * Calculate distance to AprilTag in CENTIMETERS using trigonometry
     *
     * Formula: distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
     *
     * @return Distance in CM, or -1 if no target
     */
    public double getDistanceToTarget() {
        if (!hasTarget()) return -1.0;

        double ty = getTy();
        double angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);

        double heightDifference = TARGET_HEIGHT_CM - LIMELIGHT_HEIGHT_CM;
        double distance = heightDifference / Math.tan(angleToTargetRad);

        return distance;
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