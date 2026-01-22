package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Limelight Vision Subsystem (CAMERA VERSION)
 * Handles AprilTag detection, distance calculation, and targeting
 *
 * SETUP REQUIRED:
 * 1. Mount Limelight at a fixed angle on your robot
 * 2. Measure LIMELIGHT_HEIGHT_INCHES (lens center to floor)
 * 3. Measure LIMELIGHT_MOUNT_ANGLE_DEG (angle from horizontal)
 * 4. Measure TARGET_HEIGHT_INCHES (AprilTag center height)
 * 5. Configure AprilTag pipeline in Limelight web interface
 */
@Config
public class Limelight_camera extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult = null;

    // === PHYSICAL CONSTANTS - MEASURE ON YOUR ROBOT ===

    /**
     * Height from floor to Limelight lens center (inches)
     */
    public static double LIMELIGHT_HEIGHT_INCHES = 20.0;

    /**
     * Angle of Limelight mount from horizontal (degrees)
     * Positive = angled UP, Negative = angled DOWN
     */
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;

    /**
     * Height of AprilTag center from floor (inches)
     */
    public static double TARGET_HEIGHT_INCHES = 48.0;

    public Limelight_camera(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set to AprilTag pipeline
        limelight.pipelineSwitch(0);

        // Start polling
        limelight.start();
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
    }

    // ========== TARGET DETECTION ==========

    /**
     * Check if Limelight sees a valid AprilTag
     */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    // ========== RAW TARGETING DATA ==========

    /**
     * Get horizontal offset (degrees)
     * Negative = left, Positive = right
     */
    public double getTx() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTx();
    }

    /**
     * Get vertical offset (degrees)
     */
    public double getTy() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTy();
    }

    /**
     * Get target area (0-100% of image)
     */
    public double getTa() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTa();
    }

    // ========== DISTANCE CALCULATION ==========

    /**
     * Calculate distance to AprilTag using trigonometry
     *
     * Formula: distance = (targetHeight - cameraHeight) / tan(mountAngle + ty)
     *
     * @return Distance in INCHES, or -1 if no target
     */
    public double getDistanceToTarget() {
        if (!hasTarget()) return -1.0;

        double ty = getTy();
        double angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);

        double heightDifference = TARGET_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES;
        double distance = heightDifference / Math.tan(angleToTargetRad);

        return distance;
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

    /**
     * Check if aligned (TX near zero)
     */
    public boolean isAligned(double tolerance) {
        return hasTarget() && Math.abs(getTx()) <= tolerance;
    }

    /**
     * Get rotation correction for PID
     */
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