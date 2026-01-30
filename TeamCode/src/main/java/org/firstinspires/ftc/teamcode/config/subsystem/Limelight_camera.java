package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@Config
@Configurable
public class Limelight_camera extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult = null;

    public final boolean isBlue;

    // Diagnostic data
    private double lastBotposeX = 0;
    private double lastBotposeY = 0;
    private double lastBotposeZ = 0;
    private double lastDistance = -1;
    private int lastDetectedTagId = -1;  // NEW: Track which tag is detected

    public Limelight_camera(HardwareMap hardwareMap, int pipeline, boolean isBlueAlliance) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        isBlue = isBlueAlliance;
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();

        if (latestResult != null && latestResult.isValid()) {
            // Update botpose
            Pose3D botpose = latestResult.getBotpose();
            if (botpose != null) {
                lastBotposeX = botpose.getPosition().x;
                lastBotposeY = botpose.getPosition().y;
                lastBotposeZ = botpose.getPosition().z;
            }

            // NEW: Track detected tag ID for debugging
            List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
            if (!fiducials.isEmpty()) {
                lastDetectedTagId = fiducials.get(0).getFiducialId();
            } else {
                lastDetectedTagId = -1;
            }
        } else {
            lastDetectedTagId = -1;
        }
    }

    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

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

    /**
     * Calculates distance using MegaTag (BotPose) but RELATIVE to the tag.
     * This fixes coordinate system errors (e.g., getting 389cm instead of 145cm).
     */
    public double getDistanceMegaTag() {
        if (!hasTarget()) return -1.0;

        // Get the list of tags visible (Fiducials)
        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();

        if (!fiducials.isEmpty()) {
            // Get the primary tag (usually the first one or the one with highest area)
            LLResultTypes.FiducialResult tag = fiducials.get(0);

            // Get Robot Pose relative to the Tag (Target Space)
            // X = Horizontal offset (left/right)
            // Y = Vertical offset (up/down)
            // Z = Forward distance (depth)
            Pose3D robotPoseTargetSpace = tag.getRobotPoseTargetSpace();

            double x = robotPoseTargetSpace.getPosition().x;
            double z = robotPoseTargetSpace.getPosition().z;

            // Calculate 2D ground distance (hypotenuse of X and Z)
            double distanceMeters = Math.hypot(x, z);

            return distanceMeters * 100.0; // Convert to CM
        }

        // Fallback if Fiducial list is empty but isValid is true
        return -1.0;
    }

    /**
     * Main distance getter.
     */
    public double getDistanceToTarget() {
        double distCm = getDistanceMegaTag();

        // Validity Checks
        if (distCm < 0 || distCm > 1000) {
            lastDistance = -1;
            return -1.0;
        }

        lastDistance = distCm;
        return distCm;
    }

    // Diagnostic getters
    public double getLastBotposeX() { return lastBotposeX; }
    public double getLastBotposeY() { return lastBotposeY; }
    public double getLastBotposeZ() { return lastBotposeZ; }
    public double getLastDistance() { return lastDistance; }

    // NEW: Get detected tag ID
    public int getDetectedTagId() { return lastDetectedTagId; }

    public double getDistanceToTargetMeters() {
        double cm = getDistanceToTarget();
        if (cm < 0) return -1.0;
        return cm / 100.0;
    }

    public boolean isInRange(double minDistanceCm, double maxDistanceCm) {
        double distance = getDistanceToTarget();
        return distance > 0 && distance >= minDistanceCm && distance <= maxDistanceCm;
    }

    public boolean isAligned(double tolerance) {
        return hasTarget() && Math.abs(getTx()) < tolerance;
    }

    // NEW: Manual pipeline switch (for debugging)
    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public void stop() {
        limelight.stop();
    }
}