package org.firstinspires.ftc.teamcode.config.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
public class Limelight_camera extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult = null;

    // Target positions on field (METERS) - TUNE THESE IN FTC DASHBOARD!
    public static double BLUE_TARGET_X = 0.0;
    public static double BLUE_TARGET_Y = 3.6;

    public static double RED_TARGET_X = 0.0;
    public static double RED_TARGET_Y = -3.6;

    public final boolean isBlue;

    // Diagnostic data
    private double lastBotposeX = 0;
    private double lastBotposeY = 0;
    private double lastBotposeZ = 0;
    private double lastDistance = -1;

    public Limelight_camera(HardwareMap hardwareMap, int pipeline, boolean isBlueAlliance) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        this.isBlue = isBlueAlliance;
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
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
     * Get robot position from MegaTag (MT1)
     * Returns Pose3D object with x, y, z in METERS
     */
    public Pose3D getBotpose() {
        if (!hasTarget()) return null;

        Pose3D botpose = latestResult.getBotpose();

        if (botpose == null) {
            return null;
        }

        lastBotposeX = botpose.getPosition().x;
        lastBotposeY = botpose.getPosition().y;
        lastBotposeZ = botpose.getPosition().z;

        return botpose;
    }

    /**
     * Calculate distance to target using botpose
     * Returns distance in CENTIMETERS
     */
    public double getDistanceToTarget() {
        Pose3D botpose = getBotpose();
        if (botpose == null) {
            lastDistance = -1;
            return -1.0;
        }

        double robotX = botpose.getPosition().x;
        double robotY = botpose.getPosition().y;

        double targetX = isBlue ? BLUE_TARGET_X : RED_TARGET_X;
        double targetY = isBlue ? BLUE_TARGET_Y : RED_TARGET_Y;

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double distanceMeters = Math.sqrt(dx * dx + dy * dy);
        double distanceCm = distanceMeters * 100.0;

        if (distanceCm < 0 || distanceCm > 1000) {
            lastDistance = -1;
            return -1.0;
        }

        lastDistance = distanceCm;
        return distanceCm;
    }

    // Diagnostic getters
    public double getLastBotposeX() { return lastBotposeX; }
    public double getLastBotposeY() { return lastBotposeY; }
    public double getLastBotposeZ() { return lastBotposeZ; }
    public double getLastDistance() { return lastDistance; }

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
        return hasTarget() && Math.abs(getTx()) <= tolerance;
    }

    public double getAlignmentError() {
        return getTx();
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public void stop() {
        limelight.stop();
    }
}