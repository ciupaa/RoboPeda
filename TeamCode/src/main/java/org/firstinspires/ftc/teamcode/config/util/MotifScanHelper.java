package org.firstinspires.ftc.teamcode.config.util;

import org.firstinspires.ftc.teamcode.config.Robot_v2;

/**
 * Runs AprilTag motif scan at start of auto and TeleOp:
 * 1) Turret to scan angle
 * 2) HuskyLens to TAG_RECOGNITION
 * 3) Poll for AprilTag ID until seen or timeout
 * 4) Set motif on spindexer
 * 5) HuskyLens back to COLOR_RECOGNITION
 *
 * Call from init() (TeleOp) or at the very start of auto.
 */
public final class MotifScanHelper {

    /** How long to wait for AprilTag (ms) before using default motif. */
    public static long SCAN_TIMEOUT_MS = 2000;
    /** Default motif if no tag seen in time. */
    public static Motif DEFAULT_MOTIF = Motif.GREEN_PURPLE_PURPLE;

    /**
     * Blocking motif scan. Moves turret to scan position, reads AprilTag, sets motif on spindexer.
     *
     * @param r        Robot_v2 (turret, huskyLens, spindexer)
     * @param timeoutMs max time to wait for tag (ms)
     * @return motif that was set (from tag or default)
     */
    public static Motif runMotifScan(Robot_v2 r, long timeoutMs) {
        r.turretShooter.setTurretToAprilTagScanPosition();
        r.huskyLens.switchToAprilTagMode();

        long deadline = System.currentTimeMillis() + timeoutMs;
        int tagId = 0;
        while (System.currentTimeMillis() < deadline) {
            r.periodic();
            tagId = r.huskyLens.updateAprilTagId();
            if (tagId > 0) break;
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        Motif motif = tagId > 0 ? Motif.fromAprilTagId(tagId) : DEFAULT_MOTIF;
        r.spindexer.setMotif(motif);
        r.huskyLens.switchToColorMode();
        return motif;
    }

    /** Run with default timeout. */
    public static Motif runMotifScan(Robot_v2 r) {
        return runMotifScan(r, SCAN_TIMEOUT_MS);
    }
}
