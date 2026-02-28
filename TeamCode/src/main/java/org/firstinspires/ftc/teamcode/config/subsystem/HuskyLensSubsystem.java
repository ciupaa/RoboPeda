package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Thin wrapper around the FTC HuskyLens driver.
 *
 * This subsystem:
 * - COLOR_RECOGNITION: get dominant color ID per spindexer slot.
 * - TAG_RECOGNITION: read AprilTag ID for motif scan at auto/TeleOp start.
 *
 * Training of colors (ID1, ID2, ...) and tag recognition is done on the device.
 */
@Config
@Configurable
public class HuskyLensSubsystem extends SubsystemBase {

    private final HuskyLens huskyLens;

    /** Last seen AprilTag ID when in TAG_RECOGNITION mode; 0 if none. */
    private int lastAprilTagId = 0;

    /**
     * Minimum bounding box area (pixels^2) to consider a valid detection.
     * Helps to ignore tiny noise detections.
     */
    public static int MIN_BLOCK_AREA = 200;

    public HuskyLensSubsystem(HardwareMap hardwareMap) {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    /**
     * @return raw blocks as reported by the HuskyLens.
     */
    public HuskyLens.Block[] getBlocks() {
        return huskyLens.blocks();
    }

    /**
     * Returns the color ID of the "largest" detected block (by area),
     * or 0 if nothing meets the MIN_BLOCK_AREA threshold.
     *
     * Typical use: call while a single artifact is under the camera,
     * then assign the returned ID to the current spindexer slot.
     */
    public int getDominantColorId() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks == null || blocks.length == 0) return 0;

        int bestId = 0;
        int bestArea = 0;

        for (HuskyLens.Block block : blocks) {
            int area = block.width * block.height;
            if (area >= MIN_BLOCK_AREA && area > bestArea) {
                bestArea = area;
                bestId = block.id;
            }
        }
        return bestId;
    }

    /** Switch to AprilTag (tag) recognition for motif scan. */
    public void switchToAprilTagMode() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    /** Switch back to color recognition for spindexer color scan. */
    public void switchToColorMode() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    /**
     * Call each loop during motif scan. Updates lastAprilTagId from largest block.
     * @return current best tag ID, or 0 if none
     */
    public int updateAprilTagId() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        lastAprilTagId = 0;
        if (blocks != null && blocks.length > 0) {
            int bestArea = 0;
            for (HuskyLens.Block block : blocks) {
                int area = block.width * block.height;
                if (area > bestArea) {
                    bestArea = area;
                    lastAprilTagId = block.id;
                }
            }
        }
        return lastAprilTagId;
    }

    /** Last AprilTag ID seen (after updateAprilTagId). */
    public int getLastAprilTagId() {
        return lastAprilTagId;
    }
}

