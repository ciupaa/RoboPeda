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
 * - Configures the HuskyLens in COLOR_RECOGNITION mode
 * - Exposes helper methods to get the "dominant" detected color ID
 *   that you can store per spindexer slot.
 *
 * Training of colors (ID1, ID2, ...) is done on the device itself.
 */
@Config
@Configurable
public class HuskyLensSubsystem extends SubsystemBase {

    private final HuskyLens huskyLens;

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
}

