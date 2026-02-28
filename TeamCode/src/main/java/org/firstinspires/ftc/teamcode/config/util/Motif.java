package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

/**
 * FTC Decode motif: required order of artifact colors for scoring.
 * Three patterns: Green Purple Purple, Purple Green Purple, Purple Purple Green.
 * Set via AprilTag scan at start of auto and TeleOp.
 */
@Config
@Configurable
public enum Motif {
    /** First green, then purple, purple */
    GREEN_PURPLE_PURPLE(new int[]{1, 2, 2}),
    /** First purple, then green, then purple */
    PURPLE_GREEN_PURPLE(new int[]{2, 1, 2}),
    /** First purple, purple, then green */
    PURPLE_PURPLE_GREEN(new int[]{2, 2, 1});

    /** HuskyLens color ID for Green (train as ID 1 or 2; set below). */
    public static int GREEN_HUSKY_COLOR_ID = 1;
    /** HuskyLens color ID for Purple. */
    public static int PURPLE_HUSKY_COLOR_ID = 2;

    /** Order of color IDs for this motif: [first, second, third]. 1=green, 2=purple. */
    private final int[] colorOrder;

    Motif(int[] colorOrder) {
        this.colorOrder = colorOrder;
    }

    /** Desired color ID (HuskyLens) at position 0, 1, or 2. */
    public int getColorIdAtPosition(int position) {
        if (position < 0 || position >= 3) return 0;
        return colorOrder[position];
    }

    /** AprilTag ID → Motif. Tune to match game tags (e.g. 1=GPP, 2=PGP, 3=PPG). */
    public static Motif fromAprilTagId(int tagId) {
        switch (tagId) {
            case 1: return GREEN_PURPLE_PURPLE;
            case 2: return PURPLE_GREEN_PURPLE;
            case 3: return PURPLE_PURPLE_GREEN;
            default: return GREEN_PURPLE_PURPLE;
        }
    }
}
