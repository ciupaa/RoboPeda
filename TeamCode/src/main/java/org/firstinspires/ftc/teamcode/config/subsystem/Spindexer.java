package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.config.util.Motif;

/**
 * Three-slot spindexer that:
 * - Accepts artifacts from the intake
 * - Indexes them into 3 positions
 * - Stores a color ID for each slot (from HuskyLens)
 *
 * This class handles ONLY the motor motion and bookkeeping.
 * Higher-level sequencing (when to rotate, when to scan, when to feed shooter)
 * should be done in commands or OpModes.
 */
@Config
@Configurable
public class Spindexer extends SubsystemBase {

    private final DcMotorEx motor;

    // --- GEOMETRY / TUNING ---

    /**
     * Number of discrete storage slots in the spindexer.
     */
    public static int SLOT_COUNT = 3;

    /**
     * Encoder ticks between adjacent slots.
     * Measure one full revolution in ticks, divide by SLOT_COUNT.
     */
    public static int TICKS_PER_SLOT = 1000;

    /**
     * Maximum motor power used when indexing between slots.
     */
    public static double INDEX_POWER = 0.6;

    /**
     * Maximum motor power used when feeding to the shooter (reverse direction).
     */
    public static double FEED_POWER = 0.6;

    /**
     * Small tolerance in ticks to consider the motor "at target".
     */
    public static int POSITION_TOLERANCE_TICKS = 20;

    /**
     * Optional offset to align encoder zero with a physical slot index.
     * For example, if encoder 0 is halfway between two slots, set this to the
     * encoder value of a real slot position.
     */
    public static int ENCODER_ZERO_OFFSET_TICKS = 0;

    // --- TIMING (from FTC Decode artifact size + motor RPM) ---
    /**
     * FTC Decode artifact: nominal 5" diameter ball.
     * Spindexer has 3 slots → 360/3 = 120° per slot.
     * At 435 RPM: 435/60 = 7.25 rev/s → time for 120° = (120/360)/7.25 ≈ 0.046 s.
     */
    public static int SPINDEXER_MOTOR_RPM = 435;
    /**
     * Time in milliseconds to rotate one slot (120°) at SPINDEXER_MOTOR_RPM.
     * Formula: (360/3) / (RPM/60) * 1000 = 20000/RPM ≈ 46 ms at 435 RPM.
     */
    public static int INDEX_TIME_MS = (int) Math.round(20000.0 / SPINDEXER_MOTOR_RPM);

    // --- STATE ---

    /**
     * 0..SLOT_COUNT-1. This is our logical index in the carousel.
     * It is defined relative to ENCODER_ZERO_OFFSET_TICKS.
     */
    private int currentSlotIndex = 0;

    /**
     * Simple storage for HuskyLens "color IDs" or any application-level code.
     * Use Motif.GREEN_HUSKY_COLOR_ID and Motif.PURPLE_HUSKY_COLOR_ID for Decode.
     */
    private final int[] slotColorIds;

    /** Motif from AprilTag scan (GPP, PGP, PPG). Null until scanned. */
    private Motif motif = null;

    /** Which of the 3 motif positions we will feed next (0, 1, or 2). */
    private int nextFeedPosition = 0;

    public Spindexer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexer");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        slotColorIds = new int[SLOT_COUNT];
        for (int i = 0; i < SLOT_COUNT; i++) {
            slotColorIds[i] = 0; // 0 = unknown / unassigned
        }
    }

    // --- PUBLIC API ---

    /**
     * @return current logical slot index (0..SLOT_COUNT-1)
     */
    public int getCurrentSlotIndex() {
        return currentSlotIndex;
    }

    /**
     * Rotates the carousel forward by 1 slot (for intaking the next artifact).
     * Non-blocking: call isFinished() or atTarget() to know when motion is done.
     */
    public void advanceToNextSlot() {
        int nextIndex = (currentSlotIndex + 1) % SLOT_COUNT;
        moveToSlot(nextIndex, INDEX_POWER);
    }

    /**
     * Rotates the carousel so that the specified logical slot index is under the HuskyLens
     * or shooter feed location, depending on mechanical layout.
     *
     * @param slotIndex slot index 0..SLOT_COUNT-1
     */
    public void moveToSlot(int slotIndex) {
        moveToSlot(slotIndex, INDEX_POWER);
    }

    /**
     * Internal helper: move to a desired slot index with custom power.
     */
    private void moveToSlot(int slotIndex, double power) {
        if (slotIndex < 0 || slotIndex >= SLOT_COUNT) return;
        currentSlotIndex = slotIndex;

        int targetTicks = ENCODER_ZERO_OFFSET_TICKS + slotIndex * TICKS_PER_SLOT;
        motor.setTargetPosition(targetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.abs(power));
    }

    /**
     * Feeds the current slot toward the shooter by spinning in the opposite direction.
     * This assumes your passive arm or ramp is positioned so that reverse rotation lifts
     * or drops exactly one artifact into the shooter.
     *
     * For more precise control, you can implement a separate "feed" distance using
     * a negative target position offset from the current slot.
     */
    public void feedCurrentSlotToShooter() {
        int currentTicks = motor.getCurrentPosition();
        int targetTicks = currentTicks - TICKS_PER_SLOT;
        motor.setTargetPosition(targetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.abs(FEED_POWER));
    }

    /**
     * @return true if the motor is within POSITION_TOLERANCE_TICKS of its target.
     */
    public boolean atTarget() {
        return Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) <= POSITION_TOLERANCE_TICKS;
    }

    /**
     * Store a color ID for the current slot.
     */
    public void setCurrentSlotColorId(int colorId) {
        slotColorIds[currentSlotIndex] = colorId;
    }

    /**
     * Get the stored color ID for a given slot.
     */
    public int getSlotColorId(int slotIndex) {
        if (slotIndex < 0 || slotIndex >= SLOT_COUNT) return 0;
        return slotColorIds[slotIndex];
    }

    /**
     * Clears all stored color IDs (e.g., at start of match or after emptying).
     */
    public void clearAllColors() {
        for (int i = 0; i < SLOT_COUNT; i++) {
            slotColorIds[i] = 0;
        }
    }

    // --- MOTIF (AprilTag scan at auto / TeleOp start) ---

    /** Set motif from AprilTag scan. Call after reading tag at start of auto and TeleOp. */
    public void setMotif(Motif m) {
        motif = m;
        nextFeedPosition = 0;
    }

    /** Current motif, or null if not yet scanned. */
    public Motif getMotif() {
        return motif;
    }

    /**
     * Feed order: which slot index to feed 1st, 2nd, 3rd to match motif.
     * Returns {slotForFirst, slotForSecond, slotForThird} or null if motif not set or slot colors unknown.
     */
    public int[] getFeedOrder() {
        if (motif == null) return null;
        int[] order = new int[3];
        boolean[] used = new boolean[3];
        for (int pos = 0; pos < 3; pos++) {
            int wantColorId = motif.getColorIdAtPosition(pos);
            int slot = -1;
            for (int s = 0; s < SLOT_COUNT; s++) {
                if (!used[s] && slotColorIds[s] == wantColorId) {
                    slot = s;
                    used[s] = true;
                    break;
                }
            }
            if (slot < 0) return null;
            order[pos] = slot;
        }
        return order;
    }

    /** Slot index that should be fed next to maintain motif order. -1 if no motif or feed order unknown. */
    public int getNextFeedSlotIndex() {
        int[] feedOrder = getFeedOrder();
        if (feedOrder == null || nextFeedPosition >= 3) return -1;
        return feedOrder[nextFeedPosition];
    }

    /** Call after feeding one artifact; advances to next position in motif order. */
    public void advanceFeedPosition() {
        if (nextFeedPosition < 2) nextFeedPosition++;
    }

    /** Rotate spindexer so the slot that should be fed next is at the feed position (slot 0 = feed). */
    public void rotateToNextFeedSlot() {
        int slot = getNextFeedSlotIndex();
        if (slot >= 0) moveToSlot(slot);
    }

    /** Reset feed position to first in motif (e.g. start of shooting sequence). */
    public void resetFeedPosition() {
        nextFeedPosition = 0;
    }

    /**
     * @return raw encoder position (ticks), for dashboard debugging.
     */
    public int getEncoderPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * Time in ms to rotate one slot at current SPINDEXER_MOTOR_RPM.
     * Based on FTC Decode artifact (5" ball) and 3-slot layout (120° per slot).
     */
    public static int getIndexTimeMs() {
        return (int) Math.round(20000.0 / SPINDEXER_MOTOR_RPM);
    }
}

