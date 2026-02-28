package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

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

    // --- STATE ---

    /**
     * 0..SLOT_COUNT-1. This is our logical index in the carousel.
     * It is defined relative to ENCODER_ZERO_OFFSET_TICKS.
     */
    private int currentSlotIndex = 0;

    /**
     * Simple storage for HuskyLens "color IDs" or any application-level code.
     * For example: 1 = red, 2 = blue, 3 = yellow, etc.
     */
    private final int[] slotColorIds;

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

    /**
     * @return raw encoder position (ticks), for dashboard debugging.
     */
    public int getEncoderPosition() {
        return motor.getCurrentPosition();
    }
}

