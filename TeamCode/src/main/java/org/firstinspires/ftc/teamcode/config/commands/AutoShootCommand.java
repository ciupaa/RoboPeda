package org.firstinspires.ftc.teamcode.config.commands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Limelight_camera;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**
 * AUTO SHOOT COMMAND - FAST VERSION
 *
 * Checks for AprilTag IMMEDIATELY and shoots without delay.
 * If no tag is detected, uses appropriate failsafe based on auto type.
 *
 * FAILSAFES:
 * - FAR autos: 1500 velocity, 0.8 angle
 * - CLOSE autos: 1140 velocity, 0.84 angle ← UPDATED!
 */
@Config
public class AutoShootCommand extends CommandBase {

    private final Shooter shooter;
    private final Intake intake;
    private final Limelight_camera limelight;
    private final double shootDurationSeconds;
    private final boolean isFarShot;
    private final ElapsedTime timer = new ElapsedTime();

    // PULSE FEEDING STRATEGY
    private static final double FEED_PULSE_MS = 120;
    private static final double FEED_PAUSE_MS = 100;
    private static final double VELOCITY_DROP_THRESHOLD = 150;
    private static final double MIN_RECOVERY_TIME_MS = 400;
    private static final double VELOCITY_TOLERANCE = 80;

    // FAILSAFE VALUES
    private static final double FAR_FAILSAFE_ANGLE = 0.8;
    private static final double FAR_FAILSAFE_VELOCITY = 1630;
    private static final double CLOSE_FAILSAFE_ANGLE = 0.862;   // ← UPDATED!
    private static final double CLOSE_FAILSAFE_VELOCITY = 1316; // ← UPDATED!

    private enum State {
        WAIT_SPINUP,
        PULSE_FEED,
        PULSE_PAUSE,
        WAIT_RECOVERY,
        DONE
    }
    private State state = State.WAIT_SPINUP;

    private double targetAngle = 0.8;
    private double targetVelocity = 1550;
    private double lastStableVelocity = 0.0;
    private boolean usedFailsafe = false;

    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private double shootStartTime = 0;

    /**
     * Constructor
     * @param shooter Shooter subsystem
     * @param intake Intake subsystem
     * @param limelight Limelight camera
     * @param shootDurationSeconds Total time to run shooting sequence
     * @param isFarShot true for FAR autos (1500/0.8), false for CLOSE autos (1140/0.84)
     */
    public AutoShootCommand(Shooter shooter, Intake intake, Limelight_camera limelight,
                            double shootDurationSeconds, boolean isFarShot) {
        this.shooter = shooter;
        this.intake = intake;
        this.limelight = limelight;
        this.shootDurationSeconds = shootDurationSeconds;
        this.isFarShot = isFarShot;

        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        feedTimer.reset();
        recoveryTimer.reset();
        state = State.WAIT_SPINUP;
        lastStableVelocity = 0.0;
        shootStartTime = timer.seconds();
        usedFailsafe = false;

        // IMMEDIATELY check for tag and calculate - NO WAITING!
        if (limelight.hasTarget()) {
            double distanceCm = limelight.getDistanceToTarget();

            if (distanceCm > 0) {
                // Use camera distance
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distanceCm);
                targetAngle = config.angle;
                targetVelocity = config.velocity;
                usedFailsafe = false;
            } else {
                // Invalid distance - use failsafe
                setFailsafeValues();
                usedFailsafe = true;
            }
        } else {
            // No tag visible - use failsafe immediately
            setFailsafeValues();
            usedFailsafe = true;
        }

        // Start spinning up immediately
        shooter.setAngle(targetAngle);
        shooter.unblock();
        shooter.launcher.setVelocity(targetVelocity);
        intake.stop();
    }

    @Override
    public void execute() {
        double now = timer.seconds();

        // Check if we've exceeded total shoot duration
        if ((now - shootStartTime) >= shootDurationSeconds) {
            state = State.DONE;
            return;
        }

        // Keep shooter running at target
        shooter.setAngle(targetAngle);
        shooter.unblock();
        shooter.launcher.setVelocity(targetVelocity);

        double currentVel = shooter.getVelocity();
        boolean atSpeed = currentVel >= (targetVelocity - VELOCITY_TOLERANCE);

        switch (state) {
            case WAIT_SPINUP:
                intake.stop();

                if (atSpeed) {
                    lastStableVelocity = currentVel;
                    feedTimer.reset();
                    state = State.PULSE_FEED;
                }
                break;

            case PULSE_FEED:
                if (feedTimer.milliseconds() < FEED_PULSE_MS) {
                    intake.intake();
                } else {
                    intake.stop();
                    feedTimer.reset();
                    state = State.PULSE_PAUSE;
                }

                boolean velocityDropped = (lastStableVelocity - currentVel) > VELOCITY_DROP_THRESHOLD;
                if (velocityDropped) {
                    intake.stop();
                    recoveryTimer.reset();
                    state = State.WAIT_RECOVERY;
                } else if (atSpeed) {
                    lastStableVelocity = currentVel;
                }
                break;

            case PULSE_PAUSE:
                intake.stop();

                boolean delayedDrop = (lastStableVelocity - currentVel) > VELOCITY_DROP_THRESHOLD;
                if (delayedDrop) {
                    recoveryTimer.reset();
                    state = State.WAIT_RECOVERY;
                } else if (feedTimer.milliseconds() >= FEED_PAUSE_MS) {
                    if (atSpeed) {
                        feedTimer.reset();
                        state = State.PULSE_FEED;
                    }
                }
                break;

            case WAIT_RECOVERY:
                intake.stop();

                boolean minTimeElapsed = recoveryTimer.milliseconds() >= MIN_RECOVERY_TIME_MS;
                boolean recoveredSpeed = currentVel >= (targetVelocity - VELOCITY_TOLERANCE);

                if (minTimeElapsed && recoveredSpeed) {
                    lastStableVelocity = currentVel;
                    feedTimer.reset();
                    state = State.PULSE_FEED;
                }
                break;

            case DONE:
                intake.stop();
                break;
        }
    }

    private void setFailsafeValues() {
        if (isFarShot) {
            targetAngle = FAR_FAILSAFE_ANGLE;
            targetVelocity = FAR_FAILSAFE_VELOCITY;
        } else {
            targetAngle = CLOSE_FAILSAFE_ANGLE;
            targetVelocity = CLOSE_FAILSAFE_VELOCITY;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        shooter.stop();
        shooter.block();
    }

    public boolean usedFailsafe() {
        return usedFailsafe;
    }
}