package org.firstinspires.ftc.teamcode.config.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Limelight_camera;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**
 * AUTO SHOOT COMMAND - FIXED FOR MULTIPLE BALLS
 *
 * Shoots continuously for specified duration using pulse feeding strategy.
 * Identical to TeleOp shooting logic but time-limited.
 */
public class AutoShootCommand extends CommandBase {

    private final Shooter shooter;
    private final Intake intake;
    private final Limelight_camera limelight;
    private final double shootDurationSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    // PULSE FEEDING STRATEGY (same as TeleOp)
    private static final double FEED_PULSE_MS = 120;
    private static final double FEED_PAUSE_MS = 100;
    private static final double VELOCITY_DROP_THRESHOLD = 150;
    private static final double MIN_RECOVERY_TIME_MS = 400;

    // VELOCITY READY THRESHOLD
    private static final double VELOCITY_TOLERANCE = 80;

    private enum State {
        CALCULATE_DISTANCE,
        WAIT_SPINUP,
        PULSE_FEED,
        PULSE_PAUSE,
        WAIT_RECOVERY,
        DONE
    }
    private State state = State.CALCULATE_DISTANCE;

    private double targetAngle = 0.65;
    private double targetVelocity = 1550;
    private double lastStableVelocity = 0.0;

    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private double shootStartTime = 0;

    public AutoShootCommand(Shooter shooter, Intake intake, Limelight_camera limelight,
                            double shootDurationSeconds) {
        this.shooter = shooter;
        this.intake = intake;
        this.limelight = limelight;
        this.shootDurationSeconds = shootDurationSeconds;

        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        feedTimer.reset();
        recoveryTimer.reset();
        state = State.CALCULATE_DISTANCE;
        lastStableVelocity = 0.0;
        shootStartTime = timer.seconds();
    }

    @Override
    public void execute() {
        double now = timer.seconds();

        // Check if we've exceeded total shoot duration
        if ((now - shootStartTime) >= shootDurationSeconds) {
            state = State.DONE;
            return;
        }

        switch (state) {
            case CALCULATE_DISTANCE:
                // Use Limelight to calculate optimal angle and velocity
                if (limelight.hasTarget()) {
                    double distanceCm = limelight.getDistanceToTarget();

                    if (distanceCm > 0) {
                        ShooterCalculator_camera.ShooterConfig config =
                                ShooterCalculator_camera.getConfig(distanceCm);

                        targetAngle = config.angle;
                        targetVelocity = config.velocity;
                    } else {
                        // Fallback
                        targetAngle = 0.7;
                        targetVelocity = 1200;
                    }
                } else {
                    // No target - use default
                    targetAngle = 0.7;
                    targetVelocity = 1200;
                }

                // Set shooter configuration
                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                state = State.WAIT_SPINUP;
                break;

            case WAIT_SPINUP:
                intake.stop();

                // Keep settings applied
                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                double currentVel = shooter.getVelocity();
                boolean atSpeed = currentVel >= (targetVelocity - VELOCITY_TOLERANCE);

                if (atSpeed) {
                    lastStableVelocity = currentVel;
                    feedTimer.reset();
                    state = State.PULSE_FEED;
                }
                break;

            case PULSE_FEED:
                // Keep settings applied
                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                // SHORT PULSE
                if (feedTimer.milliseconds() < FEED_PULSE_MS) {
                    intake.intake();
                } else {
                    intake.stop();
                    feedTimer.reset();
                    state = State.PULSE_PAUSE;
                }

                // Check for ball launch (velocity drop)
                double currentVelocity = shooter.getVelocity();
                boolean velocityDropped = (lastStableVelocity - currentVelocity) > VELOCITY_DROP_THRESHOLD;

                if (velocityDropped) {
                    intake.stop();
                    recoveryTimer.reset();
                    state = State.WAIT_RECOVERY;
                } else if (currentVelocity >= (targetVelocity - VELOCITY_TOLERANCE)) {
                    lastStableVelocity = currentVelocity;
                }
                break;

            case PULSE_PAUSE:
                intake.stop();

                // Keep settings applied
                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                double currentVel2 = shooter.getVelocity();

                // Check for delayed detection
                boolean delayedDrop = (lastStableVelocity - currentVel2) > VELOCITY_DROP_THRESHOLD;
                if (delayedDrop) {
                    recoveryTimer.reset();
                    state = State.WAIT_RECOVERY;
                }
                // Pulse again if no ball detected
                else if (feedTimer.milliseconds() >= FEED_PAUSE_MS) {
                    boolean stillAtSpeed = currentVel2 >= (targetVelocity - VELOCITY_TOLERANCE);
                    if (stillAtSpeed) {
                        feedTimer.reset();
                        state = State.PULSE_FEED;
                    }
                }
                break;

            case WAIT_RECOVERY:
                intake.stop();

                // Keep settings applied
                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                double recoveredVel = shooter.getVelocity();
                boolean minTimeElapsed = recoveryTimer.milliseconds() >= MIN_RECOVERY_TIME_MS;
                boolean recoveredSpeed = recoveredVel >= (targetVelocity - VELOCITY_TOLERANCE);

                if (minTimeElapsed && recoveredSpeed) {
                    lastStableVelocity = recoveredVel;
                    feedTimer.reset();
                    state = State.PULSE_FEED;  // Continue shooting more balls!
                }
                break;

            case DONE:
                intake.stop();
                break;
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
}