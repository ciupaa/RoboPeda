package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.subsystem.Limelight_camera;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**

 - AUTO SHOOT COMMAND - WITH CAMERA WARMUP & TAG TIMEOUT
 -
 - Flow:
 - 1. CAMERA_WARMUP (1 second) - Let camera initialize
 - 1. WAIT_FOR_TAG (15 seconds max) - Wait for AprilTag detection
 - 1. CALCULATE_DISTANCE - Use tag data to set shooter
 - 1. Shooting sequence - Normal operation
 */
@Config
public class AutoShootCommand extends CommandBase {

    private final Shooter shooter;
    private final Intake intake;
    private final Limelight_camera limelight;
    private final double shootDurationSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    // === TIMING CONFIGURATION ===

    // Camera warmup time before checking for tags
    public static double CAMERA_WARMUP_SEC = 1.0;

    // How long to wait for tag detection before proceeding
    public static double TAG_WAIT_TIMEOUT_SEC = 15.0;

    // PULSE FEEDING STRATEGY
    private static final double FEED_PULSE_MS = 120;
    private static final double FEED_PAUSE_MS = 100;
    private static final double VELOCITY_DROP_THRESHOLD = 150;
    private static final double MIN_RECOVERY_TIME_MS = 400;
    private static final double VELOCITY_TOLERANCE = 80;

    private enum State {
        CAMERA_WARMUP,
        WAIT_FOR_TAG,
        CALCULATE_DISTANCE,
        WAIT_SPINUP,
        PULSE_FEED,
        PULSE_PAUSE,
        WAIT_RECOVERY,
        DONE
    }
    private State state = State.CAMERA_WARMUP;

    private double targetAngle = 0.65;
    private double targetVelocity = 1550;
    private double lastStableVelocity = 0.0;

    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime warmupTimer = new ElapsedTime();
    private final ElapsedTime tagWaitTimer = new ElapsedTime();
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
        warmupTimer.reset();
        tagWaitTimer.reset();


        state = State.CAMERA_WARMUP;
        lastStableVelocity = 0.0;
        shootStartTime = 0;

        intake.stop();
        shooter.stop();
        shooter.block();


    }

    @Override
    public void execute() {
        double now = timer.seconds();


        switch (state) {
            case CAMERA_WARMUP:
                // Wait 1 second for camera to initialize
                intake.stop();

                if (warmupTimer.seconds() >= CAMERA_WARMUP_SEC) {
                    // Camera is ready - move to tag detection
                    state = State.WAIT_FOR_TAG;
                }
                break;

            case WAIT_FOR_TAG:
                // Wait up to 15 seconds for AprilTag detection
                intake.stop();

                if (limelight.hasTarget()) {
                    // Tag detected! Skip timeout and proceed
                    state = State.CALCULATE_DISTANCE;
                    shootStartTime = now;
                } else if (tagWaitTimer.seconds() >= TAG_WAIT_TIMEOUT_SEC) {
                    // Timeout reached - proceed anyway (will use existing fallback logic)
                    state = State.CALCULATE_DISTANCE;
                    shootStartTime = now;
                }
                break;

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
                        // Fallback (existing logic unchanged)
                        targetAngle = 0.7;
                        targetVelocity = 1200;
                    }
                } else {
                    // No tag - use fallback (existing logic unchanged)
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
                // Check if we've exceeded total shoot duration
                if ((now - shootStartTime) >= shootDurationSeconds) {
                    state = State.DONE;
                    return;
                }

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
                // Check if we've exceeded total shoot duration
                if ((now - shootStartTime) >= shootDurationSeconds) {
                    state = State.DONE;
                    return;
                }

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
                // Check if we've exceeded total shoot duration
                if ((now - shootStartTime) >= shootDurationSeconds) {
                    state = State.DONE;
                    return;
                }

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
                // Check if we've exceeded total shoot duration
                if ((now - shootStartTime) >= shootDurationSeconds) {
                    state = State.DONE;
                    return;
                }

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