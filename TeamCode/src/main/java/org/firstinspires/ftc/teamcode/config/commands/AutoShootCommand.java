package org.firstinspires.ftc.teamcode.config.commands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Limelight_camera;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**
 * AUTO SHOOT COMMAND - WITH FAR/CLOSE FAILSAFES
 *
 * Waits for AprilTag to be visible before calculating distance and shooting.
 * If no tag is detected after timeout, uses appropriate failsafe based on auto type.
 *
 * FAILSAFES:
 * - FAR autos: 1500 velocity, 0.65 angle
 * - CLOSE autos: 1200 velocity, 0.70 angle
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

    // TAG DETECTION TIMEOUT - 5 SECONDS (tunable from Dashboard)
    public static double TAG_WAIT_TIMEOUT_SEC = 5.0;

    // FAILSAFE VALUES
    private static final double FAR_FAILSAFE_ANGLE = 0.65;
    private static final double FAR_FAILSAFE_VELOCITY = 1500;
    private static final double CLOSE_FAILSAFE_ANGLE = 0.70;
    private static final double CLOSE_FAILSAFE_VELOCITY = 1200;

    private enum State {
        WAIT_FOR_TAG,
        CALCULATE_DISTANCE,
        WAIT_SPINUP,
        PULSE_FEED,
        PULSE_PAUSE,
        WAIT_RECOVERY,
        DONE
    }
    private State state = State.WAIT_FOR_TAG;

    private double targetAngle = 0.65;
    private double targetVelocity = 1550;
    private double lastStableVelocity = 0.0;
    private boolean usedFailsafe = false;

    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final ElapsedTime tagWaitTimer = new ElapsedTime();
    private double shootStartTime = 0;

    /**
     * Constructor
     * @param shooter Shooter subsystem
     * @param intake Intake subsystem
     * @param limelight Limelight camera
     * @param shootDurationSeconds Total time to run shooting sequence
     * @param isFarShot true for FAR autos (1500/0.65), false for CLOSE autos (1200/0.70)
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
        tagWaitTimer.reset();
        state = State.WAIT_FOR_TAG;
        lastStableVelocity = 0.0;
        shootStartTime = 0;
        usedFailsafe = false;

        intake.stop();
        shooter.stop();
        shooter.block();
    }

    @Override
    public void execute() {
        double now = timer.seconds();

        switch (state) {
            case WAIT_FOR_TAG:
                intake.stop();
                shooter.stop();
                shooter.block();

                if (limelight.hasTarget()) {
                    state = State.CALCULATE_DISTANCE;
                    shootStartTime = now;
                } else if (tagWaitTimer.seconds() >= TAG_WAIT_TIMEOUT_SEC) {
                    usedFailsafe = true;
                    state = State.CALCULATE_DISTANCE;
                    shootStartTime = now;
                }
                break;

            case CALCULATE_DISTANCE:
                if (!usedFailsafe && limelight.hasTarget()) {
                    double distanceCm = limelight.getDistanceToTarget();

                    if (distanceCm > 0) {
                        ShooterCalculator_camera.ShooterConfig config =
                                ShooterCalculator_camera.getConfig(distanceCm);
                        targetAngle = config.angle;
                        targetVelocity = config.velocity;
                    } else {
                        usedFailsafe = true;
                        setFailsafeValues();
                    }
                } else {
                    usedFailsafe = true;
                    setFailsafeValues();
                }

                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                state = State.WAIT_SPINUP;
                break;

            case WAIT_SPINUP:
                if ((now - shootStartTime) >= shootDurationSeconds) {
                    state = State.DONE;
                    return;
                }

                intake.stop();

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
                if ((now - shootStartTime) >= shootDurationSeconds) {
                    state = State.DONE;
                    return;
                }

                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                if (feedTimer.milliseconds() < FEED_PULSE_MS) {
                    intake.intake();
                } else {
                    intake.stop();
                    feedTimer.reset();
                    state = State.PULSE_PAUSE;
                }

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
                if ((now - shootStartTime) >= shootDurationSeconds) {
                    state = State.DONE;
                    return;
                }

                intake.stop();

                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                double currentVel2 = shooter.getVelocity();

                boolean delayedDrop = (lastStableVelocity - currentVel2) > VELOCITY_DROP_THRESHOLD;
                if (delayedDrop) {
                    recoveryTimer.reset();
                    state = State.WAIT_RECOVERY;
                } else if (feedTimer.milliseconds() >= FEED_PAUSE_MS) {
                    boolean stillAtSpeed = currentVel2 >= (targetVelocity - VELOCITY_TOLERANCE);
                    if (stillAtSpeed) {
                        feedTimer.reset();
                        state = State.PULSE_FEED;
                    }
                }
                break;

            case WAIT_RECOVERY:
                if ((now - shootStartTime) >= shootDurationSeconds) {
                    state = State.DONE;
                    return;
                }

                intake.stop();

                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                double recoveredVel = shooter.getVelocity();
                boolean minTimeElapsed = recoveryTimer.milliseconds() >= MIN_RECOVERY_TIME_MS;
                boolean recoveredSpeed = recoveredVel >= (targetVelocity - VELOCITY_TOLERANCE);

                if (minTimeElapsed && recoveredSpeed) {
                    lastStableVelocity = recoveredVel;
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