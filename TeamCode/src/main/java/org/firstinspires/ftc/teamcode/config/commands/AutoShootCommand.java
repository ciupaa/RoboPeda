package org.firstinspires.ftc.teamcode.config.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Limelight_camera;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**
 * AUTO SHOOT COMMAND - Distance-Based with Timed Duration
 *
 * Uses Limelight to calculate optimal angle and velocity,
 * then shoots for a specified duration.
 */
public class AutoShootCommand extends CommandBase {

    private final Shooter shooter;
    private final Intake intake;
    private final Limelight_camera limelight;
    private final double shootDurationSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    // Tuning
    private static final double VELOCITY_TOLERANCE = 80;
    private static final double STABLE_TIME_SEC = 0.15;
    private static final double FEED_TIME_SEC = 0.14;

    private enum State {
        CALCULATE_DISTANCE,
        SPINUP_STABLE,
        FEEDING,
        DONE
    }
    private State state = State.CALCULATE_DISTANCE;

    private double targetAngle = 0.65;
    private double targetVelocity = 1550;
    private double readySinceSec = -1;
    private double feedStartSec = -1;
    private double shootStartSec = -1;

    /**
     * @param shooter Shooter subsystem
     * @param intake Intake subsystem
     * @param limelight Limelight camera for distance measurement
     * @param shootDurationSeconds Total time to spend shooting (including spinup)
     */
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
        state = State.CALCULATE_DISTANCE;
        readySinceSec = -1;
        feedStartSec = -1;
        shootStartSec = timer.seconds();
    }

    @Override
    public void execute() {
        double now = timer.seconds();

        // Check if we've exceeded total shoot duration
        if ((now - shootStartSec) >= shootDurationSeconds) {
            state = State.DONE;
            return;
        }

        switch (state) {
            case CALCULATE_DISTANCE:
                // Use Limelight to calculate optimal angle and velocity
                if (limelight.hasTarget()) {
                    double distanceCm = limelight.getDistanceToTarget();

                    if (distanceCm > 0) {
                        // Use regression equations
                        ShooterCalculator_camera.ShooterConfig config =
                                ShooterCalculator_camera.getConfig(distanceCm);

                        targetAngle = config.angle;
                        targetVelocity = config.velocity;
                    } else {
                        // Fallback if distance invalid
                        targetAngle = 0.7;
                        targetVelocity = 1200;
                    }
                } else {
                    // No target - use default high shot
                    targetAngle = 0.7;
                    targetVelocity = 1200;
                }

                // Set shooter configuration
                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                state = State.SPINUP_STABLE;
                break;

            case SPINUP_STABLE:
                intake.stop();

                // Keep settings applied
                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                boolean velocityReady = shooter.getVelocity() >= (targetVelocity - VELOCITY_TOLERANCE);

                if (velocityReady) {
                    if (readySinceSec < 0) readySinceSec = now;
                    if ((now - readySinceSec) >= STABLE_TIME_SEC) {
                        feedStartSec = now;
                        state = State.FEEDING;
                    }
                } else {
                    readySinceSec = -1;
                }
                break;

            case FEEDING:
                // Keep settings applied
                shooter.setAngle(targetAngle);
                shooter.unblock();
                shooter.launcher.setVelocity(targetVelocity);

                if ((now - feedStartSec) <= FEED_TIME_SEC) {
                    intake.intake();
                } else {
                    intake.stop();
                    state = State.DONE;
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
        shooter.block(); // Close blocker when done
    }
}