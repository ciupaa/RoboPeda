package org.firstinspires.ftc.teamcode.config.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;

public class ShootCommand extends CommandBase {

    private final Shooter shooter;
    private final Intake intake;

    private final Timer timer = new Timer();

    private final double targetAngle;
    private final boolean isHighShot;

    // Tuning (mirror TeleOp logic)
    private static final double VELOCITY_TOLERANCE = 50;
    private static final double STABLE_SEC = 0.12;
    private static final double FEED_SEC = 0.14;

    private enum State { SPINUP_STABLE, FEEDING, DONE }
    private State state = State.SPINUP_STABLE;

    private double readySinceSec = -1;
    private double feedStartSec = -1;

    public ShootCommand(Shooter shooter, Intake intake, boolean isHighShot, double targetAngle) {
        this.shooter = shooter;
        this.intake = intake;
        this.isHighShot = isHighShot;
        this.targetAngle = targetAngle;

        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        shooter.setAngle(targetAngle);

        // Your preference: open immediately
        shooter.unblock();

        if (isHighShot) shooter.spinHigh();
        else shooter.spinLow();

        intake.stop();

        timer.resetTimer();
        state = State.SPINUP_STABLE;
        readySinceSec = -1;
        feedStartSec = -1;
    }

    @Override
    public void execute() {
        // Keep these “held”
        shooter.setAngle(targetAngle);
        shooter.unblock();
        if (isHighShot) shooter.spinHigh();
        else shooter.spinLow();

        double now = timer.getElapsedTimeSeconds();
        double targetVel = isHighShot ? 1550 : 1200;

        boolean velocityReady = shooter.getVelocity() >= (targetVel - VELOCITY_TOLERANCE);

        switch (state) {
            case SPINUP_STABLE:
                intake.stop();

                if (velocityReady) {
                    if (readySinceSec < 0) readySinceSec = now;
                    if ((now - readySinceSec) >= STABLE_SEC) {
                        feedStartSec = now;
                        state = State.FEEDING;
                    }
                } else {
                    readySinceSec = -1;
                }
                break;

            case FEEDING:
                if ((now - feedStartSec) <= FEED_SEC) {
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

        // On release/end, go back to safe closed
        shooter.block();
    }
}
