package org.firstinspires.ftc.teamcode.config.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;

public class ShootCommand extends CommandBase {
    private final Shooter shooter;
    private final Timer timer = new Timer();
    private final double targetAngle;
    private final boolean isHighShot;
    private boolean hasFed = false;

    public ShootCommand(Shooter shooter, boolean isHighShot, double targetAngle) {
        this.shooter = shooter;
        this.isHighShot = isHighShot;
        this.targetAngle = targetAngle;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setAngle(targetAngle);
        // OPEN IMMEDIATELY
        shooter.unblock();

        if (isHighShot) shooter.spinHigh();
        else shooter.spinLow();

        timer.resetTimer();
        hasFed = false;
    }

    @Override
    public void execute() {
        // Wait 300ms + Velocity check
        if (!hasFed && timer.getElapsedTimeSeconds() > 0.3 && (Math.abs(shooter.getVelocity() - Shooter.TARGET_VELOCITY) < 50)) {
            hasFed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return hasFed && timer.getElapsedTimeSeconds() > 0.6;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.block();
        shooter.setAngle(1.0);
    }
}