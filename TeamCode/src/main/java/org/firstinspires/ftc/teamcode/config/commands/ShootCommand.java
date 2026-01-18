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
        addRequirements(shooter); // Locks the subsystem
    }

    @Override
    public void initialize() {
        // Set Angle & Speed
        shooter.setAngle(targetAngle);
        if (isHighShot) shooter.spinHigh();
        else shooter.spinLow();

        timer.resetTimer();
        hasFed = false;
    }

    @Override
    public void execute() {
        // Wait condition: Not yet fed AND (Speed is good OR Timeout passed)
        if (!hasFed && (Math.abs(shooter.getVelocity() - Shooter.TARGET_VELOCITY) < 50 || timer.getElapsedTimeSeconds() > 1.5)) {
            shooter.feed();
            timer.resetTimer(); // Reset timer to count feed duration
            hasFed = true;
        }
    }

    @Override
    public boolean isFinished() {
        // Finish after 0.5s of feeding
        return hasFed && timer.getElapsedTimeSeconds() > 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        // Shutdown
        shooter.stop();
        shooter.stopFeeders();
        shooter.setAngle(1.0);
    }
}