package org.firstinspires.ftc.teamcode.config.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;

public class ShootCommand extends CommandBase {
    private final Shooter shooter;
    private final Timer timer = new Timer();
    private boolean hasFed = false;

    public ShootCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter); // Locks the subsystem
    }

    @Override
    public void initialize() {
        shooter.spinUp(); // Start motors
        timer.resetTimer();
        hasFed = false;
    }

    @Override
    public void execute() {
        // Wait for speed OR 1.5 seconds timeout
        if (!hasFed && (shooter.isReady() || timer.getElapsedTimeSeconds() > 1.5)) {
            shooter.feed(); // Push ring into flywheel
            timer.resetTimer(); // Reset timer to count feed duration
            hasFed = true;
        }
    }

    @Override
    public boolean isFinished() {
        // End command after feeding for 0.5 seconds
        return hasFed && timer.getElapsedTimeSeconds() > 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop(); // Turn everything off
    }
}