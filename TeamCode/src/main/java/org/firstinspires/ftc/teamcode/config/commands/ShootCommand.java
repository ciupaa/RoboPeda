package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;

public class ShootCommand extends CommandBase {
    private final Shooter shooter;
    private final double angle;
    private final boolean high;

    public ShootCommand(Shooter shooter, boolean high, double angle) {
        this.shooter = shooter;
        this.high = high;
        this.angle = angle;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (high) shooter.spinHigh();
        else shooter.spinLow();
        shooter.setAngle(angle);
    }

    @Override
    public void end(boolean interrupted) { shooter.stop(); }
}