package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;

public class ShootCommand extends CommandBase {
    private final Shooter shooter;
    private final double targetAngle;
    private final double velocity;

    public ShootCommand(Shooter shooter, double velocity, double targetAngle) {
        this.shooter = shooter;
        this.velocity = velocity;
        this.targetAngle = targetAngle;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetVelocity(velocity);
        shooter.spinUp();
        shooter.setAngle(targetAngle);
    }

    @Override
    public void end(boolean interrupted) { shooter.stop(); }
}