package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

/**
 * Runs intake continuously until interrupted by another command.
 * Designed to run in parallel with path following - will stop automatically
 * when the parallel group finishes.
 */
public class ContinuousIntakeCommand extends CommandBase {
    private final Intake intake;
    private final boolean slowOuttake;

    public ContinuousIntakeCommand(Intake intake, boolean slowOuttake) {
        this.intake = intake;
        this.slowOuttake = slowOuttake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (slowOuttake) {
            intake.outtakeSlow();
        } else {
            intake.intake();
        }
    }

    @Override
    public boolean isFinished() {
        // Never finishes on its own - only when interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the intake when this command ends
        intake.stop();
    }
}