package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

// A simple command to run the intake.
public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final boolean slowOuttake;

    public IntakeCommand(Intake intake, boolean slowOuttake) {
        this.intake = intake;
        this.slowOuttake = slowOuttake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (slowOuttake) intake.outtakeSlow();
        else intake.intake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}