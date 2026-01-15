package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final boolean reverse;

    public IntakeCommand(Intake intake, boolean reverse) {
        this.intake = intake;
        this.reverse = reverse;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (reverse) intake.outtake();
        else intake.intake();
    }

    @Override
    public void end(boolean interrupted) { intake.stop(); }
}