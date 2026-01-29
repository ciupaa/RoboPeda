package org.firstinspires.ftc.teamcode.config.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final boolean slowOuttake;
    private final double durationSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Timed intake command
     * @param intake Intake subsystem
     * @param slowOuttake true for slow outtake, false for normal intake
     * @param durationSeconds How long to run the intake motor (0 = run forever until interrupted)
     */
    public IntakeCommand(Intake intake, boolean slowOuttake, double durationSeconds) {
        this.intake = intake;
        this.slowOuttake = slowOuttake;
        this.durationSeconds = durationSeconds;
        addRequirements(intake);
    }

    /**
     * Intake command that runs indefinitely (for use in ParallelCommandGroup)
     */
    public IntakeCommand(Intake intake, boolean slowOuttake) {
        this(intake, slowOuttake, 0);
    }

    @Override
    public void initialize() {
        if (slowOuttake) {
            intake.outtakeSlow();
        } else {
            intake.intake();
        }
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        // If duration is 0, never finish (run until interrupted)
        if (durationSeconds == 0) {
            return false;
        }
        return timer.seconds() >= durationSeconds;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}