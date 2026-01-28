package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.config.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.config.paths.Blue_close;
import org.firstinspires.ftc.teamcode.config.paths.Blue_close;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

/**
 * FILE: RealAuto.java
 * PURPOSE: The "Director" of the Autonomous period.
 * It queues up actions in a specific order: Drive -> Shoot -> Drive -> Intake.
 */
@Autonomous(name = "FULL SYSTEM TEST", group = "Competition")
public class RealAuto extends OpModeCommand {

    Robot r;

    @Override
    public void initialize() {
        // 1. Initialize Robot Hardware
        r = new Robot(hardwareMap, Alliance.BLUE);

        // 2. Load the Path Map
        Blue_close p = new Blue_close(r);

        // 3. Tell Pedro Pathing where we start
        r.f.setStartingPose(p.startPose);

        // 4. Schedule the Sequence
        schedule(
                new SequentialCommandGroup(

                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.setAngle(0.65)),
                                new FollowPath(r, p.paths.Shootpreload),
                                new InstantAction(() -> r.shooter.unblock())

                        ),
                        new ShootCommand(r.shooter, r.intake, false, 0.65),
                        //new WaitCommand(5000),

                        new ParallelCommandGroup(
                                new IntakeCommand(r.intake, false),
                                new InstantAction(() -> r.shooter.block()),
                                new FollowPath(r, p.paths.GoTo1)

                                ),
                        new FollowPath(r, p.paths.Intake1),

                        new ParallelCommandGroup(
                                new InstantAction(() -> r.intake.stop()),
                                new FollowPath(r, p.paths.Shoot1),
                                new InstantAction(() -> r.shooter.setAngle(0.65))
                        ),
                        new ShootCommand(r.shooter, r.intake, false, 0.65),
                        //new WaitCommand(5000),


                        new ParallelCommandGroup(
                                new FollowPath(r, p.paths.GoTo2),
                                new IntakeCommand(r.intake, false),
                                new InstantAction(() -> r.shooter.block())
                        ),

                        new FollowPath(r, p.paths.Intake2),

                        new ParallelCommandGroup(
                                new InstantAction(() -> r.intake.stop()),
                                new FollowPath(r, p.paths.Shoot2),
                                new InstantAction(() -> r.shooter.setAngle(0.65))
                        ),
                        new ShootCommand(r.shooter, r.intake, false, 0.65)
                        //new WaitCommand(5000)
                )
        );
    }

    @Override
    public void loop() {
        super.loop();   // Run the Command Scheduler
        r.periodic();   // Update Robot Hardware (Pedro, PIDs)
    }

    // Helper Class: Runs a single line of code instantly as a Command
    public static class InstantAction extends CommandBase {
        private final Runnable action;
        public InstantAction(Runnable action) { this.action = action; }
        @Override public void initialize() { action.run(); }
        @Override public boolean isFinished() { return true; }
    }
}