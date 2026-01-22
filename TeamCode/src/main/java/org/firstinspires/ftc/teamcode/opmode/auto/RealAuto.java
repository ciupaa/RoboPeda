package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.config.paths.AutoPaths;
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
        AutoPaths p = new AutoPaths(r);

        // 3. Tell Pedro Pathing where we start
        r.f.setStartingPose(p.startPose);

        // 4. Schedule the Sequence
        schedule(
                new SequentialCommandGroup(

                        // STEP 1: PREP
                        // Lock shooter to safe angle
                        new InstantAction(() -> r.shooter.setAngle(1.0)),
                        new WaitCommand(500),

                        // STEP 2: DRIVE PATH 1
                        // Accessing the path from the nested 'paths' class
                        new FollowPath(r, p.paths.Path1),

                        // STEP 3: SHOOT
                        // High Goal, Angle 0.65
                        new ShootCommand(r.shooter, r.intake, true, 0.65),

                        // STEP 4: DRIVE PATH 3
                        new FollowPath(r, p.paths.Path3),

                        // STEP 5: INTAKE SEQUENCE
                        // Drop nozzle -> Start Motor -> Wait 1.5s -> Stop
                        new InstantAction(() -> r.shooter.setAngle(1.0)),
                        new InstantAction(() -> r.intake.intake()),
                        new WaitCommand(1500),
                        new InstantAction(() -> r.intake.stop()),

                        // STEP 6: DRIVE PATH 2 (Return)
                        new FollowPath(r, p.paths.Path2),

                        // STEP 7: SCORE
                        new InstantAction(() -> r.intake.outtakeSlow()),
                        new WaitCommand(1000),
                        new InstantAction(() -> r.intake.stop())
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