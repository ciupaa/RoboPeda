package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

// --- FIX: Import Robot from the 'config' package ---
import org.firstinspires.ftc.teamcode.config.Robot;

import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.config.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

@Autonomous(name = "Real Auto - Shoot & Path", group = "Competition")
public class RealAuto extends OpModeCommand {
    Robot r;

    @Override
    public void initialize() {
        // 1. Setup
        r = new Robot(hardwareMap, Alliance.BLUE);
        AutoPaths p = new AutoPaths(r);
        r.f.setStartingPose(p.startPose);

        // 2. Register Subsystems
        register(r.shooter); // Use 'r.shooter' because your Robot.java names the shooter 'shooter'
        schedule(new RunCommand(r::periodic));

        // 3. Define the Sequence
        schedule(
                new SequentialCommandGroup(
                        // --- STEP 1: Drive Line 1 while Reversing (Left Bumper Logic) ---
                        new SequentialCommandGroup(
                                // Turn on Reverse Mode
                                new InstantAction(() -> r.shooter.reverse()), // changed r.shooter to r.shooter

                                // Drive the path (Reverse stays ON while driving)
                                new FollowPath(r, p.line1),

                                // Stop Reverse Mode once path is done
                                new InstantAction(() -> r.shooter.stop())
                        ),

                        // --- STEP 2: Shoot 3 Times (Right Bumper Logic) ---
                        new SequentialCommandGroup(
                                new ShootCommand(r.shooter), // Shot 1
                                new WaitCommand(600),  // Wait 0.5s

                                new ShootCommand(r.shooter), // Shot 2
                                new WaitCommand(600),  // Wait 0.5s

                                new ShootCommand(r.shooter), // Shot 3
                                new WaitCommand(600)   // Safety Wait
                        ),

                        // --- STEP 3: Drive Path 2 ---
                        new FollowPath(r, p.path2)
                )
        );
    }

    // --- Helper Command for Instant Actions ---
    public static class InstantAction extends CommandBase {
        private final Runnable action;
        public InstantAction(Runnable action) { this.action = action; }
        @Override public void initialize() { action.run(); }
        @Override public boolean isFinished() { return true; }
    }
}