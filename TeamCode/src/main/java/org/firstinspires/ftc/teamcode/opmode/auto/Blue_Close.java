package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.config.paths.Blue_close;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

/**
 * FILE: Blue_Close.java
 * PURPOSE: Blue Alliance Close Position Autonomous
 *
 * SEQUENCE:
 * 1. Drive to goal (preload)
 * 2. Shoot preload artifact (distance-based, 3 seconds)
 * 3. Drive to artifact 1 position
 * 4. Intake for 4 seconds at artifact 1
 * 5. Drive to goal while prepping shooter
 * 6. Shoot artifact 1 (distance-based, 3 seconds)
 * 7. Drive to artifact 2 position
 * 8. Intake for 4 seconds at artifact 2
 * 9. Drive to goal while prepping shooter
 * 10. Shoot artifact 2 (distance-based, 3 seconds)
 */
@Autonomous(name = "Blue Close", group = "Competition")
public class Blue_Close extends OpModeCommand {

    Robot_camera r;

    @Override
    public void initialize() {
        // 1. Initialize Robot Hardware WITH CAMERA (Blue Alliance, Pipeline 1)
        r = new Robot_camera(hardwareMap, Alliance.BLUE);

        // 2. Load the Path Map
        Blue_close p = new Blue_close(r);

        // 3. Tell Pedro Pathing where we start
        r.f.setStartingPose(p.startPose);

        // 4. Schedule the Sequence
        schedule(
                new SequentialCommandGroup(

                        // ========== PRELOAD ARTIFACT SEQUENCE ==========

                        // STEP 1: Drive to goal + prep shooter
                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.setAngle(0.65)),    // Set initial angle
                                new InstantAction(() -> r.shooter.unblock()),          // Open blocker
                                new FollowPath(r, p.paths.Shootpreload)
                        ),

                        // STEP 2: Shoot preload artifact (distance-based, 3 seconds max)
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 3.0),

                        // STEP 3: Close blocker after shooting
                        new InstantAction(() -> r.shooter.block()),

                        // ========== ARTIFACT 1 SEQUENCE ==========

                        // STEP 4: Drive to artifact 1 position
                        new FollowPath(r, p.paths.GoTo1),

                        // STEP 5: Arrive at artifact 1 + intake for 4 seconds
                        new ParallelCommandGroup(
                                new IntakeCommand(r.intake, false, 4.0),  // Intake for 4 seconds
                                new FollowPath(r, p.paths.Intake1)         // Continue to exact position
                        ),

                        // STEP 6: Return to goal + prep shooter
                        // Intake auto-stops after 4 seconds, blocker opens
                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.unblock()),  // Open blocker
                                new FollowPath(r, p.paths.Shoot1)
                                // Angle will be calculated by AutoShootCommand based on distance
                        ),

                        // STEP 7: Shoot artifact 1 (distance-based, 3 seconds max)
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 3.0),

                        // STEP 8: Close blocker after shooting
                        new InstantAction(() -> r.shooter.block()),

                        // ========== ARTIFACT 2 SEQUENCE ==========

                        // STEP 9: Drive to artifact 2 position
                        new FollowPath(r, p.paths.GoTo2),

                        // STEP 10: Arrive at artifact 2 + intake for 4 seconds
                        new ParallelCommandGroup(
                                new IntakeCommand(r.intake, false, 4.0),  // Intake for 4 seconds
                                new FollowPath(r, p.paths.Intake2)         // Continue to exact position
                        ),

                        // STEP 11: Return to goal + prep shooter
                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.unblock()),  // Open blocker
                                new FollowPath(r, p.paths.Shoot2)
                        ),

                        // STEP 12: Shoot artifact 2 (distance-based, 3 seconds max)
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 3.0),

                        // STEP 13: Close blocker and safe state
                        new InstantAction(() -> {
                            r.shooter.block();
                            r.shooter.stop();
                            r.intake.stop();
                        })
                )
        );
    }

    @Override
    public void loop() {
        super.loop();   // Run the Command Scheduler
        r.periodic();   // Update Robot Hardware (Pedro, PIDs, Limelight)
    }

    // Helper Class: Runs a single line of code instantly as a Command
    public static class InstantAction extends CommandBase {
        private final Runnable action;
        public InstantAction(Runnable action) { this.action = action; }
        @Override public void initialize() { action.run(); }
        @Override public boolean isFinished() { return true; }
    }
}