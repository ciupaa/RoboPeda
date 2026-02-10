package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.config.paths.Paths_9_RedClose;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

/**
 * FILE: Auto_9_RedClose.java
 * PURPOSE: 9-Point Autonomous for RED ALLIANCE, CLOSE side.
 *
 * Scores preload + 2 intaked balls (3 shots) and pushes 3 gate artifacts.
 * Parks outside the scoring triangle at the end.
 *
 * Sequence:
 *   SHOOT PRELOAD → INTAKE 1 → OPEN 3 GATES → SHOOT 1 → INTAKE 2 → SHOOT 2 → PARK
 */
@Autonomous(name = "9pt Red Close", group = "Competition")
public class Auto_9_RedClose extends OpModeCommand {

    Robot r;

    // Shooting parameters (consistent scoring position at ~94, 93)
    private static final double SHOOT_ANGLE = 0.65;
    private static final double SAFE_ANGLE = 1.0;

    @Override
    public void initialize() {
        // 1. Initialize Robot Hardware
        r = new Robot(hardwareMap, Alliance.RED);

        // 2. Load Paths
        Paths_9_RedClose p = new Paths_9_RedClose(r.f);

        // 3. Set Starting Pose
        r.f.setStartingPose(p.startPose);

        // 4. Schedule the Full Autonomous Sequence
        schedule(
                new SequentialCommandGroup(

                        // ====== PREP ======
                        new InstantAction(() -> r.shooter.setAngle(SAFE_ANGLE)),
                        new InstantAction(() -> r.shooter.block()),
                        new WaitCommand(200),

                        // ====== SHOOT PRELOAD ======
                        // Drive to scoring position (heading 0° → 43°)
                        new FollowPath(r, p.ShootPreload),
                        // Shoot the preloaded ball
                        new ShootCommand(r.shooter, r.intake, true, SHOOT_ANGLE),

                        // ====== INTAKE CYCLE 1 ======
                        // Approach intake zone 1
                        new InstantAction(() -> r.shooter.setAngle(SAFE_ANGLE)),
                        new FollowPath(r, p.GoTo1),
                        // Drive forward while intaking
                        new InstantAction(() -> r.intake.intake()),
                        new FollowPath(r, p.Intake1),
                        new InstantAction(() -> r.intake.stop()),

                        // ====== PUSH 3 GATE ARTIFACTS ======
                        // Push artifacts into scoring zone (robot body pushes them)
                        new FollowPath(r, p.OpenGate),
                        new FollowPath(r, p.OpenGate2),
                        new FollowPath(r, p.OpenGate3),

                        // ====== SHOOT BALL 1 ======
                        // Return to scoring position
                        new FollowPath(r, p.Shoot1),
                        // Shoot intaked ball
                        new ShootCommand(r.shooter, r.intake, true, SHOOT_ANGLE),

                        // ====== INTAKE CYCLE 2 ======
                        // Approach intake zone 2
                        new InstantAction(() -> r.shooter.setAngle(SAFE_ANGLE)),
                        new FollowPath(r, p.GoTo2),
                        // Drive forward while intaking
                        new InstantAction(() -> r.intake.intake()),
                        new FollowPath(r, p.Intake2),
                        new InstantAction(() -> r.intake.stop()),

                        // ====== SHOOT BALL 2 ======
                        // Return to scoring position
                        new FollowPath(r, p.Shoot2),
                        // Shoot intaked ball
                        new ShootCommand(r.shooter, r.intake, true, SHOOT_ANGLE),

                        // ====== PARK ======
                        // Exit scoring triangle
                        new InstantAction(() -> r.shooter.setAngle(SAFE_ANGLE)),
                        new InstantAction(() -> r.shooter.block()),
                        new FollowPath(r, p.OutOfTriangle),

                        // Final stop
                        new InstantAction(() -> r.shooter.stop()),
                        new InstantAction(() -> r.intake.stop())
                )
        );
    }

    @Override
    public void loop() {
        super.loop();   // Run the Command Scheduler
        r.periodic();   // Update Robot Hardware (Pedro, PIDs)
    }

    // Helper: Runs a single action instantly as a Command
    public static class InstantAction extends CommandBase {
        private final Runnable action;
        public InstantAction(Runnable action) { this.action = action; }
        @Override public void initialize() { action.run(); }
        @Override public boolean isFinished() { return true; }
    }
}