package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.config.paths.Paths_9_BlueClose_New2;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

/**
 * FILE: Auto_9_BlueClose_New2.java
 * PURPOSE: 9-Point Autonomous – BLUE ALLIANCE, CLOSE side, NEW2 path sequence.
 *
 * Uses Paths_9_BlueClose_New2 (updated coordinates from 9_AutoBlue_Close_(1).pp).
 * Logic identical to Auto_9_BlueClose_New.
 *
 * CAMERA PIPELINE: 1 (AprilTag – Blue target TAG 20)
 *
 * Sequence:
 *   1. PREP          – Safe angle lock, limelight warmup
 *   2. ShootPreload  – Drive to scoring position, shoot preload
 *   3. GoTo2         – Drive to artifact 2 approach, intake on
 *   4. Intake2       – Drive through artifact 2 while intaking
 *   5. AlignToGate   – Align to gate artifact position
 *   6. PushGate      – Push gate artifact
 *   7. Shoot2        – Drive back to scoring position, shoot
 *   8. GoTo1         – Drive to artifact 1 approach, intake on
 *   9. Intake1       – Drive through artifact 1 while intaking
 *  10. Shoot1        – Drive back to scoring position, shoot
 *  11. Park          – Drive to park
 */
@Autonomous(name = "9pt Blue Close NEW2", group = "Competition")
public class Auto_9_BlueClose_New2 extends OpModeCommand {

    private Robot_camera r;

    @Override
    public void initialize() {
        // 1. Initialize Robot with Camera – BLUE alliance
        r = new Robot_camera(hardwareMap, Alliance.BLUE);

        // 2. Build paths
        Paths_9_BlueClose_New2 p = new Paths_9_BlueClose_New2(r);

        // 3. Set starting pose for Pedro
        r.f.setStartingPose(p.startPose);

        // 4. Schedule full sequence
        schedule(
                new SequentialCommandGroup(

                        // ── STEP 1: PREP ──────────────────────────────────────────
                        new InstantAction(() -> r.shooter.setAngle(1.0)),
                        new InstantAction(() -> r.shooter.block()),
                        new WaitCommand(1000), // Limelight warmup

                        // ── STEP 2: DRIVE TO SHOOTING POSITION (preload) ──────────
                        new FollowPath(r, p.ShootPreload),

                        // ── STEP 3: SHOOT PRELOAD ─────────────────────────────────
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 2.7, false),  // CLOSE SHOT

                        // ── STEP 4: DRIVE TO ARTIFACT 2, INTAKE ON ───────────────
                        new InstantAction(() -> r.shooter.setAngle(1.0)),
                        new InstantAction(() -> r.shooter.block()),
                        new ParallelCommandGroup(
                                new FollowPath(r, p.GoTo2),
                                new InstantAction(() -> r.intake.intake())
                        ),

                        // ── STEP 5: INTAKE ARTIFACT 2 ────────────────────────────
                        new ParallelCommandGroup(
                                new FollowPath(r, p.Intake2),
                                new InstantAction(() -> r.intake.intake())
                        ),
                        // Extra intake time after path finishes
                        new IntakeCommand(r.intake, false, 0.8),

                        // ── STEP 6: ALIGN TO GATE ────────────────────────────────
                        new InstantAction(() -> r.intake.stop()),
                        new FollowPath(r, p.AllignToGate),

                        // ── STEP 7: PUSH GATE ARTIFACT ───────────────────────────
                        new FollowPath(r, p.PushGate),

                        // ── STEP 8: DRIVE TO SCORING, SHOOT 2 ────────────────────
                        new FollowPath(r, p.Shoot2),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 2.7, false),  // CLOSE SHOT

                        // ── STEP 9: DRIVE TO ARTIFACT 1, INTAKE ON ───────────────
                        new InstantAction(() -> r.shooter.setAngle(1.0)),
                        new InstantAction(() -> r.shooter.block()),
                        new ParallelCommandGroup(
                                new FollowPath(r, p.GoTo1),
                                new InstantAction(() -> r.intake.intake())
                        ),

                        // ── STEP 10: INTAKE ARTIFACT 1 ───────────────────────────
                        new ParallelCommandGroup(
                                new FollowPath(r, p.Intake1),
                                new InstantAction(() -> r.intake.intake())
                        ),
                        // Extra intake time after path finishes
                        new IntakeCommand(r.intake, false, 0.8),

                        // ── STEP 11: DRIVE TO SCORING, SHOOT 1 ───────────────────
                        new InstantAction(() -> r.intake.stop()),
                        new FollowPath(r, p.Shoot1),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 2.7, false),  // CLOSE SHOT

                        // ── STEP 12: PARK ─────────────────────────────────────────
                        new InstantAction(() -> r.shooter.setAngle(1.0)),
                        new InstantAction(() -> r.shooter.block()),
                        new FollowPath(r, p.Park)
                )
        );
    }

    @Override
    public void loop() {
        super.loop();
        r.periodic();
    }

    public static class InstantAction extends CommandBase {
        private final Runnable action;
        public InstantAction(Runnable action) { this.action = action; }
        @Override public void initialize() { action.run(); }
        @Override public boolean isFinished() { return true; }
    }
}