package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.config.paths.Paths_9_BlueClose_New;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**
 * FILE: Auto_9_BlueClose_New.java
 * PURPOSE: 9-Point Autonomous – BLUE ALLIANCE, CLOSE side, NEW path sequence.
 *
 * NEW SEQUENCE (vs original 9pt Blue Close):
 *   Preload shot → Artifact 2 intake → Gate push → Shoot 2 → Artifact 1 intake → Shoot 1 → Park
 *
 * Uses Limelight camera for distance-based angle/velocity calculation on shots 2 and 3.
 * ShootCommand timing mirrors existing competition autos.
 *
 * CAMERA PIPELINE: 1 (AprilTag – Blue target TAG 20)
 *
 * Step-by-step:
 *   1. PREP          – Safe angle lock, limelight warmup
 *   2. ShootPreload  – Drive to scoring position, shoot preload (camera distance)
 *   3. GoTo2         – Drive to artifact 2 approach, intake on
 *   4. Intake2       – Drive through artifact 2 while intaking
 *   5. AlignToGate   – Align to gate artifact position
 *   6. PushGate      – Push gate artifact, intake off
 *   7. Shoot2        – Drive back to scoring position, shoot (camera distance)
 *   8. GoTo1         – Drive to artifact 1 approach, intake on
 *   9. Intake1       – Drive through artifact 1 while intaking
 *  10. Shoot1        – Drive back to scoring position, shoot (camera distance)
 *  11. Park          – Drive to park
 */
@Autonomous(name = "9pt Blue Close NEW", group = "Competition")
public class Auto_9_BlueClose_New extends OpModeCommand {

    private Robot_camera r;

    @Override
    public void initialize() {
        // 1. Initialize Robot with Camera – BLUE alliance
        r = new Robot_camera(hardwareMap, Alliance.BLUE);

        // 2. Build paths
        Paths_9_BlueClose_New p = new Paths_9_BlueClose_New(r);

        // 3. Set starting pose for Pedro
        r.f.setStartingPose(p.startPose);

        // 4. Schedule full sequence
        schedule(
                new SequentialCommandGroup(

                        // ── STEP 1: PREP ──────────────────────────────────────────
                        // Lock angle to safe travel position, give Limelight time to initialize
                        new InstantAction(() -> r.shooter.setAngle(1.0)),
                        new InstantAction(() -> r.shooter.block()),
                        new WaitCommand(1000), // Limelight warmup

                        // ── STEP 2: DRIVE TO SHOOTING POSITION (preload) ──────────
                        // Drive the ShootPreload bezier curve while camera acquires target
                        new FollowPath(r, p.ShootPreload),

                        // ── STEP 3: SHOOT PRELOAD ─────────────────────────────────
                        // Camera-based distance shot; fallback to 0.65 if no target
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 4, false),  // CLOSE SHOT

                        // ── STEP 4: DRIVE TO ARTIFACT 2, INTAKE ON ───────────────
                        // Lower angle to intake position and start intake concurrently with drive
                        new InstantAction(() -> r.shooter.setAngle(1.0)),
                        new InstantAction(() -> r.shooter.block()),
                        new ParallelCommandGroup(
                                new FollowPath(r, p.GoTo2),
                                new InstantAction(() -> r.intake.intake())
                        ),

                        // ── STEP 5: INTAKE ARTIFACT 2 ────────────────────────────
                        // Drive straight through artifact while intake spins
                        new ParallelCommandGroup(
                                new FollowPath(r, p.Intake2),
                                new InstantAction(() -> r.intake.intake())
                        ),

                        // ── STEP 6: ALIGN TO GATE ────────────────────────────────
                        // Stop intake, navigate to gate alignment position
                        new InstantAction(() -> r.intake.stop()),
                        new FollowPath(r, p.AlignToGate),

                        // ── STEP 7: PUSH GATE ARTIFACT ───────────────────────────
                        new FollowPath(r, p.PushGate),

                        // ── STEP 8: DRIVE TO SCORING, SHOOT 2 ────────────────────
                        new FollowPath(r, p.Shoot2),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 4, false),  // CLOSE SHOT

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

                        // ── STEP 11: DRIVE TO SCORING, SHOOT 1 ───────────────────
                        new InstantAction(() -> r.intake.stop()),
                        new FollowPath(r, p.Shoot1),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 4, false),  // CLOSE SHOT

                        // ── STEP 12: PARK ─────────────────────────────────────────
                        new InstantAction(() -> r.shooter.setAngle(1.0)),
                        new InstantAction(() -> r.shooter.block()),
                        new FollowPath(r, p.Park)
                )
        );
    }

    @Override
    public void loop() {
        super.loop();   // Run the Command Scheduler
        r.periodic();   // Update Robot hardware (Pedro, PIDs, Limelight)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // HELPER: Instant action command (single lambda, finishes immediately)
    // ─────────────────────────────────────────────────────────────────────────
    public static class InstantAction extends CommandBase {
        private final Runnable action;
        public InstantAction(Runnable action) { this.action = action; }
        @Override public void initialize() { action.run(); }
        @Override public boolean isFinished() { return true; }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // HELPER: Camera-based shoot command
    //
    // Uses Limelight distance measurement to compute angle/velocity via
    // ShooterCalculator_camera. Falls back to 0.65 / 1550 if no target seen
    // within CAMERA_WAIT_MS milliseconds.
    //
    // State machine mirrors ShootCommand (SPINUP_STABLE → FEEDING → DONE)
    // with the same tolerances used in competition TeleOp and RealAuto.
    //
    // Declared static (like InstantAction) so Java allows static final
    // constants and the State enum inside this nested class.
    // ─────────────────────────────────────────────────────────────────────────
    public static class ShootWithCamera extends CommandBase {

        private final Robot_camera robot;
        private final boolean isHighShot;

        // Timing constants – mirrors ShootCommand and competition TeleOp
        private static final double VELOCITY_TOLERANCE = 50;
        private static final double STABLE_SEC         = 0.12;
        private static final double FEED_SEC           = 0.14;
        private static final long   CAMERA_WAIT_MS     = 5000; // max ms to wait for target

        // Fallback values if camera has no target
        private static final double FALLBACK_ANGLE    = 0.65;
        private static final double FALLBACK_VEL_HIGH = 1550;
        private static final double FALLBACK_VEL_LOW  = 1200;

        private enum State { WAIT_CAMERA, SPINUP_STABLE, FEEDING, DONE }
        private State state = State.WAIT_CAMERA;

        private double targetAngle;
        private double targetVel;

        private long   cameraWaitStart;
        private double readySinceSec = -1;
        private double feedStartSec  = -1;

        public ShootWithCamera(Robot_camera robot, boolean isHighShot) {
            this.robot      = robot;
            this.isHighShot = isHighShot;
            addRequirements(robot.shooter, robot.intake);
        }

        @Override
        public void initialize() {
            // Set safe defaults immediately in case camera never gets a target
            targetAngle = FALLBACK_ANGLE;
            targetVel   = isHighShot ? FALLBACK_VEL_HIGH : FALLBACK_VEL_LOW;

            robot.shooter.unblock();
            robot.intake.stop();

            state           = State.WAIT_CAMERA;
            cameraWaitStart = System.currentTimeMillis();
            readySinceSec   = -1;
            feedStartSec    = -1;
        }

        @Override
        public void execute() {
            double nowSec = System.currentTimeMillis() / 1000.0;

            switch (state) {

                case WAIT_CAMERA:
                    // Wait up to CAMERA_WAIT_MS for a valid Limelight target
                    robot.intake.stop();
                    if (robot.limelight.hasTarget()) {
                        double dist = robot.limelight.getDistanceToTarget();
                        ShooterCalculator_camera.ShooterConfig cfg =
                                ShooterCalculator_camera.getConfig(dist);
                        targetAngle = cfg.angle;
                        targetVel   = cfg.velocity;
                        state       = State.SPINUP_STABLE;
                    } else if ((System.currentTimeMillis() - cameraWaitStart) > CAMERA_WAIT_MS) {
                        // No target seen – fall back to defaults and shoot anyway
                        state = State.SPINUP_STABLE;
                    }
                    // Start spinning immediately while waiting for camera
                    robot.shooter.setAngle(targetAngle);
                    robot.shooter.unblock();
                    robot.shooter.launcher.setVelocity(targetVel);
                    break;

                case SPINUP_STABLE:
                    robot.intake.stop();
                    robot.shooter.setAngle(targetAngle);
                    robot.shooter.unblock();
                    robot.shooter.launcher.setVelocity(targetVel);

                    boolean velocityReady =
                            robot.shooter.getVelocity() >= (targetVel - VELOCITY_TOLERANCE);
                    if (velocityReady) {
                        if (readySinceSec < 0) readySinceSec = nowSec;
                        if ((nowSec - readySinceSec) >= STABLE_SEC) {
                            feedStartSec = nowSec;
                            state        = State.FEEDING;
                        }
                    } else {
                        readySinceSec = -1;
                    }
                    break;

                case FEEDING:
                    robot.shooter.setAngle(targetAngle);
                    robot.shooter.unblock();
                    robot.shooter.launcher.setVelocity(targetVel);

                    if ((nowSec - feedStartSec) <= FEED_SEC) {
                        robot.intake.intake();
                    } else {
                        robot.intake.stop();
                        state = State.DONE;
                    }
                    break;

                case DONE:
                    robot.intake.stop();
                    break;
            }
        }

        @Override
        public boolean isFinished() { return state == State.DONE; }

        @Override
        public void end(boolean interrupted) {
            robot.intake.stop();
            robot.shooter.stop();
            robot.shooter.block();
        }
    }
}