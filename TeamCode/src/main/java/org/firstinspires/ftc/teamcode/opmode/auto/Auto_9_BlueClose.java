package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.config.paths.Paths_9_BlueClose;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera;

/**
 * FILE: Auto_9_BlueClose.java
 * PURPOSE: 9-Point Autonomous for BLUE ALLIANCE, CLOSE side.
 *
 * Scores preload + 2 intaked balls (3 shots) and pushes 3 gate artifacts.
 * Parks outside the scoring triangle at the end.
 *
 * Uses Robot_camera + AutoShootCommand (Limelight distance-based shooting)
 * Same logic/timing as Blue_Close working autonomous.
 *
 * Sequence:
 *   SHOOT PRELOAD → INTAKE 1 → OPEN 3 GATES → SHOOT 1 → INTAKE 2 → SHOOT 2 → PARK
 */
@Autonomous(name = "9pt Blue Close", group = "Competition")
public class Auto_9_BlueClose extends OpModeCommand {

    Robot_camera r;
    private String currentStep = "INIT";

    @Override
    public void initialize() {
        // 1. Initialize Robot Hardware (with camera)
        r = new Robot_camera(hardwareMap, Alliance.BLUE);

        // 2. Load Paths
        Paths_9_BlueClose p = new Paths_9_BlueClose(r.f);

        // 3. Set Starting Pose
        r.f.setStartingPose(p.startPose);

        // 4. Schedule the Full Autonomous Sequence
        schedule(
                new SequentialCommandGroup(

                        // ====== SHOOT PRELOAD ======
                        new InstantAction(() -> currentStep = "Preload: Prep"),
                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.setAngle(0.7)),
                                new InstantAction(() -> r.shooter.unblock()),
                                new FollowPath(r, p.ShootPreload)
                        ),
                        new InstantAction(() -> currentStep = "Preload: Shooting"),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 4, false),  // CLOSE SHOT
                        new InstantAction(() -> r.shooter.block()),

                        // ====== INTAKE CYCLE 1 ======
                        new InstantAction(() -> currentStep = "Art1: Driving to position"),
                        new FollowPath(r, p.GoTo1),
                        new InstantAction(() -> currentStep = "Art1: Intaking"),
                        new ParallelCommandGroup(
                                new IntakeCommand(r.intake, false, 2.5),
                                new FollowPath(r, p.Intake1)
                        ),

                        // ====== PUSH 3 GATE ARTIFACTS ======
                        new InstantAction(() -> currentStep = "Gates: Pushing artifacts"),
                        new FollowPath(r, p.OpenGate),
                        new FollowPath(r, p.OpenGate2),
                        new FollowPath(r, p.OpenGate3),

                        // ====== SHOOT BALL 1 ======
                        new InstantAction(() -> currentStep = "Art1: Returning to goal"),
                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.unblock()),
                                new FollowPath(r, p.Shoot1)
                        ),
                        new InstantAction(() -> currentStep = "Art1: Shooting"),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 4, false),  // CLOSE SHOT
                        new InstantAction(() -> r.shooter.block()),

                        // ====== INTAKE CYCLE 2 ======
                        new InstantAction(() -> currentStep = "Art2: Driving to position"),
                        new FollowPath(r, p.GoTo2),
                        new InstantAction(() -> currentStep = "Art2: Intaking"),
                        new ParallelCommandGroup(
                                new IntakeCommand(r.intake, false, 2),
                                new FollowPath(r, p.Intake2)
                        ),

                        // ====== SHOOT BALL 2 ======
                        new InstantAction(() -> currentStep = "Art2: Returning to goal"),
                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.unblock()),
                                new FollowPath(r, p.Shoot2)
                        ),
                        new InstantAction(() -> currentStep = "Art2: Shooting"),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 4, false),  // CLOSE SHOT
                        new InstantAction(() -> r.shooter.block()),

                        // ====== PARK ======
                        new InstantAction(() -> currentStep = "Park: Moving out of triangle"),
                        new FollowPath(r, p.OutOfTriangle),

                        // COMPLETE
                        new InstantAction(() -> {
                            currentStep = "COMPLETE";
                            r.shooter.block();
                            r.shooter.stop();
                            r.intake.stop();
                        })
                )
        );
    }

    @Override
    public void loop() {
        super.loop();
        r.periodic();

        telemetry.addLine("=== 9PT BLUE CLOSE AUTO ===");
        telemetry.addData("Runtime", "%.1f sec", getRuntime());
        telemetry.addData("Current Step", currentStep);
        telemetry.addLine("");

        telemetry.addLine("=== LIMELIGHT (TAG 20) ===");
        if (r.limelight.hasTarget()) {
            double distance = r.limelight.getDistanceToTarget();
            telemetry.addData("Target", "LOCKED");
            telemetry.addData("Tag ID", r.limelight.getDetectedTagId());
            telemetry.addData("Distance", "%.1f cm", distance);
            telemetry.addData("TX", "%.2f deg", r.limelight.getTx());
            telemetry.addData("TY", "%.2f deg", r.limelight.getTy());

            if (distance > 0) {
                ShooterCalculator_camera.ShooterConfig config =
                        ShooterCalculator_camera.getConfig(distance);
                telemetry.addData("Calc Angle", "%.3f", config.angle);
                telemetry.addData("Calc Vel", "%.0f", config.velocity);
            }
        } else {
            telemetry.addData("Target", "NO TAG 20");
            telemetry.addData("Failsafe", "CLOSE (1140 / 0.84)");
        }

        telemetry.addLine("");
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Actual Vel", "%.0f RPM", r.shooter.getVelocity());
        telemetry.addData("Actual Angle", "%.3f", r.shooter.getAngle());

        telemetry.addLine("");
        telemetry.addLine("=== POSITION ===");
        telemetry.addData("X", "%.1f", r.f.getPose().getX());
        telemetry.addData("Y", "%.1f", r.f.getPose().getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(r.f.getPose().getHeading()));
        telemetry.addData("Path Busy", r.f.isBusy());

        telemetry.update();
    }

    public static class InstantAction extends CommandBase {
        private final Runnable action;
        public InstantAction(Runnable action) { this.action = action; }
        @Override public void initialize() { action.run(); }
        @Override public boolean isFinished() { return true; }
    }
}