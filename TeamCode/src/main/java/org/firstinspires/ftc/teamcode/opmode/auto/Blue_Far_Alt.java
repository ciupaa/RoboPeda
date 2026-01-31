package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.config.paths.Blue_Far_Path_Alt;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

@Autonomous(name = "Blue Far Alt", group = "Competition")
public class Blue_Far_Alt extends OpModeCommand {

    Robot_camera r;
    private String currentStep = "INIT";

    @Override
    public void initialize() {
        r = new Robot_camera(hardwareMap, Alliance.BLUE);
        Blue_Far_Path_Alt p = new Blue_Far_Path_Alt(r);
        r.f.setStartingPose(p.startPose);

        schedule(
                new SequentialCommandGroup(
                        // PRELOAD
                        new InstantAction(() -> currentStep = "Preload: Prep"),
                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.setAngle(0.7)),
                                new InstantAction(() -> r.shooter.unblock()),
                                new FollowPath(r, p.paths.ShootPreload)
                        ),
                        new InstantAction(() -> currentStep = "Preload: Shooting"),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 4, true),  // FAR SHOT
                        new InstantAction(() -> r.shooter.block()),

                        // ARTIFACT 1
                        new InstantAction(() -> currentStep = "Art1: Driving to position"),
                        new FollowPath(r, p.paths.GoTo1),
                        new InstantAction(() -> currentStep = "Art1: Intaking"),
                        new ParallelCommandGroup(
                                new IntakeCommand(r.intake, false, 2),
                                new FollowPath(r, p.paths.Intake1)
                        ),
                        new InstantAction(() -> currentStep = "Art1: Returning to goal"),
                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.unblock()),
                                new FollowPath(r, p.paths.Shoot1)
                        ),
                        new InstantAction(() -> currentStep = "Art1: Shooting"),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 4, true),  // FAR SHOT
                        new InstantAction(() -> r.shooter.block()),

                        // EXIT TRIANGLE
                        new InstantAction(() -> currentStep = "Exit: Moving out of triangle"),
                        new FollowPath(r, p.paths.OutofTriangle),

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

        telemetry.addLine("=== BLUE FAR ALT AUTO ===");
        telemetry.addData("Shoot Pos", "60.28, 18.49, 120°");
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
                org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera.ShooterConfig config =
                        org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera.getConfig(distance);
                telemetry.addData("Calc Angle", "%.3f", config.angle);
                telemetry.addData("Calc Vel", "%.0f", config.velocity);
            }
        } else {
            telemetry.addData("Target", "NO TAG 20");
            telemetry.addData("Failsafe", "FAR (1500 / 0.65)");
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