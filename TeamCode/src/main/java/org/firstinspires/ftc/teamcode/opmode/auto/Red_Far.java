package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.Robot_camera;
import org.firstinspires.ftc.teamcode.config.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.config.paths.Red_Far_Path;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

@Autonomous(name = "Red Far", group = "Competition")
public class Red_Far extends OpModeCommand {

    Robot_camera r;

    @Override
    public void initialize() {
        r = new Robot_camera(hardwareMap, Alliance.RED);
        Red_Far_Path p = new Red_Far_Path(r);
        r.f.setStartingPose(p.startPose);

        schedule(
                new SequentialCommandGroup(
                        // PRELOAD
                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.setAngle(0.7)),
                                new InstantAction(() -> r.shooter.unblock()),
                                new FollowPath(r, p.paths.ShootPreload)
                        ),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 4),
                        new InstantAction(() -> r.shooter.block()),

                        // ARTIFACT 1
                        new FollowPath(r, p.paths.GoTo1),
                        new ParallelCommandGroup(
                                new IntakeCommand(r.intake, false, 2),
                                new FollowPath(r, p.paths.Intake1)
                        ),
                        new ParallelCommandGroup(
                                new InstantAction(() -> r.shooter.unblock()),
                                new FollowPath(r, p.paths.Shoot1)
                        ),
                        new AutoShootCommand(r.shooter, r.intake, r.limelight, 4),
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
        super.loop();
        r.periodic();

        // === COMPREHENSIVE TELEMETRY ===
        telemetry.addLine("=== RED FAR AUTO ===");
        telemetry.addData("Runtime", "%.1f sec", getRuntime());
        telemetry.addLine("");

        // Limelight Status
        telemetry.addLine("=== LIMELIGHT (TAG 24) ===");
        if (r.limelight.hasTarget()) {
            double distance = r.limelight.getDistanceToTarget();
            telemetry.addData("Target", "LOCKED");
            telemetry.addData("Distance", "%.1f cm (%.2f m)", distance, distance/100.0);
            telemetry.addData("TX", "%.2f deg", r.limelight.getTx());
            telemetry.addData("TY", "%.2f deg", r.limelight.getTy());

            // Show calculated shooter settings
            org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera.ShooterConfig config =
                    org.firstinspires.ftc.teamcode.config.util.ShooterCalculator_camera.getConfig(distance);
            telemetry.addData("Calc Angle", "%.3f", config.angle);
            telemetry.addData("Calc Vel", "%.0f", config.velocity);
        } else {
            telemetry.addData("Target", "NO TAG 24");
        }

        // Shooter Status
        telemetry.addLine("");
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Actual Vel", "%.0f RPM", r.shooter.getVelocity());
        telemetry.addData("Actual Angle", "%.3f", r.shooter.getAngle());

        // Position
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