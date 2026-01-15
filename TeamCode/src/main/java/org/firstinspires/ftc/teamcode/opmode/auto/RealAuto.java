package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.commands.FollowPath;
import org.firstinspires.ftc.teamcode.config.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.config.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.OpModeCommand;

@Autonomous(name = "Real Auto", group = "Competition")
public class RealAuto extends OpModeCommand {
    @Override
    public void initialize() {
        Robot r = new Robot(hardwareMap, Alliance.BLUE);
        AutoPaths p = new AutoPaths(r);
        r.f.setStartingPose(p.startPose);

        schedule(
                new SequentialCommandGroup(
                        new FollowPath(r, p.line1),
                        new ShootCommand(r.shooter, 1450, 0.42), // Speed and Angle
                        new WaitCommand(1000),
                        new FollowPath(r, p.path2)
                )
        );
    }
}