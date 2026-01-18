package org.firstinspires.ftc.teamcode.config.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.Robot;


/*
A command that wraps the Pedro Pathing logic. It tells the robot to follow a specific line/curve and finishes when the robot stops.
 */
public class FollowPath extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = true;

    public FollowPath(Robot r, PathChain pathChain) {
        this.follower = r.f;
        this.path = pathChain;
    }

    @Override
    public void initialize() {
        // Start the path following process
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        // Finished when Pedro says "Not Busy"
        return !follower.isBusy();
    }
}