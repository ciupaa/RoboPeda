package org.firstinspires.ftc.teamcode.config.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot; // Import YOUR Robot class

public class FollowPath extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = true;
    private double maxPower = 1;

    public FollowPath(Robot r, PathChain pathChain) {
        // This requires your Robot.java to have 'public Follower f;'
        this.follower = r.f;
        this.path = pathChain;
    }

    public FollowPath(Robot r, PathChain pathChain, boolean holdEnd) {
        this.follower = r.f;
        this.path = pathChain;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        follower.setMaxPower(this.maxPower);
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        // Returns true when the robot finishes the path
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        follower.setMaxPower(1);
    }
}