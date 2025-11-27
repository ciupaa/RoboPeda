package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;

public class AutoPaths {
    private final Follower follower;

    // 1. Define Start Pose (Taken from the start of 'line1')
    public Pose startPose = new Pose(14.674, 108.978, Math.toRadians(90));

    // 2. Declare Path Chains
    public PathChain line1, path2;

    public AutoPaths(Robot r) {
        this.follower = r.f;
        buildPaths();
    }

    private void buildPaths() {
        // --- Path 1: Start -> Shooting Position ---
        line1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(14.674, 108.978),
                                new Pose(26.413, 113.478),
                                new Pose(24.261, 124.630)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(144))
                .build();

        // --- Path 2: Shooting Position -> Next Spot ---
        path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(24.261, 124.630),
                                new Pose(46.761, 124.043),
                                new Pose(57.717, 135.196)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(90))
                .build();
    }
}