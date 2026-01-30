package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Red_Far_Path.java
 * PURPOSE: Red Alliance Far Position - Paths
 *
 * UPDATED ANGLES: 71° → 73°
 *
 * Path Names:
 * - ShootPreload: Drive to goal to shoot preload artifact
 * - GoTo1: Drive toward artifact 1
 * - Intake1: Final approach to artifact 1
 * - Shoot1: Return to goal to shoot artifact 1
 */
public class Red_Far_Path {

    public Paths paths;

    // Starting position from Red_Far_Path.pp
    public Pose startPose = new Pose(111.35135135135135, 8.194594594594609, Math.toRadians(90));

    // Constructor for Robot (no camera)
    public Red_Far_Path(Robot r) {
        paths = new Paths(r.f);
    }

    // Constructor for Robot_camera
    public Red_Far_Path(Robot_camera r) {
        paths = new Paths(r.f);
    }

    public static class Paths {
        public PathChain ShootPreload;
        public PathChain GoTo1;
        public PathChain Intake1;
        public PathChain Shoot1;

        public Paths(Follower follower) {

            // ShootPreload - Curve to goal
            // UPDATED: 71° → 73°
            ShootPreload = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(111.351, 8.195),
                                    new Pose(99.103, 19.665),
                                    new Pose(83.66727906976745, 17.39436877076411)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                    .build();

            // GoTo1 - Curve toward artifact 1
            // UPDATED: 71° → 73°
            GoTo1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(83.66727906976745, 17.39436877076411),
                                    new Pose(85.689, 28.992),
                                    new Pose(100.000, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .build();

            // Intake1 - Straight line to artifact 1
            Intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 35.000),
                                    new Pose(128.865, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Shoot1 - Return to goal
            // UPDATED: 71° → 73°
            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.865, 35.000),
                                    new Pose(95.281, 36.862),
                                    new Pose(83.66727906976745, 17.39436877076411)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();
        }
    }
}