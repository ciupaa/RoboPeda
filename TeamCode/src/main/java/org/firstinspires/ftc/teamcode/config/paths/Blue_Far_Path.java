package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Blue_Far_Path.java
 * PURPOSE: Blue Alliance Far Position - Paths
 *
 * Path Names:
 * - ShootPreload: Drive to goal to shoot preload artifact
 * - GoTo1: Drive toward artifact 1
 * - Intake1: Final approach to artifact 1
 * - Shoot1: Return to goal to shoot artifact 1
 */
public class Blue_Far_Path {

    public Paths paths;

    // Starting position from Blue_Far_Path.pp
    public Pose startPose = new Pose(32.648648648648646, 8.194594594594609, Math.toRadians(90));

    // Constructor for Robot (no camera)
    public Blue_Far_Path(Robot r) {
        paths = new Paths(r.f);
    }

    // Constructor for Robot_camera
    public Blue_Far_Path(Robot_camera r) {
        paths = new Paths(r.f);
    }

    public static class Paths {
        public PathChain ShootPreload;
        public PathChain GoTo1;
        public PathChain Intake1;
        public PathChain Shoot1;

        public Paths(Follower follower) {

            // ShootPreload - Curve to goal
            ShootPreload = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(32.649, 8.195),
                                    new Pose(44.897, 19.665),
                                    new Pose(61.449, 14.205)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(109))
                    .build();

            // GoTo1 - Curve toward artifact 1
            GoTo1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.449, 14.205),
                                    new Pose(58.311, 28.992),
                                    new Pose(44.000, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(180))
                    .build();

            // Intake1 - Straight line to artifact 1
            Intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44.000, 35.000),
                                    new Pose(15.135, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Shoot1 - Return to goal
            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.135, 35.000),
                                    new Pose(48.719, 36.862),
                                    new Pose(61.449, 14.205)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(109))
                    .build();
        }
    }
}