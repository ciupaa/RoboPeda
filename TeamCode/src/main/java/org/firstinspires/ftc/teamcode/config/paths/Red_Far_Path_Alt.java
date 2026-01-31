package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Red_Far_Path_Alt.java
 * PURPOSE: Red Alliance Far Position - Alternative Shooting Position
 *
 * SHOOTING POSITION: (83.72, 18.49, 60°)
 *
 * Path Names:
 * - ShootPreload: Drive to goal to shoot preload artifact
 * - GoTo1: Drive toward artifact 1
 * - Intake1: Final approach to artifact 1
 * - Shoot1: Return to goal to shoot artifact 1
 * - OutOfTriangle: Exit triangle after final shot
 */
public class Red_Far_Path_Alt {

    public Paths paths;

    // Starting position from Red_Far_Path.pp
    public Pose startPose = new Pose(101.9, 8.195, Math.toRadians(90));

    // Constructor for Robot (no camera)
    public Red_Far_Path_Alt(Robot r) {
        paths = new Paths(r.f);
    }

    // Constructor for Robot_camera
    public Red_Far_Path_Alt(Robot_camera r) {
        paths = new Paths(r.f);
    }

    public static class Paths {
        public PathChain ShootPreload;
        public PathChain GoTo1;
        public PathChain Intake1;
        public PathChain Shoot1;
        public PathChain OutOfTriangle;

        public Paths(Follower follower) {

            // ShootPreload - Curve to goal
            ShootPreload = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(101.9, 8.195),
                                    new Pose(99.103, 19.660),
                                    new Pose(83.71856756756756, 18.486081081081085)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                    .build();

            // GoTo1 - Curve toward artifact 1
            GoTo1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(83.71856756756756, 18.486081081081085),
                                    new Pose(85.689, 28.992),
                                    new Pose(100.000, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                    .build();

            // Intake1 - Straight line to artifact 1
            Intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 35.000),
                                    new Pose(128.860, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Shoot1 - Return to goal
            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.860, 35.000),
                                    new Pose(95.281, 36.862),
                                    new Pose(83.71856756756756, 18.486081081081085)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                    .build();

            // OutOfTriangle - Exit triangle after final shot
            OutOfTriangle = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.71856756756756, 18.486081081081085),
                                    new Pose(97.103, 36.195)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(60))
                    .build();
        }
    }
}