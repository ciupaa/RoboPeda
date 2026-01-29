package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Blue_close_path.java
 * PURPOSE: Blue Alliance Close Position - Paths
 *
 * Path Names (Updated Terminology):
 * - Shootpreload: Drive to goal to shoot preload artifact
 * - GoTo1: Drive toward artifact 1
 * - Intake1: Final approach to artifact 1
 * - Shoot1: Return to goal to shoot artifact 1
 * - GoTo2: Drive toward artifact 2
 * - Intake2: Final approach to artifact 2
 * - Shoot2: Return to goal to shoot artifact 2
 */
public class Blue_close_path {

    public Paths paths;

    // Starting position (X, Y, Heading in radians)
    // 270 degrees = facing down (toward bottom of field)
    public Pose startPose = new Pose(37.12432432432433, 135.65405405405406, Math.toRadians(270));

    // Constructor for Robot (no camera)
    public Blue_close_path(Robot r) {
        paths = new Paths(r.f);
    }

    // Constructor for Robot_camera
    public Blue_close_path(Robot_camera r) {
        paths = new Paths(r.f);
    }

    public static class Paths {
        public PathChain Shootpreload;
        public PathChain GoTo1;
        public PathChain Intake1;
        public PathChain Shoot1;
        public PathChain GoTo2;
        public PathChain Intake2;
        public PathChain Shoot2;

        public Paths(Follower follower) {
            // Path to shoot preload artifact at goal
            Shootpreload = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(37.124, 135.654),
                                    new Pose(37.622, 120.941),
                                    new Pose(30.119, 114.227)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(143))
                    .build();

            // Drive toward artifact 1
            GoTo1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(30.119, 114.227),
                                    new Pose(34.953, 109.943),
                                    new Pose(37.495, 107.181),
                                    new Pose(39.633, 104.382),
                                    new Pose(41.287, 101.696),
                                    new Pose(42.639, 98.824),
                                    new Pose(43.588, 95.915),
                                    new Pose(44.115, 93.124),
                                    new Pose(44.278, 90.142),
                                    new Pose(44.038, 87.122),
                                    new Pose(43.395, 84.065)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            // Final approach to artifact 1
            Intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.395, 84.065),
                                    new Pose(16.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Return to goal to shoot artifact 1
            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.000, 84.000),
                                    new Pose(43.297, 93.054),
                                    new Pose(30.119, 114.227)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();

            // Drive toward artifact 2
            GoTo2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(30.119, 114.227),
                                    new Pose(56.297, 85.751),
                                    new Pose(42.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            // Final approach to artifact 2
            Intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 60.000),
                                    new Pose(15.724, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Return to goal to shoot artifact 2
            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.724, 60.000),
                                    new Pose(52.278, 69.265),
                                    new Pose(30.119, 114.227)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();
        }
    }
}