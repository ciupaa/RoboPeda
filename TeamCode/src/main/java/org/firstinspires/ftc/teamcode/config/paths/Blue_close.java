package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Robot;

/**
 * FILE: Blue_close.java
 * PURPOSE: Stores the specific coordinates and curves for Autonomous.
 * NOTE: The 'Paths' class inside is structured to match the output of the Path Generator website.
 */
public class Blue_close {

    // We create an instance of the nested class to access the paths
    public Paths paths;

    // Defines where the robot starts on the field (X, Y, Heading)
    // 180 Degrees usually means facing towards the audience/drivers from the blue side
    public Pose startPose = new Pose(37.12432432432433, 135.65405405405406, Math.toRadians(270));

    public Blue_close(Robot r) {
        // Initialize the nested Paths class using the robot's follower
        paths = new Paths(r.f);
    }

    // --- PASTE GENERATOR CODE BELOW THIS LINE ---

    public static class Paths {
        public PathChain Shootpreload;
        public PathChain GoTo1;
        public PathChain Intake1;
        public PathChain Shoot1;
        public PathChain GoTo2;
        public PathChain Intake2;
        public PathChain Shoot2;

        public Paths(Follower follower) {
            Shootpreload = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(37.124, 135.654),
                                    new Pose(37.622, 120.941),
                                    new Pose(30.119, 114.227)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(143))

                    .build();

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

            Intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.395, 84.065),

                                    new Pose(16.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.000, 84.000),
                                    new Pose(43.297, 93.054),
                                    new Pose(30.119, 114.227)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                    .build();

            GoTo2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(30.119, 114.227),
                                    new Pose(56.297, 85.751),
                                    new Pose(42.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                    .build();

            Intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 60.000),

                                    new Pose(15.724, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

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
