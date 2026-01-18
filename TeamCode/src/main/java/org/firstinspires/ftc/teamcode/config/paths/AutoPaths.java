package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathBuilder;
import org.firstinspires.ftc.teamcode.config.Robot;

/**
 * FILE: AutoPaths.java
 * PURPOSE: Stores the specific coordinates and curves for Autonomous.
 * NOTE: The 'Paths' class inside is structured to match the output of the Path Generator website.
 */
public class AutoPaths {

    // We create an instance of the nested class to access the paths
    public Paths paths;

    // Defines where the robot starts on the field (X, Y, Heading)
    // 180 Degrees usually means facing towards the audience/drivers from the blue side
    public Pose startPose = new Pose(125.341, 80.016, Math.toRadians(180));

    public AutoPaths(Robot r) {
        // Initialize the nested Paths class using the robot's follower
        paths = new Paths(r.f);
    }

    // --- PASTE GENERATOR CODE BELOW THIS LINE ---
    public static class Paths {
        public PathChain Path2;
        public PathChain Path1;
        public PathChain Path3;

        public Paths(Follower follower) {

            // Path 2: A straight line
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(125.341, 80.016)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 1: A curve (Bezier Curve)
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.341, 80.016),
                                    new Pose(22.416, 43.292),
                                    new Pose(56.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 3: A line with spinning (Linear Heading Interpolation)
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 36.000),
                                    new Pose(36.978, 25.114)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(31), Math.toRadians(23))
                    .build();
        }
    }
}