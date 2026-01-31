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
 * UPDATED: New starting position (33.4, 135.654) and added exit path
 *
 * Path Names:
 * - Shootpreload: Drive to goal to shoot preload artifact
 * - GoTo1: Drive toward artifact 1
 * - Intake1: Final approach to artifact 1
 * - Shoot1: Return to goal to shoot artifact 1
 * - GoTo2: Drive toward artifact 2
 * - Intake2: Final approach to artifact 2
 * - Shoot2: Return to goal to shoot artifact 2
 * - ExitPath: Move away from goal after final shot
 */
public class Blue_close_path {

    public Paths paths;

    // UPDATED: Starting position changed from (37.124, 135.654) to (33.4, 135.654)
    public Pose startPose = new Pose(33.4, 135.65405405405406, Math.toRadians(270));

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
        public PathChain ExitPath;  // NEW: Exit path after final shot

        public Paths(Follower follower) {
            // Shootpreload - Curve to goal
            Shootpreload = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(33.4, 135.654),
                                    new Pose(46.963, 123.471),
                                    new Pose(46.773053156146176, 107.38641528239202)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(143))
                    .build();

            // GoTo1 - Curve toward artifact 1
            GoTo1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(46.773053156146176, 107.38641528239202),
                                    new Pose(40.986, 111.305),
                                    new Pose(43.917, 107.181),
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

            // Intake1 - Straight line to artifact 1
            Intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.395, 84.065),
                                    new Pose(16.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Shoot1 - Return to goal
            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.000, 84.000),
                                    new Pose(43.297, 93.054),
                                    new Pose(46.773053156146176, 107.38641528239202)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();

            // GoTo2 - Curve toward artifact 2
            GoTo2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(46.773053156146176, 107.38641528239202),
                                    new Pose(59.995, 83.611),
                                    new Pose(42.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            // Intake2 - Straight line to artifact 2
            Intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 60.000),
                                    new Pose(15.724, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Shoot2 - Return to goal
            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.724, 60.000),
                                    new Pose(52.278, 69.265),
                                    new Pose(46.773053156146176, 107.38641528239202)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();

            // ExitPath - NEW: Exit away from goal after final shot
            ExitPath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(46.773053156146176, 107.38641528239202),
                                    new Pose(33.062, 95.577),
                                    new Pose(16.573, 95.659)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }
}