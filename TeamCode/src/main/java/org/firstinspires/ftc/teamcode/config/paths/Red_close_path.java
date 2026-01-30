package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Red_close_path.java
 * PURPOSE: Red Alliance Close Position - MIRRORED FROM BLUE
 *
 * Path Names:
 * - Shootpreload: Drive to goal to shoot preload artifact
 * - GoTo1: Drive toward artifact 1
 * - Intake1: Final approach to artifact 1
 * - Shoot1: Return to goal to shoot artifact 1
 * - GoTo2: Drive toward artifact 2
 * - Intake2: Final approach to artifact 2
 * - Shoot2: Return to goal to shoot artifact 2
 */
public class Red_close_path {

    public Paths paths;

    // Starting position (mirrored from Blue)
    // -90 degrees = 270 degrees (facing down toward bottom of field)
    public Pose startPose = new Pose(106.87567567567567, 135.65405405405406, Math.toRadians(270));

    public Red_close_path(Robot r) {
        paths = new Paths(r.f);
    }

    public Red_close_path(Robot_camera r) {
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

            // Shootpreload - Curve to goal
            // -90° → 37° (270° → 37°)
            Shootpreload = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(106.876, 135.654),
                                    new Pose(106.378, 120.941),
                                    new Pose(97.22694684385382, 107.38641528239202)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(37))
                    .build();

            // GoTo1 - Curve toward artifact 1
            // 37° → 0°
            GoTo1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(97.22694684385382, 107.38641528239202),
                                    new Pose(103.014, 111.305),
                                    new Pose(100.083, 107.181),
                                    new Pose(104.367, 104.382),
                                    new Pose(102.713, 101.696),
                                    new Pose(101.361, 98.824),
                                    new Pose(100.412, 95.915),
                                    new Pose(99.885, 93.124),
                                    new Pose(99.722, 90.142),
                                    new Pose(99.962, 87.122),
                                    new Pose(100.605, 84.065)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                    .build();

            // Intake1 - Straight line to artifact 1
            // 0° → 0°
            Intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.605, 84.065),
                                    new Pose(128.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Shoot1 - Return to goal
            // 0° → 37°
            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.000, 84.000),
                                    new Pose(100.703, 93.054),
                                    new Pose(97.22694684385382, 107.38641528239202)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                    .build();

            // GoTo2 - Curve toward artifact 2
            // 37° → 0°
            GoTo2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(97.22694684385382, 107.38641528239202),
                                    new Pose(84.005, 83.611),
                                    new Pose(102.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                    .build();

            // Intake2 - Straight line to artifact 2
            // 0° → 0°
            Intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 60.000),
                                    new Pose(128.276, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Shoot2 - Return to goal
            // 0° → 37°
            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.276, 60.000),
                                    new Pose(91.722, 69.265),
                                    new Pose(97.22694684385382, 107.38641528239202)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                    .build();
        }
    }
}