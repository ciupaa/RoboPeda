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
 * PURPOSE: Red Alliance Close Position - CORRECTED FORWARD DRIVING
 *
 * FIX: Added +180° to ALL headings so robot drives FORWARD instead of BACKWARD
 */
public class Red_close_path {

    public Paths paths;

    // FIXED: Changed from 270° to 90° (270 + 180 = 450 = 90)
    // This makes the robot face UP (toward the top of the field) so it drives FORWARD
    public Pose startPose = new Pose(106.87567567567567, 135.65405405405406, Math.toRadians(90));

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

            // FIXED: All headings + 180°
            // OLD: 270→217  NEW: 90→37 (270+180=90, 217+180=397=37)
            Shootpreload = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(106.876, 135.654),
                                    new Pose(106.378, 120.941),
                                    new Pose(102.011, 112.011)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(37))
                    .build();

            // OLD: 217→180  NEW: 37→0 (217+180=37, 180+180=0)
            GoTo1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(102.011, 112.011),
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

            // OLD: 180→180  NEW: 0→0
            Intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.605, 84.065),
                                    new Pose(128.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // OLD: 180→217  NEW: 0→37
            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.000, 84.000),
                                    new Pose(100.703, 93.054),
                                    new Pose(102.011, 112.011)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                    .build();

            // OLD: 217→180  NEW: 37→0
            GoTo2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(102.011, 112.011),
                                    new Pose(84.005, 83.611),
                                    new Pose(102.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                    .build();

            // OLD: 180→180  NEW: 0→0
            Intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 60.000),
                                    new Pose(128.276, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // OLD: 180→217  NEW: 0→37
            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.276, 60.000),
                                    new Pose(91.722, 69.265),
                                    new Pose(102.011, 112.011)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                    .build();
        }
    }
}