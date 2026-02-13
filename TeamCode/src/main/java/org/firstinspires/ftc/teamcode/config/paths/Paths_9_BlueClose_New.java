package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Paths_9_BlueClose_New.java
 * PURPOSE: 9-Point Autonomous paths for BLUE ALLIANCE, CLOSE side - NEW SEQUENCE.
 *
 * KEY DIFFERENCE from Paths_9_BlueClose:
 *   - Goes to Artifact 2 (lower, y=60) FIRST after preload, then pushes 1 gate artifact
 *   - Then goes back up to Artifact 1 (y=84) for the second intake/shoot cycle
 *   - Simpler gate sequence: AlignToGate + PushGate (2 paths instead of 3)
 *   - Shoot1 is a straight BezierLine (no curve)
 *
 * Sequence:
 *   1. ShootPreload  – Bezier curve from start to scoring position (50, 93) at 137°
 *   2. GoTo2         – Straight line down to artifact 2 approach (41, 60)
 *   3. Intake2       – Drive straight while intaking artifact 2 (18, 60)
 *   4. AlignToGate   – Line to align with gate artifact (36.945, 69.055), rotate 180°→0°
 *   5. PushGate      – Line to push gate artifact left (15.945, 69.046)
 *   6. Shoot2        – Bezier curve back to scoring position (50, 93) at 137°
 *   7. GoTo1         – Straight line to artifact 1 approach (41, 84)
 *   8. Intake1       – Drive straight while intaking artifact 1 (16.4, 84)
 *   9. Shoot1        – Straight line back to scoring position (50, 93) at 137°
 *  10. Park          – Straight line to park outside triangle (18.991, 92.578)
 */
public class Paths_9_BlueClose_New {

    // Robot starting pose (BLUE CLOSE) - same start as Paths_9_BlueClose
    public Pose startPose = new Pose(25.24770642201835, 128.5871559633028, Math.toRadians(-37));

    // All path chains
    public PathChain ShootPreload;
    public PathChain GoTo2;
    public PathChain Intake2;
    public PathChain AllignToGate;
    public PathChain PushGate;
    public PathChain Shoot2;
    public PathChain GoTo1;
    public PathChain Intake1;
    public PathChain Shoot1;
    public PathChain Park;

    // Constructor for Robot (no camera)
    public Paths_9_BlueClose_New(Robot r) {
        buildPaths(r.f);
    }

    // Constructor for Robot_camera
    public Paths_9_BlueClose_New(Robot_camera r) {
        buildPaths(r.f);
    }

    // Constructor for direct Follower
    public Paths_9_BlueClose_New(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {

        // Path 1: ShootPreload – Bezier curve from start to scoring position
        // Heading: -37° → 137°
        ShootPreload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(25.248, 128.587),
                                new Pose(44.110, 117.771),
                                new Pose(50.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(137))

                .build();

        GoTo2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.000, 93.000),
                                new Pose(53.372, 76.977),
                                new Pose(48.046, 59.853)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.046, 59.853),

                                new Pose(15.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        AllignToGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.000, 60.000),

                                new Pose(36.945, 69.055)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))

                .build();

        PushGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(36.945, 69.055),

                                new Pose(15.945, 69.046)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.945, 69.046),
                                new Pose(49.532, 58.298),
                                new Pose(50.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(137))

                .build();

        GoTo1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.000, 93.000),
                                new Pose(49.647, 85.830),
                                new Pose(48.193, 84.147)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.193, 84.147),

                                new Pose(16.400, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.400, 84.000),

                                new Pose(50.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 93.000),

                                new Pose(18.991, 92.578)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();
    }
}