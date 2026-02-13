package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Paths_9_RedClose_New.java
 * PURPOSE: 9-Point Autonomous paths for RED ALLIANCE, CLOSE side - NEW SEQUENCE.
 *
 * Mirrored version of Paths_9_BlueClose_New.
 *
 * KEY DIFFERENCE from Paths_9_RedClose:
 *   - Goes to Artifact 2 (lower, y=60) FIRST after preload, then pushes 1 gate artifact
 *   - Then goes back up to Artifact 1 (y=84) for the second intake/shoot cycle
 *   - Simpler gate sequence: AlignToGate + PushGate (2 paths instead of 3)
 *   - Shoot1 is a straight BezierLine (no curve)
 *
 * Sequence:
 *   1. ShootPreload  – Bezier curve from start to scoring position (94, 93) at 43°
 *   2. GoTo2         – Straight line down to artifact 2 approach (103, 60)
 *   3. Intake2       – Drive straight while intaking artifact 2 (126, 60)
 *   4. AlignToGate   – Line to align with gate artifact (107.055, 69.055), rotate 0°→180°
 *   5. PushGate      – Line to push gate artifact right (128.055, 69.046)
 *   6. Shoot2        – Bezier curve back to scoring position (94, 93) at 43°
 *   7. GoTo1         – Straight line to artifact 1 approach (103, 84)
 *   8. Intake1       – Drive straight while intaking artifact 1 (127.6, 84)
 *   9. Shoot1        – Straight line back to scoring position (94, 93) at 43°
 *  10. Park          – Straight line to park outside triangle (125.009, 92.578)
 */
public class Paths_9_RedClose_New {

    // Robot starting pose (RED CLOSE) - same start as Paths_9_RedClose
    public Pose startPose = new Pose(118.75229357798165, 128.5871559633028, Math.toRadians(217));

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
    public Paths_9_RedClose_New(Robot r) {
        buildPaths(r.f);
    }

    // Constructor for Robot_camera
    public Paths_9_RedClose_New(Robot_camera r) {
        buildPaths(r.f);
    }

    // Constructor for direct Follower
    public Paths_9_RedClose_New(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {

        // Path 1: ShootPreload – Bezier curve from start to scoring position
        // Heading: 0° → 43°  (note: .pp file shows startDeg=217 which wraps to 0° effectively)
        ShootPreload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(118.752, 128.587),
                                new Pose(99.890, 117.771),
                                new Pose(94.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(217), Math.toRadians(43))

                .build();

        GoTo2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.000, 93.000),

                                new Pose(103.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(103.000, 60.000),

                                new Pose(126.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        AllignToGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.000, 60.000),

                                new Pose(107.055, 69.055)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))

                .build();

        PushGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(107.055, 69.055),

                                new Pose(128.055, 69.046)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(128.055, 69.046),
                                new Pose(94.468, 58.298),
                                new Pose(94.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(43))

                .build();

        GoTo1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.000, 93.000),

                                new Pose(103.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(103.000, 84.000),

                                new Pose(127.600, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.600, 84.000),

                                new Pose(94.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.000, 93.000),

                                new Pose(125.009, 92.578)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                .build();
    }
}