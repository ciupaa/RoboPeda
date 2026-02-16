package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Paths_9_BlueClose_New2.java
 * PURPOSE: 9-Point Autonomous paths for BLUE ALLIANCE, CLOSE side - NEW2 SEQUENCE.
 *
 * Updated version of Paths_9_BlueClose_New with revised coordinates from 9_AutoBlue_Close_(1).pp:
 *   - Intake2 endpoint: x=12, y=59.5 (was x=13, y=60)
 *   - GoTo2 endpoint: y=59.5
 *   - AllignToGate: different control point (17.487, 39.748)
 *   - Park endpoint: x=23.395 (was x=18.991)
 *
 * Sequence:
 *   1. ShootPreload  – Bezier curve from start to scoring position (50, 93) at 137°
 *   2. GoTo2         – Curve down to artifact 2 approach (48.046, 59.5)
 *   3. Intake2       – Drive straight while intaking artifact 2 (12, 59.5)
 *   4. AllignToGate  – Curve to align with gate artifact (36.945, 69.055), rotate 180°→0°
 *   5. PushGate      – Line to push gate artifact left (15.945, 69.046)
 *   6. Shoot2        – Bezier curve back to scoring position (50, 93) at 137°
 *   7. GoTo1         – Curve to artifact 1 approach (48.193, 84)
 *   8. Intake1       – Drive straight while intaking artifact 1 (16.4, 84)
 *   9. Shoot1        – Straight line back to scoring position (50, 93) at 137°
 *  10. Park          – Straight line to park (23.395, 92.578)
 */
public class Paths_9_BlueClose_New2 {

    // Robot starting pose (BLUE CLOSE)
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
    public Paths_9_BlueClose_New2(Robot r) {
        buildPaths(r.f);
    }

    // Constructor for Robot_camera
    public Paths_9_BlueClose_New2(Robot_camera r) {
        buildPaths(r.f);
    }

    // Constructor for direct Follower
    public Paths_9_BlueClose_New2(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {

        // Path 1: ShootPreload – Bezier curve from start to scoring position
        // Heading: -37° → 137°
        ShootPreload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(25.248, 128.587),
                                new Pose(44.110, 117.771),
                                new Pose(59.600, 102.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(143))

                .build();

        GoTo2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.600, 102.600),
                                new Pose(53.372, 76.977),
                                new Pose(48.046, 59.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.046, 59.500),

                                new Pose(12.000, 59.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        AllignToGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(12.000, 59.500),
                                new Pose(17.487, 39.748),
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
                                new Pose(59.600, 102.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(143))

                .build();

        GoTo1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.600, 102.600),
                                new Pose(49.647, 85.830),
                                new Pose(48.193, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.193, 84.000),

                                new Pose(16.400, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.400, 84.000),

                                new Pose(59.600, 102.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.600, 102.600),

                                new Pose(23.395, 92.578)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                .build();
    }
}