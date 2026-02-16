package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Paths_9_RedClose_New2.java
 * PURPOSE: 9-Point Autonomous paths for RED ALLIANCE, CLOSE side - NEW2 SEQUENCE.
 *
 * Mirrored version of Paths_9_BlueClose_New2.
 * Updated coordinates from 9_AutoRed_Close_(1).pp:
 *   - Intake2 endpoint: x=132, y=59.5 (was x=130, y=60)
 *   - GoTo2 endpoint: y=59.5
 *   - AllignToGate: different control point (126.513, 39.748)
 *   - Park endpoint: x=120.605 (was x=125.009)
 *
 * Sequence:
 *   1. ShootPreload  – Bezier curve from start to scoring position (94, 93) at 43°
 *   2. GoTo2         – Curve down to artifact 2 approach (95.954, 59.5)
 *   3. Intake2       – Drive straight while intaking artifact 2 (132, 59.5)
 *   4. AllignToGate  – Curve to align with gate artifact (107.055, 69.055), rotate 0°→180°
 *   5. PushGate      – Line to push gate artifact right (128.055, 69.046)
 *   6. Shoot2        – Bezier curve back to scoring position (94, 93) at 43°
 *   7. GoTo1         – Curve to artifact 1 approach (95.807, 84)
 *   8. Intake1       – Drive straight while intaking artifact 1 (127.6, 84)
 *   9. Shoot1        – Straight line back to scoring position (94, 93) at 43°
 *  10. Park          – Straight line to park (120.605, 92.578)
 */
public class Paths_9_RedClose_New2 {

    // Robot starting pose (RED CLOSE)
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
    public Paths_9_RedClose_New2(Robot r) {
        buildPaths(r.f);
    }

    // Constructor for Robot_camera
    public Paths_9_RedClose_New2(Robot_camera r) {
        buildPaths(r.f);
    }

    // Constructor for direct Follower
    public Paths_9_RedClose_New2(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {

        // Path 1: ShootPreload – Bezier curve from start to scoring position
        // Heading: 217° → 43°
        ShootPreload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(118.752, 128.587),
                                new Pose(99.890, 117.771),
                                new Pose(84.400, 102.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(217), Math.toRadians(37))

                .build();

        GoTo2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.400, 102.600),
                                new Pose(90.628, 76.977),
                                new Pose(95.954, 59.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.954, 59.500),

                                new Pose(132.000, 59.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        AllignToGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 59.500),
                                new Pose(126.513, 39.748),
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
                                new Pose(84.400, 102.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(37))

                .build();

        GoTo1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.400, 102.600),
                                new Pose(94.353, 85.830),
                                new Pose(95.807, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.807, 84.000),

                                new Pose(127.600, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.600, 84.000),

                                new Pose(84.400, 102.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.400, 102.600),

                                new Pose(120.605, 92.578)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                .build();
    }
}