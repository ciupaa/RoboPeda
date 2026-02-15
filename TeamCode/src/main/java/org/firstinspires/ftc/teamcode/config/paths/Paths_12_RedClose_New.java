package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Paths_12_RedClose_New.java
 * PURPOSE: 12-Point Autonomous paths for RED ALLIANCE, CLOSE side - NEW SEQUENCE.
 *
 * Mirrored version of Paths_12_BlueClose_New with the Red alliance field coordinates.
 * Coordinates sourced from 12_AutoRed_Close.pp
 *
 * Sequence:
 *   1.  ShootPreload  – Curve from start to scoring position (94, 93) at 43°
 *   2.  GoTo2         – Curve down to artifact 2 approach (95.954, 59.5)
 *   3.  Intake2       – Drive straight while intaking artifact 2 (132, 59.5)
 *   4.  AllignToGate  – Curve to gate alignment position, rotate 0°→180°
 *   5.  PushGate      – Line to push gate artifact right (128.055, 69.046)
 *   6.  Shoot2        – Curve back to scoring position (94, 93) at 43°
 *   7.  GoTo1         – Curve to artifact 1 approach (95.807, 84)
 *   8.  Intake1       – Drive straight while intaking artifact 1 (127.6, 84)
 *   9.  Shoot1        – Straight line back to scoring position (94, 93) at 43°
 *  10.  GoTo3         – Straight line to artifact 3 approach (95.193, 35.193)
 *  11.  Intake3       – Drive straight while intaking artifact 3 (135.092, 35.560)
 *  12.  Shoot3        – Straight line back to scoring position (94, 93) at 43°
 *  13.  Park          – Straight line to park (117.798, 92.752)
 */
public class Paths_12_RedClose_New {

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
    public PathChain GoTo3;
    public PathChain Intake3;
    public PathChain Shoot3;
    public PathChain Park;

    // Constructor for Robot (no camera)
    public Paths_12_RedClose_New(Robot r) {
        buildPaths(r.f);
    }

    // Constructor for Robot_camera
    public Paths_12_RedClose_New(Robot_camera r) {
        buildPaths(r.f);
    }

    // Constructor for direct Follower
    public Paths_12_RedClose_New(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {

        // Path 1: ShootPreload – Bezier curve from start to scoring position
        // Heading: 217° → 43°
        ShootPreload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(118.752, 128.587),
                                new Pose(99.890, 117.771),
                                new Pose(94.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(217), Math.toRadians(43))
                .build();

        // Path 2: GoTo2 – Curve toward artifact 2 approach
        // Heading: 43° → 0°
        GoTo2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(94.000, 93.000),
                                new Pose(90.628, 76.977),
                                new Pose(95.954, 59.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        // Path 3: Intake2 – Straight line while intaking artifact 2
        // Heading: constant 0°
        Intake2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.954, 59.5),
                                new Pose(132.000, 59.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 4: AllignToGate – Curve to gate alignment position, rotate 0° → 180°
        AllignToGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 59.5),
                                new Pose(126.513, 39.748),
                                new Pose(107.055, 69.055)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        // Path 5: PushGate – Straight line pushing gate artifact right
        // Heading: constant 180°
        PushGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(107.055, 69.055),
                                new Pose(128.055, 69.046)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 6: Shoot2 – Curve back to scoring position
        // Heading: 180° → 43°
        Shoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(128.055, 69.046),
                                new Pose(94.468, 58.298),
                                new Pose(94.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(43))
                .build();

        // Path 7: GoTo1 – Curve to artifact 1 approach
        // Heading: 43° → 0°
        GoTo1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(94.000, 93.000),
                                new Pose(94.353, 85.830),
                                new Pose(95.807, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        // Path 8: Intake1 – Straight line while intaking artifact 1
        // Heading: constant 0°
        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.807, 84.000),
                                new Pose(127.600, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 9: Shoot1 – Straight line back to scoring position
        // Heading: 0° → 43°
        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.600, 84.000),
                                new Pose(94.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        // Path 10: GoTo3 – Straight line to artifact 3 approach
        // Heading: 43° → 0°
        GoTo3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.000, 93.000),
                                new Pose(95.193, 35.193)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        // Path 11: Intake3 – Straight line while intaking artifact 3
        // Heading: constant 0°
        Intake3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.193, 35.193),
                                new Pose(135.092, 35.560)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 12: Shoot3 – Straight line back to scoring position
        // Heading: 0° → 43°
        Shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135.092, 35.560),
                                new Pose(94.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        // Path 13: Park – Straight line to parking position
        // Heading: 43° → 0°
        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.000, 93.000),
                                new Pose(117.798, 92.752)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();
    }
}