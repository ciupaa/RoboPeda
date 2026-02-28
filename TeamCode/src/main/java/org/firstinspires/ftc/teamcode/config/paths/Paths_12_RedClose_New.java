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
 * Coordinates sourced from 12_Auto_Red.pp
 * Uses (84.4, 102.6) shooting position like Red 9pt NEW2
 *
 * Sequence:
 *   1.  ShootPreload  – Curve from start to scoring position (84.4, 102.6) at 35°
 *   2.  GoTo2         – Curve down to artifact 2 approach (95.954, 59.5)
 *   3.  Intake2       – Drive straight while intaking artifact 2 (132, 59.5)
 *   4.  AllignToGate  – Curve to gate alignment position, rotate 0°→180°
 *   5.  PushGate      – Line to push gate artifact right (128.055, 69.046)
 *   6.  Shoot2        – Curve back to scoring position (84.4, 102.6) at 35°
 *   7.  GoTo1         – Curve to artifact 1 approach (95.807, 84)
 *   8.  Intake1       – Drive straight while intaking artifact 1 (127.6, 84)
 *   9.  Shoot1        – Straight line back to scoring position (84.4, 102.6) at 35°
 *  10.  GoTo3         – Straight line to artifact 3 approach (95.8, 35.4)
 *  11.  Intake3       – Drive straight while intaking artifact 3 (132, 35.376)
 *  12.  Shoot3        – Curve back to scoring position (84.4, 102.6) at 35°
 *  13.  Park          – Straight line to park (120.073, 92.183)
 */
public class Paths_12_RedClose_New {

    // Robot starting pose (RED CLOSE) - from 12_Auto_Red.pp
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
        // Heading: 217° → 35°
        ShootPreload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(118.75229357798165, 128.5871559633028),
                                new Pose(99.89, 117.771),
                                new Pose(84.400, 102.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(217), Math.toRadians(35))
                .build();

        // Path 2: GoTo2 – Curve toward artifact 2 approach
        // Heading: 35° → 0°
        GoTo2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.400, 102.600),
                                new Pose(90.628, 76.977),
                                new Pose(95.954, 59.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
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
        // Heading: 180° → 35°
        Shoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(128.055, 69.046),
                                new Pose(94.468, 58.298),
                                new Pose(84.400, 102.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(35))
                .build();

        // Path 7: GoTo1 – Curve to artifact 1 approach
        // Heading: 35° → 0°
        GoTo1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.400, 102.600),
                                new Pose(94.353, 85.83),
                                new Pose(95.807, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
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
        // Heading: 0° → 35°
        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.600, 84.000),
                                new Pose(84.400, 102.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                .build();

        // Path 10: GoTo3 – Straight line to artifact 3 approach
        // Heading: 35° → 0°
        GoTo3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.400, 102.600),
                                new Pose(95.800, 35.400)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();

        // Path 11: Intake3 – Straight line while intaking artifact 3
        // Heading: constant 0°
        Intake3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.800, 35.400),
                                new Pose(132.000, 35.37614678899082)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 12: Shoot3 – Curve back to scoring position
        // Heading: 0° → 35°
        Shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 35.37614678899082),
                                new Pose(99.47522935779817, 66.37339449541285),
                                new Pose(84.400, 102.600)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                .build();

        // Path 13: Park – Straight line to parking position
        // Heading: 35° → 0°
        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.400, 102.600),
                                new Pose(120.07339449541284, 92.18348623853211)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();
    }
}