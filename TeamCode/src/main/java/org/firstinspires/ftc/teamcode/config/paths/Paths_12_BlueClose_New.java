package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Robot_camera;

/**
 * FILE: Paths_12_BlueClose_New.java
 * PURPOSE: 12-Point Autonomous paths for BLUE ALLIANCE, CLOSE side - NEW SEQUENCE.
 *
 * Extended version of Paths_9_BlueClose_New2 with an additional intake/shoot cycle (Artifact 3).
 * Coordinates sourced from 12_AutoBlue.pp
 *
 * Sequence:
 *   1.  ShootPreload  – Curve from start to scoring position (50, 93) at 137°
 *   2.  GoTo2         – Curve down to artifact 2 approach (48.046, 59.5)
 *   3.  Intake2       – Drive straight while intaking artifact 2 (12, 59.5)
 *   4.  AllignToGate  – Curve to gate alignment position, rotate 180°→0°
 *   5.  PushGate      – Line to push gate artifact left (15.945, 69.046)
 *   6.  Shoot2        – Curve back to scoring position (50, 93) at 137°
 *   7.  GoTo1         – Curve to artifact 1 approach (48.193, 84)
 *   8.  Intake1       – Drive straight while intaking artifact 1 (16.5, 84)
 *   9.  Shoot1        – Straight line back to scoring position (50, 93) at 137°
 *  10.  GoTo3         – Straight line to artifact 3 approach (48.807, 35.193)
 *  11.  Intake3       – Drive straight while intaking artifact 3 (8.908, 35.560)
 *  12.  Shoot3        – Straight line back to scoring position (50, 93) at 137°
 *  13.  Park          – Straight line to park (26.202, 92.752)
 */
public class Paths_12_BlueClose_New {

    // Robot starting pose (BLUE CLOSE) - from 12_AutoBlue.pp
    public Pose startPose = new Pose(25.247706422018354, 128.5871559633028, Math.toRadians(-37));

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
    public Paths_12_BlueClose_New(Robot r) {
        buildPaths(r.f);
    }

    // Constructor for Robot_camera
    public Paths_12_BlueClose_New(Robot_camera r) {
        buildPaths(r.f);
    }

    // Constructor for direct Follower
    public Paths_12_BlueClose_New(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {

        // Path 1: ShootPreload – Bezier curve from start to scoring position
        // Heading: -37° → 137°
        ShootPreload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(25.247706422018354, 128.5871559633028),
                                new Pose(44.11, 117.771),
                                new Pose(50.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(137))
                .build();

        // Path 2: GoTo2 – Curve toward artifact 2 approach
        // Heading: 137° → 180°
        GoTo2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.000, 93.000),
                                new Pose(53.372, 76.977),
                                new Pose(48.046, 59.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        // Path 3: Intake2 – Straight line while intaking artifact 2
        // Heading: constant 180°
        Intake2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.046, 59.5),
                                new Pose(12.000, 59.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 4: AllignToGate – Curve to gate alignment position, rotate 180° → 0°
        AllignToGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(12.000, 59.5),
                                new Pose(17.486513761467904, 39.74754128440367),
                                new Pose(36.945, 69.055)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        // Path 5: PushGate – Straight line pushing gate artifact left
        // Heading: constant 0°
        PushGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(36.945, 69.055),
                                new Pose(15.944999999999993, 69.046)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 6: Shoot2 – Curve back to scoring position
        // Heading: 0° → 137°
        Shoot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.944999999999993, 69.046),
                                new Pose(49.532, 58.298),
                                new Pose(50.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(137))
                .build();

        // Path 7: GoTo1 – Curve to artifact 1 approach
        // Heading: 137° → 180°
        GoTo1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.000, 93.000),
                                new Pose(49.647, 85.83),
                                new Pose(48.193, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        // Path 8: Intake1 – Straight line while intaking artifact 1
        // Heading: constant 180°
        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.193, 84.000),
                                new Pose(16.500, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 9: Shoot1 – Straight line back to scoring position
        // Heading: 180° → 137°
        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.500, 84.000),
                                new Pose(50.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        // Path 10: GoTo3 – Straight line to artifact 3 approach
        // Heading: 137° → 180°
        GoTo3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 93.000),
                                new Pose(48.80733944954129, 35.192660550458726)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        // Path 11: Intake3 – Straight line while intaking artifact 3
        // Heading: constant 180°
        Intake3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.80733944954129, 35.192660550458726),
                                new Pose(8.908256880733944, 35.55963302752296)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 12: Shoot3 – Straight line back to scoring position
        // Heading: 180° → 137°
        Shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.908256880733944, 35.55963302752296),
                                new Pose(50.000, 93.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        // Path 13: Park – Straight line to parking position
        // Heading: 137° → 180°
        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 93.000),
                                new Pose(26.20183486238531, 92.75229357798163)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();
    }
}