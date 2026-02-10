package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;

/**
 * FILE: Paths_12_BlueClose.java
 * PURPOSE: 12-Point Autonomous paths for BLUE ALLIANCE, CLOSE side.
 *
 * Extended version of 9-point with a third intake/shoot cycle.
 *
 * Sequence:
 *   1. ShootPreload  – Drive to scoring position, shoot preloaded ball
 *   2. GoTo1         – Approach first intake zone
 *   3. Intake1       – Drive forward while intaking ball 1
 *   4. OpenGate      – Push artifact into scoring zone
 *   5. OpenGate2     – Push second artifact
 *   6. OpenGate3     – Push third artifact
 *   7. Shoot1        – Return to scoring position, shoot ball 1
 *   8. GoTo2         – Approach second intake zone
 *   9. Intake2       – Drive forward while intaking ball 2
 *  10. Shoot2        – Return to scoring position, shoot ball 2
 *  11. GoTo3         – Approach third intake zone
 *  12. Intake3       – Drive forward while intaking ball 3
 *  13. Shoot3        – Return to scoring position, shoot ball 3
 *  14. OutOfTriangle – Park outside scoring triangle
 */
public class Paths_12_BlueClose {

    // Robot starting pose (BLUE CLOSE)
    public Pose startPose = new Pose(25.248, 128.587, Math.toRadians(180));

    // All path chains
    public PathChain ShootPreload;
    public PathChain GoTo1;
    public PathChain Intake1;
    public PathChain OpenGate;
    public PathChain OpenGate2;
    public PathChain OpenGate3;
    public PathChain Shoot1;
    public PathChain GoTo2;
    public PathChain Intake2;
    public PathChain Shoot2;
    public PathChain GoTo3;
    public PathChain Intake3;
    public PathChain Shoot3;
    public PathChain OutOfTriangle;

    public Paths_12_BlueClose(Follower follower) {

        // Path 1: ShootPreload – Curve from start to scoring position
        // Heading: 180° → 137°
        ShootPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(25.248, 128.587),
                        new Pose(44.110, 117.771),
                        new Pose(50.000, 93.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        // Path 2: GoTo1 – Curve to first intake approach
        // Heading: 137° → 180°
        GoTo1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.000, 93.000),
                        new Pose(49.606, 85.606),
                        new Pose(41.000, 84.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        // Path 3: Intake1 – Straight line forward while intaking
        // Heading: constant 180°
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(41.000, 84.000),
                        new Pose(16.400, 84.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 4: OpenGate – Curve to push first artifact
        // Heading: 180° → 0°
        OpenGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(16.400, 84.000),
                        new Pose(26.145, 79.844),
                        new Pose(27.890, 73.688)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        // Path 5: OpenGate2 – Curve to push second artifact
        // Heading: constant 0°
        OpenGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(27.890, 73.688),
                        new Pose(24.583, 68.599),
                        new Pose(14.578, 69.839)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 6: OpenGate3 – Straight line to push third artifact
        // Heading: constant 0°
        OpenGate3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(14.578, 69.839),
                        new Pose(38.601, 69.677)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 7: Shoot1 – Curve back to scoring position
        // Heading: 0° → 137°
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(38.601, 69.677),
                        new Pose(52.549, 76.133),
                        new Pose(50.000, 93.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(137))
                .build();

        // Path 8: GoTo2 – Curve to second intake approach
        // Heading: 137° → 180°
        GoTo2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.000, 93.000),
                        new Pose(48.205, 72.127),
                        new Pose(40.000, 60.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        // Path 9: Intake2 – Straight line forward while intaking
        // Heading: constant 180°
        Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(40.000, 60.000),
                        new Pose(18.000, 60.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 10: Shoot2 – Curve back to scoring position
        // Heading: 180° → 137°
        Shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(18.000, 60.000),
                        new Pose(41.927, 72.748),
                        new Pose(50.000, 93.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        // Path 11: GoTo3 – Curve to third intake approach
        // Heading: 137° → 180°
        GoTo3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.000, 93.000),
                        new Pose(51.580, 59.225),
                        new Pose(41.000, 35.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        // Path 12: Intake3 – Straight line forward while intaking
        // Heading: constant 180°
        Intake3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(41.000, 35.000),
                        new Pose(18.000, 35.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 13: Shoot3 – Straight line back to scoring position
        // Heading: 180° → 137°
        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(18.000, 35.000),
                        new Pose(50.000, 93.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        // Path 14: OutOfTriangle – Curve to parking position
        // Heading: 137° → 180°
        OutOfTriangle = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.000, 93.000),
                        new Pose(38.165, 88.661),
                        new Pose(23.743, 95.851)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();
    }
}