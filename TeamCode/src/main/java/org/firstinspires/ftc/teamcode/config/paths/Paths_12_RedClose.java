package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.config.Robot;

/**
 * FILE: Paths_12_RedClose.java
 * PURPOSE: 12-Point Autonomous paths for RED ALLIANCE, CLOSE side.
 *
 * Mirrored version of Paths_12_BlueClose with a third intake/shoot cycle.
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
public class Paths_12_RedClose {

    // Robot starting pose (RED CLOSE)
    public Pose startPose = new Pose(118.752, 128.587, Math.toRadians(0));

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

    public Paths_12_RedClose(Follower follower) {

        // Path 1: ShootPreload – Curve from start to scoring position
        // Heading: 0° → 43°
        ShootPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(118.752, 128.587),
                        new Pose(99.890, 117.771),
                        new Pose(94.000, 93.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        // Path 2: GoTo1 – Curve to first intake approach
        // Heading: 43° → 0°
        GoTo1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(94.000, 93.000),
                        new Pose(94.394, 85.606),
                        new Pose(103.000, 84.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        // Path 3: Intake1 – Straight line forward while intaking
        // Heading: constant 0°
        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(103.000, 84.000),
                        new Pose(127.600, 84.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 4: OpenGate – Curve to push first artifact
        // Heading: 0° → 180°
        OpenGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(127.600, 84.000),
                        new Pose(117.855, 79.844),
                        new Pose(116.110, 73.688)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        // Path 5: OpenGate2 – Curve to push second artifact
        // Heading: constant 180°
        OpenGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(116.110, 73.688),
                        new Pose(119.417, 68.599),
                        new Pose(129.422, 69.839)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 6: OpenGate3 – Straight line to push third artifact
        // Heading: constant 180°
        OpenGate3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(129.422, 69.839),
                        new Pose(105.399, 69.677)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 7: Shoot1 – Curve back to scoring position
        // Heading: 180° → 43°
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(105.399, 69.677),
                        new Pose(91.451, 76.133),
                        new Pose(94.000, 93.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(43))
                .build();

        // Path 8: GoTo2 – Curve to second intake approach
        // Heading: 43° → 0°
        GoTo2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(94.000, 93.000),
                        new Pose(95.795, 72.127),
                        new Pose(104.000, 60.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        // Path 9: Intake2 – Straight line forward while intaking
        // Heading: constant 0°
        Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(104.000, 60.000),
                        new Pose(126.000, 60.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 10: Shoot2 – Curve back to scoring position
        // Heading: 0° → 43°
        Shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(126.000, 60.000),
                        new Pose(102.073, 72.748),
                        new Pose(94.000, 93.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        // Path 11: GoTo3 – Curve to third intake approach
        // Heading: 43° → 0°
        GoTo3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(94.000, 93.000),
                        new Pose(92.420, 59.225),
                        new Pose(103.000, 35.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        // Path 12: Intake3 – Straight line forward while intaking
        // Heading: constant 0°
        Intake3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(103.000, 35.000),
                        new Pose(126.000, 35.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 13: Shoot3 – Straight line back to scoring position
        // Heading: 0° → 43°
        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(126.000, 35.000),
                        new Pose(94.000, 93.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        // Path 14: OutOfTriangle – Curve to parking position
        // Heading: 43° → 0°
        OutOfTriangle = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(94.000, 93.000),
                        new Pose(105.835, 88.661),
                        new Pose(120.257, 95.851)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();
    }
}