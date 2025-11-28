package org.firstinspires.ftc.teamcode.config.pedro;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7.50)
            .forwardZeroPowerAcceleration(-45.343)
            //.lateralZeroPowerAcceleration(-67.58)
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3, 0, .04, 0))
            //.headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.015,0,0.003,0))
            //.useSecondaryDrivePIDF(true)
            //.useSecondaryHeadingPIDF(true)
            //.useSecondaryTranslationalPIDF(true)
            ;

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            .xVelocity(82.6278)
            .yVelocity(49.1836)

            // --- MOTOR NAMES (Romanian) ---
            .leftFrontMotorName("fata_stanga")
            .rightFrontMotorName("fata_dreapta")
            .leftRearMotorName("spate_stanga")
            .rightRearMotorName("spate_dreapta")

            // --- FIX: DIRECTIONS FLIPPED ---
            // Left = FORWARD, Right = REVERSE
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(13)
            .strafePodX(13)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.975, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}