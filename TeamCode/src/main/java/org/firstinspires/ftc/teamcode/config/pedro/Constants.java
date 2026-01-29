package org.firstinspires.ftc.teamcode.config.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Configurable
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9)
            .forwardZeroPowerAcceleration(-38.44246)
            .lateralZeroPowerAcceleration(-54.74851)

            // Primary PIDs
            .headingPIDFCoefficients(new PIDFCoefficients(0.80, 0, 0.04, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.0325, 0.018))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.001, 0.6, 0))

            // Secondary PIDs
            .useSecondaryHeadingPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3.0, 0, 0.04, 0))

            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.04, 0.02))

            .useSecondaryDrivePIDF(true)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.01, 0.6, 0))
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(78.32155)
            .yVelocity(54.50742)
            .rightFrontMotorName("fata_dreapta")
            .rightRearMotorName("spate_dreapta")
            .leftRearMotorName("spate_stanga")
            .leftFrontMotorName("fata_stanga")

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.33)
            .strafePodX(-5.23)

            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)

            // TRY THESE FIRST - both FORWARDkk
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}