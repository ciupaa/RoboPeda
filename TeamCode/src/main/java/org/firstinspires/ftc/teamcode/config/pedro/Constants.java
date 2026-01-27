package org.firstinspires.ftc.teamcode.config.pedro;

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

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9)
            // .forwardZeroPowerAcceleration(-38.936)
            // .lateralZeroPowerAcceleration(-65.965)

            // Primary PIDs
            // .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.01, 0))
            // .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.0, 0.018))
            // .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.001, 0.6, 0))

            // Secondary PIDs
            // .useSecondaryHeadingPIDF(true)
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3.0, 0, 0.04, 0))

            // .useSecondaryTranslationalPIDF(true)
            // .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.04, 0.02))

            // .useSecondaryDrivePIDF(true)
            // .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.01, 0.6, 0))
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fata_dreapta")
            .rightRearMotorName("spate_dreapta")
            .leftRearMotorName("spate_stanga")
            .leftFrontMotorName("fata_stanga")

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-1.18110236)
            .strafePodX(5.11811024)

            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            // TRY THESE FIRST - both FORWARD
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
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