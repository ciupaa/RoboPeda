package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Simple subsystem wrapper around the goBILDA Pinpoint odometry computer.
 *
 * This is separate from Pedro Pathing's PinpointConstants so that:
 * - TeleOp can always read the current pose & heading
 * - Turret alignment can use heading and field position
 *
 * The configuration here should mirror your Pedro localizer constants.
 */
@Config
@Configurable
public class PinpointLocalizerSubsystem extends SubsystemBase {

    private final GoBildaPinpointDriver pinpoint;

    // These should match your Pedro PinpointConstants (inches).
    public static double FORWARD_POD_Y_IN = -4.33;
    public static double STRAFE_POD_X_IN = -5.23;

    // Use the same odometry pod type as Pedro.
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;

    // Encoder directions must match Pedro's localizerConstants.
    public static GoBildaPinpointDriver.EncoderDirection FORWARD_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection STRAFE_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public PinpointLocalizerSubsystem(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        configure();
    }

    private void configure() {
        // Match sample SensorGoBildaPinpoint but in inches using your offsets.
        pinpoint.setOffsets(STRAFE_POD_X_IN, FORWARD_POD_Y_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(FORWARD_ENCODER_DIRECTION, STRAFE_ENCODER_DIRECTION);

        // Reset pose and IMU at start; for TeleOp this gives a clean (0,0,0).
        pinpoint.resetPosAndIMU();
    }

    /**
     * Call once per loop to update pose estimates.
     */
    @Override
    public void periodic() {
        pinpoint.update();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public double getX(DistanceUnit unit) {
        return pinpoint.getPosition().getX(unit);
    }

    public double getY(DistanceUnit unit) {
        return pinpoint.getPosition().getY(unit);
    }

    public double getHeading(AngleUnit unit) {
        return pinpoint.getPosition().getHeading(unit);
    }

    /**
     * Reset to a known field position and heading, for drift correction.
     */
    public void setPose(double x, double y, double headingDeg, DistanceUnit distUnit) {
        Pose2D pose = new Pose2D(distUnit, x, y, AngleUnit.DEGREES, headingDeg);
        pinpoint.setPosition(pose);
    }
}

