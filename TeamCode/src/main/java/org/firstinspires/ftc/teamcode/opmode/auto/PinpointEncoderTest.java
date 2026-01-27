package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint Encoder Test", group = "Diagnostics")
public class PinpointEncoderTest extends OpMode {

    private GoBildaPinpointDriver odo;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set offsets WITH DistanceUnit
        odo.setOffsets(-1.18110236, 5.11811024, DistanceUnit.INCH);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Ready to test encoders!");
        telemetry.update();
    }

    @Override
    public void loop() {
        odo.update();

        // Get position using the correct method
        Pose2D pos = odo.getPosition();

        telemetry.addLine("=== RAW ENCODER COUNTS ===");
        telemetry.addData("Forward Encoder", odo.getEncoderX());
        telemetry.addData("Strafe Encoder", odo.getEncoderY());

        telemetry.addLine("");
        telemetry.addLine("=== POSITION (INCHES) ===");
        telemetry.addData("X Position", pos.getX(DistanceUnit.INCH));
        telemetry.addData("Y Position", pos.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));

        telemetry.addLine("");
        telemetry.addLine("=== INSTRUCTIONS ===");
        telemetry.addLine("1. Push robot FORWARD");
        telemetry.addLine("   → Forward Encoder should change");
        telemetry.addLine("");
        telemetry.addLine("2. Push robot LEFT (strafe)");
        telemetry.addLine("   → Strafe Encoder should change");
        telemetry.addLine("");
        telemetry.addData("Press A", "to reset position");

        if (gamepad1.a) {
            odo.resetPosAndIMU();
        }

        telemetry.update();
    }
}