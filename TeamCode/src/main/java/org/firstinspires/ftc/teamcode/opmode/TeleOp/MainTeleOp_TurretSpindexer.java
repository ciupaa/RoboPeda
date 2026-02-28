package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.Robot_v2;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

/**
 * New TeleOp for the rebuilt robot:
 * - Mecanum drive
 * - Turreted shooter that auto-aims at the goal using Pinpoint heading/position
 * - Intake + three-slot Spindexer with HuskyLens color IDs
 *
 * This OpMode is intentionally simpler than your current camera-based TeleOp.
 * We can extend it with more automation once the hardware is available.
 */
@Config
@Configurable
@TeleOp(name = "MainTeleOp_TurretSpindexer", group = "Competition")
public class MainTeleOp_TurretSpindexer extends OpMode {

    private Robot_v2 r;

    // Field position of the goal (Pinpoint coordinate frame, inches).
    // You will need to tune these based on how you define (0,0).
    public static double GOAL_X_IN = 0.0;
    public static double GOAL_Y_IN = 72.0;

    // Shooter angle presets (servo position)
    public static double HIGH_PRESET = 0.65;
    public static double LOW_PRESET = 0.70;
    public static double IDLE_PRESET = 1.0;

    @Override
    public void init() {
        r = new Robot_v2(hardwareMap, Alliance.BLUE);
        r.turretShooter.block();
        telemetry.addData("Status", "Initialized (Turret + Spindexer)");
    }

    @Override
    public void loop() {
        r.periodic();

        // ------------------------------------------------------------
        // DRIVING (field-centric can be added later; for now, robot-centric)
        // ------------------------------------------------------------
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        r.drive.driveRobotCentric(x, y, rx);

        // ------------------------------------------------------------
        // TURRET AUTO-AIM
        // ------------------------------------------------------------
        double robotX = r.pinpoint.getX(DistanceUnit.INCH);
        double robotY = r.pinpoint.getY(DistanceUnit.INCH);
        double robotHeadingDeg = r.pinpoint.getHeading(AngleUnit.DEGREES);

        double dx = GOAL_X_IN - robotX;
        double dy = GOAL_Y_IN - robotY;
        double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));

        r.turretShooter.aimUsingRobotHeading(robotHeadingDeg, targetHeadingDeg);

        // ------------------------------------------------------------
        // SHOOTER CONTROL
        // Right bumper: high shot; Left bumper: low shot
        // ------------------------------------------------------------
        boolean highHeld = gamepad1.right_bumper;
        boolean lowHeld = gamepad1.left_bumper;
        boolean shootHeld = highHeld || lowHeld;

        if (shootHeld) {
            double targetAngle = highHeld ? HIGH_PRESET : LOW_PRESET;
            r.turretShooter.setAngle(targetAngle);

            if (highHeld) r.turretShooter.spinHigh();
            else r.turretShooter.spinLow();

            r.turretShooter.unblock();
            r.intake.intake();
        } else {
            r.turretShooter.block();
            r.turretShooter.stopFlywheels();
        }

        // ------------------------------------------------------------
        // INTAKE / SPINDEXER CONTROL (very simple for now)
        // X: intake on; B: stop; Y: advance to next slot and scan color
        // ------------------------------------------------------------
        if (gamepad1.x) {
            r.intake.intake();
        } else if (gamepad1.b) {
            r.intake.stop();
        }

        // Advance spindexer and scan color under HuskyLens
        if (gamepad1.y) {
            r.spindexer.advanceToNextSlot();
            int colorId = r.huskyLens.getDominantColorId();
            if (colorId != 0) {
                r.spindexer.setCurrentSlotColorId(colorId);
            }
        }

        // ------------------------------------------------------------
        // TELEMETRY
        // ------------------------------------------------------------
        telemetry.addData("Pose (in)", "x=%.1f y=%.1f heading=%.1f",
                robotX, robotY, robotHeadingDeg);
        telemetry.addData("TurretDeg", "%.1f", r.turretShooter.getTurretAngleDegrees());
        telemetry.addData("SpindexerSlot", r.spindexer.getCurrentSlotIndex());
        telemetry.update();
    }
}

