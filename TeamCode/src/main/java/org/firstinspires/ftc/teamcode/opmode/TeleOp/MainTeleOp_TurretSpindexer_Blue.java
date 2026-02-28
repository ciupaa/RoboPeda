package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.Robot_v2;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.Motif;
import org.firstinspires.ftc.teamcode.config.util.MotifScanHelper;
import org.firstinspires.ftc.teamcode.config.util.ShooterCalculatorVelocityComp;

/**
 * Blue alliance TeleOp: turret tracks goal, velocity-compensated launch.
 * Goal position is set for the BLUE side (mirrored from Red). Typically Y is negated.
 */
@Config
@Configurable
@TeleOp(name = "MainTeleOp Turret+Spindexer BLUE", group = "Competition")
public class MainTeleOp_TurretSpindexer_Blue extends OpMode {

    private Robot_v2 r;

    /** Goal position in field coords (inches). Tune for BLUE alliance (mirrored from Red). */
    public static double GOAL_X_IN = 0.0;
    public static double GOAL_Y_IN = -72.0;
    /** Height of goal center above the shooter launch point (inches). */
    public static double GOAL_HEIGHT_ABOVE_LAUNCH_IN = 24.0;

    @Override
    public void init() {
        r = new Robot_v2(hardwareMap, Alliance.BLUE);
        r.turretShooter.block();
        telemetry.addData("Status", "Scanning AprilTag for motif...");
        telemetry.update();
        Motif motif = MotifScanHelper.runMotifScan(r);
        telemetry.addData("Status", "Initialized BLUE | Motif: %s", motif);
    }

    @Override
    public void loop() {
        r.periodic();

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        r.drive.driveRobotCentric(x, y, rx);

        double robotX = r.pinpoint.getX(DistanceUnit.INCH);
        double robotY = r.pinpoint.getY(DistanceUnit.INCH);
        double robotHeadingDeg = r.pinpoint.getHeading(AngleUnit.DEGREES);
        double[] vel = r.getRobotVelocityInPerSec();
        double vx = vel[0], vy = vel[1];

        ShooterCalculatorVelocityComp.ShotResult shot = ShooterCalculatorVelocityComp.computeShot(
                robotX, robotY,
                GOAL_X_IN, GOAL_Y_IN,
                GOAL_HEIGHT_ABOVE_LAUNCH_IN,
                vx, vy);

        double dx = GOAL_X_IN - robotX;
        double dy = GOAL_Y_IN - robotY;
        double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));

        r.turretShooter.aimWithVelocityCompensation(
                robotHeadingDeg, targetHeadingDeg, shot.turretOffsetDeg);
        r.turretShooter.setAngle(shot.hoodServoPosition);
        r.turretShooter.setFlywheelVelocity(shot.flywheelVelocity);

        boolean shootHeld = gamepad1.right_bumper || gamepad1.left_bumper;
        if (shootHeld) {
            r.turretShooter.unblock();
            r.intake.intake();
        } else {
            r.turretShooter.block();
            r.turretShooter.stopFlywheels();
        }

        if (gamepad1.x) r.intake.intake();
        else if (gamepad1.b) r.intake.stop();

        if (gamepad1.y) {
            r.spindexer.advanceToNextSlot();
            int colorId = r.huskyLens.getDominantColorId();
            if (colorId != 0) r.spindexer.setCurrentSlotColorId(colorId);
        }

        telemetry.addData("Pose (in)", "x=%.1f y=%.1f hdg=%.1f", robotX, robotY, robotHeadingDeg);
        telemetry.addData("Vel comp", shot.usedVelocityCompensation ? "ON" : "OFF");
        telemetry.addData("Hood", "%.2f Flywheel %.0f", shot.hoodServoPosition, shot.flywheelVelocity);
        telemetry.addData("Turret offset (deg)", "%.1f", shot.turretOffsetDeg);
        telemetry.addData("Motif", r.spindexer.getMotif() != null ? r.spindexer.getMotif().name() : "?");
        telemetry.addData("Spindexer slot", r.spindexer.getCurrentSlotIndex());
        telemetry.update();
    }
}
