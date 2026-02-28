package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystem.HuskyLensSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystem.PinpointLocalizerSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.Spindexer;
import org.firstinspires.ftc.teamcode.config.subsystem.TurretedShooter;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

import java.util.List;

/**
 * New robot configuration for the rebuilt robot:
 * - Mecanum drive
 * - Turreted shooter (two flywheels + turret)
 * - Intake + three-slot Spindexer
 * - Pinpoint odometry (Pedro + direct TeleOp access)
 * - HuskyLens color sensor for spindexer slots
 *
 * Existing Robot / Robot_camera remain unchanged.
 */
public class Robot_v2 {
    public final MecanumDrive drive;
    public final TurretedShooter turretShooter;
    public final Intake intake;
    public final Spindexer spindexer;
    public final PinpointLocalizerSubsystem pinpoint;
    public final HuskyLensSubsystem huskyLens;
    public final Follower f;

    private final List<LynxModule> allHubs;

    public Robot_v2(HardwareMap hardwareMap, Alliance alliance) {
        drive = new MecanumDrive(hardwareMap);
        turretShooter = new TurretedShooter(hardwareMap);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        pinpoint = new PinpointLocalizerSubsystem(hardwareMap);
        huskyLens = new HuskyLensSubsystem(hardwareMap);

        f = Constants.createFollower(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void periodic() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Pedro Follower + Pinpoint + TurretedShooter dashboard PID updates.
        f.update();
        pinpoint.periodic();
        turretShooter.periodic();
    }

    /**
     * Robot velocity in field frame (in/s). From Pedro localizer.
     * Returns [vx, vy]. If unavailable, returns [0, 0].
     */
    public double[] getRobotVelocityInPerSec() {
        try {
            var vel = f.getPoseTracker().getLocalizer().getVelocity();
            double vx = vel.getX();
            double vy = vel.getY();
            return new double[]{vx, vy};
        } catch (Throwable t) {
            try {
                var vel = f.getVelocity();
                double vx = vel.getXComponent();
                double vy = vel.dot(new Vector(1, Math.PI / 2));
                return new double[]{vx, vy};
            } catch (Throwable t2) {
                return new double[]{0, 0};
            }
        }
    }
}

