package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Limelight_camera;
import org.firstinspires.ftc.teamcode.config.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import java.util.List;

/**
 * Robot Configuration WITH Limelight
 *
 * Pipeline Selection:
 * - Blue Alliance: Pipeline 1 (Tag 20)
 * - Red Alliance: Pipeline 2 (Tag 24)
 */
public class Robot_camera {
    public final MecanumDrive drive;
    public final Shooter shooter;
    public final Intake intake;
    public final Limelight_camera limelight;
    public final Follower f;
    private final List<LynxModule> allHubs;

    public Robot_camera(HardwareMap hardwareMap, Alliance alliance) {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        // FIXED: Pipeline 1 = Blue (Tag 20), Pipeline 2 = Red (Tag 24)
        int pipeline = (alliance == Alliance.BLUE) ? 1 : 2;
        limelight = new Limelight_camera(hardwareMap,    pipeline);

        f = Constants.createFollower(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void periodic() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();
        f.update();
        shooter.periodic();
        limelight.periodic();
    }
}