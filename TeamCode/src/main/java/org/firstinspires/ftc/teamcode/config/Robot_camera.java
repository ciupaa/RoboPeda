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
 * Use this for camera-based TeleOps
 *
 * Pipeline Selection:
 * - Blue Alliance: Pipeline 1
 * - Red Alliance: Pipeline 2
 */
public class Robot_camera {
    public final MecanumDrive drive;
    public final Shooter shooter;
    public final Intake intake;
    public final Limelight_camera limelight;  // Camera subsystem
    public final Follower f;
    private final List<LynxModule> allHubs;

    /**
     * Constructor with pipeline selection
     * @param hardwareMap Hardware map
     * @param alliance Alliance color (determines pipeline)
     */
    public Robot_camera(HardwareMap hardwareMap, Alliance alliance) {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        // Select pipeline based on alliance
        int pipeline = (alliance == Alliance.BLUE) ? 1 : 2;
        limelight = new Limelight_camera(hardwareMap, pipeline);

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