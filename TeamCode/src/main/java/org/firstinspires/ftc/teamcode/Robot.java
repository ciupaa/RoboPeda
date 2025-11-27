package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.pedro.Constants; // <--- ADD THIS IMPORT

import java.util.List;

public class Robot {
    // PUBLIC Subsystems
    public final MecanumDrive drive;
    public final Shooter shooter;
    public final Follower f; // PedroPathing Follower
    public Alliance alliance;

    private final List<LynxModule> allHubs;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        // 1. Initialize Custom Subsystems
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // 2. Initialize PedroPathing Follower
        // FIX: Use the method inside Constants to create the follower
        f = Constants.createFollower(hardwareMap);

        // 3. Configure Bulk Caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void periodic() {
        // Clear cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Update Subsystems
        f.update();
        shooter.periodic();
    }
}